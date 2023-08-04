'''
Code created, in large part, with help from GPT-4, so dubious licenses apply. Don't use this code
in a license-critical context, this is personal sandbox code.
'''

# Work around some conda-related environment issue
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

import matplotlib.pyplot as plt
import numpy as np
import pyrealsense2 as rs
import torch
import cv2
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn
from torchvision.transforms import functional as F

# Load pre-trained model from torchvision
model = fasterrcnn_mobilenet_v3_large_320_fpn(pretrained=True)
model = model.eval()  # Set the model to evaluation mode

# Initialize and configure the realsense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Get a list of connected devices
ctx = rs.context()
devices = ctx.query_devices()

if devices:
    # Enable the first device found
    config.enable_device(devices[0].get_info(rs.camera_info.serial_number))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

# Create a figure for the subplots
fig, axs = plt.subplots(1, 2)

# Enable interactive mode
plt.ion()

# Create initial plots
frames = pipeline.wait_for_frames()
depth_image = np.asanyarray(frames.get_depth_frame().get_data())
color_image = np.ascontiguousarray(np.asanyarray(frames.get_color_frame().get_data())[:, :, ::-1])  # BGR -> RGB
depth_plot = axs[0].imshow(depth_image, cmap=plt.cm.get_cmap('RdBu'))
cbar_ax = fig.add_axes([0.125, 0.05, 0.35, 0.03]) # position of the colorbar (left, bottom, width, height)
fig.colorbar(depth_plot, cax=cbar_ax, orientation='horizontal')
color_plot = axs[1].imshow(color_image)

# Update the plots in a loop
try:
    while True:
        # Get the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Get the depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Update the data of the plots
        depth_image = np.asanyarray(depth_frame.get_data()) / 1000. # to meters from mm
        color_image = np.ascontiguousarray(np.asanyarray(color_frame.get_data())[:, :, ::-1])  # BGR -> RGB

        # Apply object detection
        image_tensor = F.to_tensor(color_image).unsqueeze(0)
        predictions = model(image_tensor)

        # Draw bounding boxes around detected persons (label == 1)
        boxes = predictions[0]["boxes"][predictions[0]["labels"] == 1].tolist()
        scores = predictions[0]["scores"][predictions[0]["labels"] == 1].tolist()

        if scores:
            max_score_idx = scores.index(max(scores))
            max_score_box = boxes[max_score_idx]

            for i, box in enumerate(boxes):
                score = scores[i]
                # Draw all detections that are reasonably good
                if score > 0.5:
                    cv2.rectangle(color_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{score:.2f}", (int(box[0]), int(box[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                # For the best one, compute some additional detailed info.
                if i == max_score_idx:
                    # Calculate the median depth value within the bounding box
                    x1, y1, x2, y2 = map(int, max_score_box)
                    depth_box = depth_image[y1:y2, x1:x2].copy()
                    # Make out-of-range values nan
                    #depth_box[np.logical_or(depth_box < 0.5, depth_box > 5.0)] = np.nan
                    median_depth = np.nanmedian(depth_box)

                    # Annotate the median depth value on the depth image
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.drawMarker(depth_image, (center_x, center_y), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
                    cv2.putText(depth_image, f"{median_depth:.2f}m", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        depth_plot.set_data(depth_image)
        axs[0].images[0].set_clim(vmin=0.0, vmax=5.0)
        color_plot.set_data(color_image)
        plt.tight_layout()

        # Redraw the plots
        plt.draw()
        plt.pause(0.01)
except KeyboardInterrupt:
    # If the user hits Ctrl+C, exit the loop
    pass
finally:
    # Clean up the pipeline when we're done
    pipeline.stop()