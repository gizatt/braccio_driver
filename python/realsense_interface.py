'''
Code created, in large part, with help from GPT-4, so dubious licenses apply. Don't use this code
in a license-critical context, this is personal sandbox code.
'''

# Work around some conda-related environment issue
from slingshot_tester import get_joint_angles, pack_trajectory_to_buf
from torchvision.transforms import functional as F
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn
import cv2
import sys
import torch
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import os
import time
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"


class VisionManager:
    '''
        Manages realsense with streaming person detector.
    '''
    SCORE_THRESHOLD = 0.9

    def __init__(self):
        # Load pre-trained model from torchvision
        self.model = fasterrcnn_mobilenet_v3_large_320_fpn(pretrained=True).cuda()
        self.model = self.model.eval()  # Set the model to evaluation mode
        self.boxes_with_scores = []

        # Initialize and configure the realsense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get a list of connected devices
        ctx = rs.context()
        devices = ctx.query_devices()

        if devices:
            # Enable the first device found
            config.enable_device(devices[0].get_info(
                rs.camera_info.serial_number))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start the pipeline
        self.pipeline.start(config)

    def __del__(self):
        self.pipeline.stop()

    def update(self):
        frames = self.pipeline.wait_for_frames()
        self.depth_image = np.asanyarray(
            frames.get_depth_frame().get_data()).astype(float) / 1000.
        self.depth_image = np.ascontiguousarray(np.rot90(self.depth_image))
        self.color_image = np.asanyarray(frames.get_color_frame().get_data())[
            :, :, ::-1]  # BGR -> RGB
        self.color_image = np.ascontiguousarray(np.rot90(self.color_image))

        # Perform person detection
        image_tensor = F.to_tensor(self.color_image).unsqueeze(0).cuda()
        predictions = self.model(image_tensor)

        boxes = predictions[0]["boxes"][predictions[0]["labels"] == 1].tolist()
        scores = predictions[0]["scores"][predictions[0]
                                          ["labels"] == 1].tolist()
        self.boxes_with_scores = [
            (box, score) for box, score in zip(boxes, scores) if score > self.SCORE_THRESHOLD
        ]
        # Sort by descending score
        self.boxes_with_scores.sort(key=lambda x: -x[1])


class TargetTracker():
    def __init__(self, vision_manager: VisionManager):
        self.vision_manager = vision_manager
        self.reset()

    def reset(self):
        self.tracked_box = None
        self.tracked_pt = None
        self.median_depth = None

    def update(self):
        if len(self.vision_manager.boxes_with_scores) == 0:
            self.reset()
            return

        def get_center(box):
            x1, y1, x2, y2 = map(int, box)
            center = np.array([(x1 + x2) // 2, (y1 + y2) // 2])
            return center

        if self.tracked_pt is None:
            box, score = self.vision_manager.boxes_with_scores[0]
            self.tracked_box = box
            self.tracked_pt = get_center(box)
        else:
            min_dist = np.inf
            closest_pt = None
            closest_box = None
            for box, score in self.vision_manager.boxes_with_scores:
                this_pt = get_center(box)
                this_dist = np.linalg.norm(this_pt - self.tracked_pt)
                if this_dist < min_dist:
                    closest_box = box
                    closest_pt = this_pt
                    min_dist = this_dist
            self.tracked_box = box
            self.tracked_pt = closest_pt

        # Calculate the median depth value within the bounding box of this best box
        x1, y1, x2, y2 = map(int, self.tracked_box)
        depth_box = self.vision_manager.depth_image[y1:y2, x1:x2].copy()
        # Make out-of-range values nan
        depth_box[depth_box < 0.5] = np.nan
        depth_box[depth_box > 5.0] = np.nan
        self.median_depth = np.nanmedian(depth_box)


class RobotController():
    UPDATE_PERIOD = 0.05

    def __init__(self, port: str, target_tracker: TargetTracker):
        self.last_update_t = time.time() + 1.
        self.target_tracker = target_tracker
        self.seek_direction = 1.

        if port is None:
            print("No port supplied, running with no robot.")
            self.ser = None
        else:
            import serial  # Weird error if I import this in the header... maybe dependency declares 'serial' in some weird way?
            try:
                self.ser = serial.Serial(port=port, baudrate=9600, timeout=.1)
            except serial.SerialException as e:
                print(e)
                print("Available ports:")
                import serial.tools.list_ports
                for port in serial.tools.list_ports.comports(include_links=False):
                    print(port)
                sys.exit(0)

        self.go_to_q(t=1., q=get_joint_angles(throw_yaw=np.deg2rad(90.), throw_elevation=np.deg2rad(60.),
                     pin_to_hand_distance=0.22, theta_release=-np.pi/4.))


        self.target_q = self.last_q.copy()
        self.tracking_mode = "auto"

    def go_to_q(self, t: float, q: np.ndarray):
        if self.ser is not None:
            print(f"Going to {q}")
            self.ser.write(pack_trajectory_to_buf(
                np.array([t]), np.rad2deg(q).reshape(1, 6)))
            # Need to write this to "flush" serial? Something weird going on.
            self.ser.write(b"dummy")
        self.last_q = q

    def update(self):
        if self.ser is None:
            return
        if (self.ser.inWaiting() > 0):
            # read the bytes and convert from binary array to ASCII
            data_str = self.ser.read(self.ser.inWaiting()).decode('ascii')
            # print the incoming string without putting a new-line
            # ('\n') automatically after every print()
            print(data_str, end='')

        t = time.time()
        if t - self.last_update_t > self.UPDATE_PERIOD:
            seek_time = self.UPDATE_PERIOD
            if self.tracking_mode == "auto":
                if self.target_tracker.tracked_pt is not None:
                    print("In auto mode")
                    # Right-shifted a bit to account
                    fractional_image_err = (self.target_tracker.tracked_pt[0] - 300) / 240.
                    self.target_q[0] += fractional_image_err * 0.2
                    target_elevation, pin_to_hand_distance = self.get_target_elevation_and_pin_to_hand_distance()
                    self.target_q = get_joint_angles(self.target_q[0], target_elevation, pin_to_hand_distance, -3.*np.pi/8.)
                    print(self.target_tracker.tracked_pt)
                else:
                    print("No lock, going to seek.")
                    self.tracking_mode = "seek"
            elif self.tracking_mode == "seek":
                if self.target_tracker.tracked_pt is not None:
                    self.tracking_mode = "auto"
                else:
                    self.target_q[0] += self.seek_direction * np.pi / 8
                    if self.target_q[0] > np.pi:
                        self.seek_direction = -1.
                    elif self.target_q[0] < 0.:
                        self.seek_direction = 1.
                    seek_time = 2. * np.pi/8.
                
            self.go_to_q(seek_time, self.target_q)
            self.last_update_t = t + seek_time - self.UPDATE_PERIOD

    def get_target_elevation_and_pin_to_hand_distance(self):
        if self.target_tracker.median_depth is None:
            return np.deg2rad(60), 0.2
        
        #elevation = np.deg2rad( - 15 * min(3., self.target_tracker.median_depth) / 3.) # at 3meters, 45 degrees
        fractional_elevation_from_center = -(self.target_tracker.tracked_pt[1] - 100) / 320.
        elevation = np.deg2rad(30. + fractional_elevation_from_center * 30)
        speed = 0.22 + 0.02 * np.clip(self.target_tracker.median_depth / 3., 0., 1.)

        print(f"At range {self.target_tracker.median_depth}, using elevation {elevation} and pin-to-hand {speed}")
        return elevation, speed

    def fire(self):
        ts = []
        qs = []

        target_elevation = np.deg2rad(60)
        pin_to_hand_distance = 0.24

        ts = np.linspace(1, 3., 10)
        theta_release_center = -np.pi/8
        theta_releases = theta_release_center + np.linspace(-np.pi/4., np.pi/8, len(ts))
        target_elevation, pin_to_hand_distance = self.get_target_elevation_and_pin_to_hand_distance()
        for theta_release in theta_releases:
            qs.append(
                get_joint_angles(self.target_q[0], target_elevation,
                                pin_to_hand_distance, theta_release)
            )

        # After firing, reset back to initial pose.
        ts = np.r_[ts, ts[-1] + 1.]
        qs.append(qs[0])

        qs = np.stack(qs)
        print("Firing: ", ts, np.rad2deg(qs))
        if self.ser is not None:
            self.ser.write(pack_trajectory_to_buf(ts, np.rad2deg(qs)))
            self.ser.write(b"dummy") # Need to write this to "flush" serial? Something weird going on.
        
        # Defer update till after we complete the firing process
        self.last_update_t = time.time() + ts[-1]

    def on_press(self, event):
        print('press', event.key)
        if event.key == "a":
            self.target_q[0] -= 0.2
        elif event.key == "d":
            self.target_q[0] += 0.2
        elif event.key == "r":
            self.target_tracker.reset()
            print(f"Tracking reset")
        elif event.key == "t": # spacebar swaps tracking modes
            if self.tracking_mode == "auto":
                self.tracking_mode = "seek"
            elif self.tracking_mode == "seek":
                self.tracking_mode = "manual"
            else:
                self.tracking_mode = "auto"
            print(f"Tracking mode set to {self.tracking_mode}")
        elif event.key == " ":
            self.fire()

if __name__ == "__main__":
    vision_manager = VisionManager()
    vision_manager.update()
    target_tracker = TargetTracker(vision_manager)
    robot_controller = RobotController(port="COM4", target_tracker=target_tracker)

    # Create a figure for the subplots
    fig, axs = plt.subplots(1, 2)
    fig.canvas.mpl_connect('key_press_event', robot_controller.on_press)

    # Enable interactive mode
    plt.ion()

    # Create initial plots
    depth_plot = axs[0].imshow(
        vision_manager.depth_image, cmap=plt.cm.get_cmap('RdBu'))
    # position of the colorbar (left, bottom, width, height)
    cbar_ax = fig.add_axes([0.125, 0.05, 0.35, 0.03])
    fig.colorbar(depth_plot, cax=cbar_ax, orientation='horizontal')
    color_plot = axs[1].imshow(vision_manager.color_image)

    # Update the plots in a loop
    try:
        while True:
            vision_manager.update()
            target_tracker.update()
            robot_controller.update()

            color_image = vision_manager.color_image.copy()
            depth_image = vision_manager.depth_image.copy()

            for box, score in vision_manager.boxes_with_scores:
                cv2.rectangle(color_image, (int(box[0]), int(
                    box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                cv2.putText(color_image, f"{score:.2f}", (int(box[0]), int(
                    box[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            if target_tracker.tracked_pt is not None:
                cv2.drawMarker(depth_image, target_tracker.tracked_pt, (0, 0, 255),
                               markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
                cv2.putText(depth_image, f"{target_tracker.median_depth:.2f}m", (
                    target_tracker.tracked_pt[0], target_tracker.tracked_pt[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

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
