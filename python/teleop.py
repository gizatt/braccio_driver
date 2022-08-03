import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import serial
import time
import multiprocessing as mp
from threading import Lock

from util import pack_trajectory_to_buf

SLEW_TIME = 0.1

if __name__ == "__main__":
    alive = True
    ser = serial.Serial(port='COM11', baudrate=9600, timeout=.1)

    # Create the figure with sliders that we'll manipulate.
    fig = plt.figure()
    sliders = []
    for k in range(6):
        sliders.append(
            Slider(
                ax = plt.subplot(6, 1, k+1),
                label=f"M{k}",
                valmin=-90,
                valmax=270.,
                valinit=90.
            )
        )

    # The function to be called anytime a slider's value changes
    def update(val):
        if ser is None:
            print("No serial!")
            return
        qs = np.array([slider.val for slider in sliders]).reshape(1, 6)
        ts = np.array([SLEW_TIME])
        print("Sending to ", qs)
        ser.write(pack_trajectory_to_buf(ts, qs))

        if (ser.inWaiting() > 0):
            # read the bytes and convert from binary array to ASCII
            data_str = ser.read(ser.inWaiting()).decode('ascii') 
            # print the incoming string without putting a new-line
            # ('\n') automatically after every print()
            print(data_str, end='') 

    # register the update function with each slider
    for slider in sliders:
        slider.on_changed(update)

    plt.show()