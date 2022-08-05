'''
Throw a paper airplane using trajopt.
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import serial
import time
import multiprocessing as mp
from threading import Lock
import os

from util import pack_trajectory_to_buf

throw_traj_file = os.path.join("trajectories", "throw_trajectory_plane.npz")


def add_setpoint(ts, qs, dt_from_last, q):
    # Modify ts and qs in-place to add a setpoint `q` `dt_from_last` after
    # the current end of the trajectory.
    if len(ts) == 0:
        ts.append(dt_from_last)
    else:
        ts.append(ts[-1] + dt_from_last)
    qs.append(q)


if __name__ == "__main__":
    alive = True
    ser = serial.Serial(port='COM11', baudrate=9600, timeout=.1)
    time.sleep(0.5)

    # Angle of gripper to open/close
    open_angle = 80
    closed_angle = 120

    # Load up and prep the actual throw trajectory.
    dat = np.load(throw_traj_file)
    throw_ts = dat["ts"]
    throw_xs = dat["xs"]
    t_release = dat["t_throw"]
    
    # Convert to degrees and throw out velocities.
    throw_qs = throw_xs[:, :6] * 180. / np.pi
    # Fix gripper state, which isn't part of trajopt.
    # This time offset delays release a little; this
    # compensates for tracking delay.
    t_release_offset = 0
    throw_qs[throw_ts <= t_release + t_release_offset, 5] = closed_angle
    throw_qs[throw_ts > t_release + t_release_offset, 5] = open_angle

    # Create a move-to-pre-throw trajectory.
    base_angle = 90.
    pregrasp = np.array([base_angle, 90., 175, 175, 91, open_angle])
    grasp = np.array([base_angle, 106., 176., 179., 91., open_angle])
    closed = np.array([base_angle, 106., 176., 179., 91., closed_angle])
    pregrasp_closed = np.array([base_angle, 90., 175, 175, 91, closed_angle])
    prethrow_ts = []
    prethrow_qs = []
    add_setpoint(prethrow_ts, prethrow_qs, 1., pregrasp)
    add_setpoint(prethrow_ts, prethrow_qs, 0.5, grasp)
    add_setpoint(prethrow_ts, prethrow_qs, 0.25, closed)
    add_setpoint(prethrow_ts, prethrow_qs, 0.5, pregrasp_closed)
    prethrow_ts = np.array(prethrow_ts)
    prethrow_qs = np.stack(prethrow_qs, axis=0)

    # Glue these trajectories together.
    ts = np.concatenate(
        [prethrow_ts, throw_ts + prethrow_ts[-1] + 1.], axis=0
    )
    qs = np.r_[prethrow_qs, throw_qs]

    ser.write(pack_trajectory_to_buf(ts, qs))
    ser.write(b"dummy") # Need to write this to "flush" serial? Something weird going on.
    t_start = time.time()
    while (time.time() - t_start < ts[-1] + 0.5):
        if (ser.inWaiting() > 0):
            # read the bytes and convert from binary array to ASCII
            data_str = ser.read(ser.inWaiting()).decode('ascii') 
            # print the incoming string without putting a new-line
            # ('\n') automatically after every print()
            print(data_str, end='') 
        time.sleep(0.1)

