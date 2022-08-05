'''
Throw using a hand-designed trajectory.
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

    # Hand-create a throw trajectory.
    throw_duration = 0.25
    throw_center = np.array([90., 90., 90., 90., 90., open_angle])
    max_vel = 60/.2
    throw_velocity = np.array([0., max_vel/2., max_vel, max_vel, 0., 0.])
    N_pts = 64
    throw_ts = np.linspace(0., throw_duration, N_pts)
    throw_qs = np.stack(
        [throw_center + np.sin( (t - throw_duration/2.) / (throw_duration/2.)) * (throw_duration/2.) * throw_velocity for t in throw_ts],
         axis=0
    )
    t_release = throw_duration / 2.
    t_release_offset = 0.2
    throw_qs[throw_ts <= t_release + t_release_offset, 5] = closed_angle
    throw_qs[throw_ts > t_release + t_release_offset, 5] = open_angle
    if (t_release_offset + t_release) > throw_ts[-1]:
        throw_ts = np.r_[throw_ts, t_release_offset + t_release]
        throw_qs = np.concatenate([throw_qs, throw_qs[-1:, :]], axis=0)
        throw_qs[-1, 5] = open_angle
        print("Appended")

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

