import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import serial
import time
import multiprocessing as mp
from threading import Lock
import os

from util import pack_trajectory_to_buf

throw_traj_file = os.path.join("trajectories", "throw_trajectory.npz")


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

    # Load up and prep the actual throw trajectory.
    dat = np.load(throw_traj_file)
    throw_ts = dat["ts"]
    throw_xs = dat["xs"]
    # Convert to degrees and throw out velocities.
    throw_qs = throw_xs[:, :6] * 180. / np.pi

    # Create a move-to-pre-throw trajectory.
    pregrasp = np.array([147, 80, 45, 0., 0., 66])
    grasp = np.array([147, 62, 35, 0., 0., 50])
    closed = np.array([147, 62, 35, 0., 0., 120])
    pregrasp_closed = np.array([147, 80, 45, 0., 0., 120])
    prethrow_ts = []
    prethrow_qs = []
    add_setpoint(prethrow_ts, prethrow_qs, 3., pregrasp)
    add_setpoint(prethrow_ts, prethrow_qs, 1., grasp)
    add_setpoint(prethrow_ts, prethrow_qs, 0.5, closed)
    add_setpoint(prethrow_ts, prethrow_qs, 1., pregrasp_closed)
    prethrow_ts = np.array(prethrow_ts)
    prethrow_qs = np.stack(prethrow_qs, axis=0)

    # Glue these trajectories together.
    ts = np.concatenate(
        [prethrow_ts, throw_ts + prethrow_ts[-1] + 3.], axis=0
    )
    qs = np.r_[prethrow_qs, throw_qs]

    print(ts)
    # Wait a bit before sending.
    time.sleep(0.5)

    ser.write(pack_trajectory_to_buf(ts, qs))
    ser.write(b"dummy") # Need to write this to "flush" serial? Something weird going on.
    t_start = time.time()
    while (time.time() - t_start < ts[-1] + 2.):
        if (ser.inWaiting() > 0):
            # read the bytes and convert from binary array to ASCII
            data_str = ser.read(ser.inWaiting()).decode('ascii') 
            # print the incoming string without putting a new-line
            # ('\n') automatically after every print()
            print(data_str, end='') 
        time.sleep(0.1)

