import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import serial
import time
import multiprocessing as mp
from threading import Lock

from util import pack_trajectory_to_buf

SLEW_TIME = 2.0

if __name__ == "__main__":
    alive = True
    ser = serial.Serial(port='COM11', baudrate=9600, timeout=.1)

    # Spool out throwing setpoints.
    pregrasp = np.array([147, 80, 45, 0., 0., 66])
    grasp = np.array([147, 62, 35, 0., 0., 50])
    closed = np.array([147, 62, 35, 0., 0., 120])
    pregrasp_closed = np.array([147, 80, 45, 0., 0., 120])

    pre_throw_stance = np.array([55, 145, 45, 0, 90, 120])
    pre_throw_windup = np.array([55, 170, 90, 90, 90, 120])
    mid_throw = np.array([55, 90, 90, 90, 90, 120])
    post_throw_extended = np.array([55, 70, 90, 90, 90, 40])

    ts = []
    qs = []

    def add_setpoint(dt_from_last, q):
        if len(ts) == 0:
            ts.append(dt_from_last)
        else:
            ts.append(ts[-1] + dt_from_last)
        qs.append(q)

    add_setpoint(3., pre_throw_stance)
    add_setpoint(1., pregrasp)
    add_setpoint(1., grasp)
    add_setpoint(0.5, closed)
    add_setpoint(1., pregrasp_closed)
    add_setpoint(1., pre_throw_stance)

    add_setpoint(2., pre_throw_windup)
    add_setpoint(0.5, mid_throw)
    add_setpoint(0.1, post_throw_extended)
    add_setpoint(3., pre_throw_stance)

    ts = np.array(ts)
    qs = np.stack(qs)
    print(ts, qs)
    ser.write(pack_trajectory_to_buf(ts, qs))

    t_start = time.time()
    while (time.time() - t_start < ts[-1] + 2.):
        if (ser.inWaiting() > 0):
            # read the bytes and convert from binary array to ASCII
            data_str = ser.read(ser.inWaiting()).decode('ascii') 
            # print the incoming string without putting a new-line
            # ('\n') automatically after every print()
            print(data_str, end='') 
        time.sleep(0.1)

