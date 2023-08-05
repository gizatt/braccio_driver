import numpy as np
import serial
import time
import argparse
import math
import sys

from util import pack_trajectory_to_buf


def two_link_inverse_kinematics(target, a1, a2):
    """
        Target: 2D point to place EE.
        a1, a2: link lengths a1, a2.

        Returns:
        two sets of angles [q1, q2], 0 with arm extended in +x direction
    """
    cos_q2 = (target[0]**2 + target[1]**2 - a1**2 - a2**2) / (2. * a1 * a2)
    cos_q2_clamped = np.clip(cos_q2, -0.9999, 0.9999)
    if not np.isclose(cos_q2, cos_q2_clamped):
        print("WARN: IK out of range, giving approximate answer.")
    q2 = np.arccos(cos_q2_clamped)
    q1_pos = np.arctan2(target[1], target[0]) - np.arctan2(a2 * np.sin(q2), a1 + a2 * np.cos(q2))
    q1_neg = np.arctan2(target[1], target[0]) - np.arctan2(a2 * np.sin(-q2), a1 + a2 * np.cos(-q2))
    return [q1_pos, q2], [q1_neg, -q2]

def wrap_angles(q):
    q = q.copy()
    for i in range(len(q)):
        while q[i] > np.pi:
            q[i] -= np.pi
        while q[i] < -np.pi:
            q[i] += np.pi
    return q

def get_joint_angles(throw_yaw: float, throw_elevation: float, pin_to_hand_distance: float, theta_release: float):
    '''
        Given throw yaw and pitch (e.w.r.t. robot base), a distance between the pin and the hand,
        and a desired "release angle" from the pin, compute 6DOF joint angles for the robot.

        throw_elevation is 0 when flat and 90 when straight up.
    '''
    q = np.zeros((6,))
    # Always keep the hand fully open as the "Y" of our slingshot.
    q[4] = np.deg2rad(90.)
    q[5] = np.deg2rad(40.)
    # Pass yaw through
    q[0] = throw_yaw
    # Adjust for extra theta_release in the desired pitch, remembering that +90 is straight up, and +180 faces
    # "forward".
    q[1] = np.pi - throw_elevation + theta_release

    # Calculate elbow and wrist using 2-link IK.
    # [x, y], x along elbow dir at 0 degrees, y perpendicular in plane of elbow/wrist joints.
    PIN_EWRT_ELBOW = np.array([0.02, 0.0])
    target_ewrt_elbow = PIN_EWRT_ELBOW + pin_to_hand_distance * np.array([
        np.cos(theta_release), -np.sin(theta_release)
    ])
    ELBOW_TO_WRIST_LEN = 0.123
    WRIST_TO_GRIPPER_CENTER_LEN = 0.171
    sol1, sol2 = two_link_inverse_kinematics(
        target_ewrt_elbow, a1=ELBOW_TO_WRIST_LEN, a2=WRIST_TO_GRIPPER_CENTER_LEN
    )
    print(target_ewrt_elbow, sol1, sol2)
    q[2:4] = wrap_angles(sol2[:])
    q[2] += np.pi/2.
    q[3] += np.pi/2.
    return q


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", default="COM4")

    parser.add_argument("--throw_yaw", default=90., type=float)
    parser.add_argument("--throw_elevation", default=60, type=float)
    parser.add_argument("--pin_to_hand_distance", default=0.22, type=float)

    args = parser.parse_args()

    try:
        ser = serial.Serial(port=args.port, baudrate=9600, timeout=.1)
    except serial.SerialException as e:
        print(e)
        print("Available ports:")
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports(include_links=False):
            print(port)
        sys.exit(1)

    ts = []
    qs = []

    ts = np.linspace(1, 3., 10)
    theta_release_center = -np.pi/8
    theta_releases = theta_release_center + np.linspace(-np.pi/4., np.pi/8, len(ts))
    for theta_release in theta_releases:
        qs.append(
            get_joint_angles(np.deg2rad(180.) - np.deg2rad(args.throw_yaw), np.deg2rad(args.throw_elevation),
                             args.pin_to_hand_distance, theta_release)
        )

    ts = np.r_[ts, ts[-1] + 1.]
    qs.append(qs[0])

    qs = np.stack(qs)
    print(ts, np.rad2deg(qs))
    ser.write(pack_trajectory_to_buf(ts, np.rad2deg(qs)))
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