# Arduino Braccio Piecewise Trajectory Driver

Accepts trajectory commands over Serial and spools them out to the robot, hopefully allowing for accurate playback of rapid, coordinated trajectories.

A trajectory is specified as a list of *knot points* or *knots*, each consisting of a pair of a time (offset from the time that the message is received by the Arduino) and a list of joint commands (which, for the Braccio, is [base, shoulder, elbow, wrist 1, wrist 2, gripper]) in degrees. At each tick, the Arduino commands the Braccio to move to a linear interpolation of the most recent and next position commands. If a trajectory is supplied whose first knot point is specified at t > 0, the driver will interpolate from the robot's commanded position at the time the command was received. At startup, the driver defaults to a single-knot trajectory that holds the robot at its totally-upright posture (all angles = 0).

## Environment / Setup

The driver code is intended for an Arduino Due; with minor mods it short work on basically any Arduino. (You may need to change "SerialUSB" to "Serial" depending on the Arduino and connection you're using.) It's structured as a [PlatformIO](https://platformio.org/) project.

## Serial command spec

A trajectory is specified by a serialized buffer of floating point numbers serialized like:
```
<0xFFFFFFFF> <N><binary blob><checksum>
```
After a start of 4 all-one bytes, the first data byte is a uint8 N indicating the number of knots. The next `N*7*4` bytes are a packed array of 32-bit floats, where the first 7 entries are the first knot ([t, m1, m2, ..., m6]), the second 7 are the second knot, etc. Then, a 32-bit checksum (bitwise sum of all bytes in the binary blob).

## Proof of life

The code, as-is, sends ASCII heartbeat strings over serial every second; if you see those, it's working right. When you send a command buffer, it should return an ASCII string confirming its reception, or complaining about one of a large number of possible formatting or encoding problems.

## Python stuff

A couple of example Python scripts that utilize this driver to make the arm do interesting things are included under the `python` folder. `util.py:pack_trajectory_to_buf` is the meat of the message packing; supply it with a vector of N knot times `ts` and an Nx6 array of joint configurations `qs`, and shove the buffer you get into the serial port, and the robot should do what you want.

- `teleop.py` runs a Matplotlib widget allowing direct teleop of the robot with some sliders.
- `throw_manual.py` drives the robot through a scripted sequence that picks up an object from in front of it, and then throws it. It generates a throwing trajectoy manually by creating joint angle setpoints around a fully-extended release configuration with the right relative motion to achieve a pretty-close-to-the-practical-joint-velocity-limits throw. I've tuned this to throw a paper airplane pretty well.
- `throw_trajopt.py` and `throw_plane_trajopt.py` play back trajectories from trajectory optimization to perform different throws. `throw_trajopt.py` picks up objects from three different locations and throws them each in sequence.
- `test_send_trajectory.py` is some old testing cruft.

### Deps

`pyserial`, `numpy`, `matplotlib`

### Trajectory optimization

The files under `python/trajectories` were generated via a trajectory optimization using Drake, implemented in [this deepnote](https://deepnote.com/workspace/greg-izatt-73314da8-6e78-4e92-a3d0-50c60a42ac40/project/braccioarmtrajopt-7ed10f47-1848-4d65-81c9-53a81d301e31/%2Fcore.629).

## Other notes

The Braccio URDF included here is modified from the one helpfully provided at https://github.com/jonabalzer/braccio_description. It's the URDF used for trajectory optimization, and seems to be a good match for the real robot.