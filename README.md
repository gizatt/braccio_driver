# Arduino Braccio Piecewise Trajectory Driver

Accepts trajectory commands over Serial and spools them out to the robot, hopefully allowing for accurate playback of rapid, coordinated trajectories.

A trajectory is specified as a list of *knot points* or *knots*, each consisting of a pair of a time (offset from the time that the message is received by the Arduino) and a list of joint commands (which, for the Braccio, is [base, shoulder, elbow, wrist 1, wrist 2, gripper]) in degrees. At each tick, the Arduino commands the Braccio to move to a linear interpolation of the most recent and next position commands. If a trajectory is supplied whose first knot point is specified at t > 0, the driver will interpolate from the robot's commanded position at the time the command was received. At startup, the driver defaults to a single-knot trajectory that holds the robot at its totally-upright posture (all angles = 0).

## Serial command spec

A trajectory is specified by a serialized buffer of floating point numbers serialized like:
```
<0xFFFFFFFF> <N><binary blob><checksum>
```
After a start of 4 all-one bytes, the first data byte is a uint8 N indicating the number of knots. The next `N*7*4` bytes are a packed array of 32-bit floats, where the first 7 entries are the first knot ([t, m1, m2, ..., m6]), the second 7 are the second knot, etc. Then, a 32-bit checksum (bitwise sum of all bytes in the binary blob).

## Useful robot configurations

### Reaching for a thing
Pregrasp: [147, 80, 45, 0., 0., 66]
Grasp: [147, 62, 35, 0., 0., 50]
Closed: [147, 62, 35, 0., 0., 120]

### Throwing
Pre-throw neutral: [55, 145, 45, 0, 90, 120]
Pre-throw extended: [55, 170, 90, 90, 90, 120]
Post-throw extended: [55, 90, 90, 90, 90, 120]
(That second joint is the base pitch joint. At 
135 it's at 45 degrees going up; at 90 it's straight up.)