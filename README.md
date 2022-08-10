# Description

This package supports re-initializing the robot joints to reduce jitter in the `zero_offset` detection on startup.
The values calibrated through this package are stored on the Motor Control Boards (MCBs) of each motor and vary each time the robot is powered up.
It does **NOT** change the static calibration of the robot itself which persists across startups and is defined in the URDF.

# Howto

Start `refine_startup_calibration.sh` to invoke the full recalibration procedure.
The script is rather verbose about each step, so please read the terminal output
and only press enter when ready for the next step.

For the r_elbow_flex and r_forearm_roll make sure you move the unactuated joints of the arm to the (rough) specified positions (see doc/ folder for images).

As each joint calibrates, add a bit of noise by repeatedly changing the configuration of other joints.
For r_elbow_flex: move r_forearm_roll around to multiple positions and slightly vary r_upper_arm_roll too.
Do **not** make the joint go through the zero crossing with a different velocity as this introduces strong noise on the estimation.
