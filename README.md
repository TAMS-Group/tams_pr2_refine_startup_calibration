# Description

This package supports re-initializing the robot joints to reduce jitter in the `zero_offset` detection on startup.
The values calibrated through this package are stored on the Motor Control Boards (MCBs) of each motor and vary each time the robot is powered up.
It does **NOT** change the static calibration of the robot itself which persists across startups and is defined in the URDF.

# Howto

Start `refine_startup_calibration.sh` to invoke the full recalibration procedure.
The script is rather verbose about each step, so please read the terminal output
and only press enter when ready for the next step.
