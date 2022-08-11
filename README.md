# Description

This package supports re-initializing the robot joints to reduce jitter in the `zero_offset` detection on startup.
The values calibrated through this package are stored on the Motor Control Boards (MCBs) of each motor and vary each time the robot is powered up.
It does **NOT** change the static calibration of the robot itself which persists across startups and is defined in the URDF.

# Howto

Start `refine_startup_calibration.sh` to invoke the full recalibration procedure.
The script is rather verbose about each step, so please read the terminal output and only press enter when ready for the next step.

For each target joint the procedure will load a slow velocity controller one at a time and repeatedly move the joint through its zero crossing.
Typically you should record at least 20 samples (I sometimes collected 200) to get a good estimate for the detected mean.
The sampled distributions can change after slight external pertubations or changing the joint states of otherwise unrelated joints.
If you notice such a case, try to record roughly the same amount of samples for each such peak.

After you exit the data collection step the estimated mean will be written to the controller and the joint is done calibrating.
To make the calibration of the next joints more stable the procedure will load a position controller for the joint next and move it to a "hold" position that will not change for the rest of the script.

For `r_elbow_flex`, `r_forearm_roll` and `r_shoulder_lift` make sure you move the unactuated joints of the arm to the (rough) specified positions (see doc/ folder for images). You need to hold these positions for a while, so better use some furniture as support for the arm where required.

As each joint calibrates, add a bit of noise by repeatedly changing the configuration of other joints between trials.
E.g., when calibrating r_elbow_flex, move r_forearm_roll around to multiple positions and slightly vary r_upper_arm_roll too.

Do **not** make the joint go through the zero crossing with a different velocity as this introduces strong noise on the estimation.
