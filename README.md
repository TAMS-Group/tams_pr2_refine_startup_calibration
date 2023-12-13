# Description

This package supports repeated initializing of the robot's joints to reduce jitter in the `zero_offset`/edge trigger detection on startup.
The values calibrated through this procedure are stored on the Motor Control Boards (MCBs) of each motor and vary each time the robot is powered up.
It does **NOT** change the static calibration of the robot itself which persists across startups and is defined in the URDF.

# Howto

Run `refine_startup_calibration <left_arm/right_arm/head/all/any_joint_name>` to invoke the recalibration procedure on the robot.
To visualize the data acquisition process and decide when each joint is well-estimated, you should additionally run
`plot_zero_offsets` which will show you all currently perceived zero offsets and their mean (+ outlier-resistant mean).

The refinement script is verbose about each step (with /say output if Text-to-Speech is running). Please read the terminal output
and only press enter when ready to move the joint group to the new position or finish the calibration loop for each joint.

The procedure will move the head/left\_arm/right\_arm joint group into dedicated "calibration positions" and afterwards load
a velocity based pid controller to move each target joint through its joint edge trigger repeatedly.

Once you expect the outlier-resistant mean represents your expected zero offset, you can move on to the next joint.
