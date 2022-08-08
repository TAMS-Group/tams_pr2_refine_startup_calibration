#!/bin/bash

# top-level file: run in order to refine all right arm joints

PKG=tams_pr2_refine_startup_calibration

cat <<EOF
Going to stop all running controllers on the PR2.
The arms/head will drop towards gravity, so make sure
they will not hit anything and the head will fall backwards.

Press Enter to proceed
EOF

read
sleep 2.0
rosrun $PKG stop_controllers.sh

refine_joint() {
  J=$1
  cat <<EOF
Going to run recalibration (in endless loop)
for joint $J .
Please make sure the robot is in the expected configuration
for this calibration step.
EOF
  ROS_NAMESPACE /calibration_slow_controllers/cal_$J \
    rosrun $PKG monitor_zero_offset \
      set_zero_offset_from_mean:=/monitor_zero_offset/set_zero_offset_from_mean
  MONITOR_OFFSET_PID=$!

  rosrun $PKG rerun_calibration_slow.sh $J >/dev/null 2>&1 &
  CAL_PID=$!

  cat <<EOF
Recalibration is running.
Press Enter to abort the loop and write the mean of all trials as offset.
EOF
  read

  echo "Killing recalibration routine"
  kill -INT $CAL_PID
  sleep 1.0

  rosservice call /monitor_zero_offset/set_zero_offset_from_mean
  kill $MONITOR_OFFSET_PID
}

#rosrun $PKG online_histogram.py

refine_joint r_elbow_flex
cat <<EOF
Moving r_elbow_flex to hold position. This directly starts a PID position controller,
so the motion will be fast unless the joint is already close to the target.
EOF
rosrun $PKG hold_position.sh r_elbow_flex
