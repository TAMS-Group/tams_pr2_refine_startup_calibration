#!/bin/bash

# top-level file: run in order to refine all right arm joints

PKG=tams_pr2_refine_startup_calibration

# load controllers to set zero offsets
rosrun $PKG start_set_zero_offset_controllers.sh >/dev/null 2>&1

# STOP all Trajectory/position controllers
##########################################

cat <<EOF
========
Going to stop all running controllers on the PR2.
The arms/head will drop towards gravity, so make sure
they will not hit anything and the head will fall backwards.

Press Enter to proceed
EOF

read
sleep 2.0
rosrun $PKG stop_controllers.sh >/dev/null 2>&1
cat <<EOF
Done

EOF

refine_joint() {
  J=$1
  cat <<EOF
========
Going to run recalibration (in endless loop)
for joint ${J}.
Please make sure the robot is in the expected configuration
for this calibration step.
Press Enter to continue
EOF
  read

  ROS_NAMESPACE=/calibration_slow_controllers/cal_$J \
    rosrun $PKG monitor_zero_offset.py \
      set_zero_offset_from_mean:=/monitor_zero_offset/set_zero_offset_from_mean &
  MONITOR_OFFSET_PID=$!

  rosrun topic_tools relay \
    /calibration_slow_controllers/cal_$J/zero_offset \
    /zero_offset_histogram/zero_offset >/dev/null 2>&1 &
  OFFSET_RELAY_PID=$!

  rosrun $PKG rerun_calibration_slow.sh $J >/dev/null 2>&1 &
  CAL_PID=$!

  cat <<EOF
========
Recalibration for ${J} is running.
Press Enter to abort the loop and write the mean of all trials as offset.
EOF
  read

  echo "Killing recalibration routine"
  kill -TERM $CAL_PID
  sleep 1.0

  kill -TERM $OFFSET_RELAY_PID

  echo -n "Writing result to MCB... "
  rosservice call /monitor_zero_offset/set_zero_offset_from_mean && echo "success" || echo "failed"
  kill $MONITOR_OFFSET_PID

  rosservice call /zero_offset_histogram/reset
}

# Start GUI to show histogram of offset detections
##################################################

rosrun $PKG zero_offset_histogram.py >/dev/null 2>&1 &
HISTOGRAM_PLOT_PID=$!

trap shutdown_histogram INT TERM EXIT
shutdown_histogram() {
  # if only INT/TERM would work for python ROS nodes...
  kill -KILL $HISTOGRAM_PLOT_PID
  exit 0
}

# Start of calibration routine
################################

refine_and_hold() {
  refine_joint $1

  if [[ -n "$refine_first_joint" ]]; then
    cat <<EOF
========
Moving r_elbow_flex to hold position. This directly starts a PID position controller,
so the motion will be fast unless the joint is already close to the target.
EOF
  else
    cat <<EOF
========
Moving $1 to hold position. Proceed with caution.
Press Enter to continue
EOF
  fi

  read

  rosrun $PKG hold_position.sh $1 >/dev/null 2>&1
} 

refine_and_hold r_elbow_flex

refine_and_hold r_forearm_roll

refine_and_hold r_shoulder_lift

refine_and_hold r_upper_arm_roll

refine_and_hold r_shoulder_pan

cat <<EOF
========
Recalibration complete. Starting trajectory controllers back up.
Switching off position controllers to do that, so the arms will drop in between.
Press Enter to continue.
EOF

rosrun $PKG stop_controllers.sh >/dev/null 2>&1
rosrun $PKG start_trajectory_controllers.sh >/dev/null 2>&1

cat <<EOF
========
Done! You are up and running. Good luck with your experiments.
========
EOF

