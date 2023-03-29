#!/bin/bash

JOINTS="
r_shoulder_pan
r_shoulder_lift
r_upper_arm_roll
r_elbow_flex
r_forearm_roll
"


query="stop_controllers:\n"
for j in $JOINTS; do
  query="${query}\n- 'zero_offset/$j'"
  query="${query}\n- 'calibration_slow_controllers/cal_$j'"
done
query="$query\nstrictness: 1"

rosservice call /pr2_controller_manager/switch_controller "$(printf "$query")"

for j in $JOINTS; do
  rosservice call /pr2_controller_manager/unload_controller "name: 'zero_offset/$j'"
  rosservice call /pr2_controller_manager/unload_controller "name: 'calibration_slow_controllers/cal_$j'"
done
