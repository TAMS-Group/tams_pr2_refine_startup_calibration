#!/bin/bash

JOINTS="
r_shoulder_pan
r_shoulder_lift
r_upper_arm_roll
r_elbow_flex
r_forearm_roll
"

rosparam load `rospack find $PKG`/config/pr2_set_zero_offset_controllers.yaml

for j in $JOINTS; do
  rosservice call /pr2_controller_manager/load_controller "name: 'zero_offset/$j'"
done

query="start_controllers:\n"
for j in $JOINTS; do
  query="${query}\n- 'zero_offset/$j'"
done
query="$query\nstrictness: 2"

rosservice call /pr2_controller_manager/switch_controller "$(printf "$query")"
