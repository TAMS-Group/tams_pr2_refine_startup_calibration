#!/bin/bash

J="${1:-r_elbow_flex}"

declare -A hold_positions
hold_positions[r_elbow_flex]=-1.57
hold_positions[r_forearm_roll]=0.0
hold_positions[r_shoulder_lift]=0.0
hold_positions[r_upper_arm_roll]=0.0
hold_positions[r_shoulder_pan]=0.0

if [[ -z "${hold_positions[$J]}" ]]; then
  echo "error: no hold position defined for joint $J" >&2
  exit 1
fi

C="${J}_position_controller"
NS=position_controllers 

rosparam delete $NS
rosparam load $(rospack find tams_pr2_controller_configuration)/pr2_joint_position_controllers.yaml /$NS

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- '$NS/$C'
strictness: 2"

rosservice call /pr2_controller_manager/unload_controller "name: '$NS/$C'"
# give time to update parameters
sleep 1.0
rosservice call /pr2_controller_manager/load_controller "name: '$NS/$C'"

rosservice call /pr2_controller_manager/switch_controller "start_controllers:
- '$NS/$C'
strictness: 2"

rostopic pub -1 /position_controllers/${J}_position_controller/command std_msgs/Float64 "data: ${hold_positions[$J]}"
