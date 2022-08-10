#!/bin/bash

# stop all controllers that might get into the way of calibration

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- 'r_arm_controller'
- 'l_arm_controller'
- 'head_traj_controller'
strictness: 1"

sleep 0.5

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- 'r_arm_controller_loose'
- 'l_arm_controller_loose'
strictness: 1"

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- 'position_controllers/r_elbow_flex_position_controller'
- 'position_controllers/r_forearm_roll_position_controller'
- 'position_controllers/r_shoulder_lift_position_controller'
- 'position_controllers/r_shoulder_pan_position_controller'
- 'position_controllers/r_upper_arm_roll_position_controller'
strictness: 1"

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- 'zero_effort/r_elbow_flex'
- 'zero_effort/r_forearm_roll'
- 'zero_effort/r_shoulder_lift'
- 'zero_effort/r_shoulder_pan'
- 'zero_effort/r_upper_arm_roll'
strictness: 1"
