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

# TODO: this does not stop the left arm controllers and we really don't want that either. Redesign process please

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- 'position_controllers/r_elbow_flex_position_controller'
- 'position_controllers/r_forearm_roll_position_controller'
- 'position_controllers/r_shoulder_lift_position_controller'
- 'position_controllers/r_shoulder_pan_position_controller'
- 'position_controllers/r_upper_arm_roll_position_controller'
strictness: 1"

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- 'zero_offset/r_elbow_flex'
- 'zero_offset/r_forearm_roll'
- 'zero_offset/r_shoulder_lift'
- 'zero_offset/r_shoulder_pan'
- 'zero_offset/r_upper_arm_roll'
strictness: 1"
