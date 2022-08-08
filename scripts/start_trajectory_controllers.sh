#!/bin/bash

# start expected controllers after successful recalibration

rosservice call /pr2_controller_manager/switch_controller "start_controllers:
- 'r_arm_controller'
- 'l_arm_controller'
- 'head_traj_controller'
strictness: 2"
