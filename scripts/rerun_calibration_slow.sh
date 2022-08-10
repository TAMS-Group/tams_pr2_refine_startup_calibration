#!/bin/bash

# run calibration controller in endless loop to repeatedly detect zero offset (published as topic)
# reload configuration file in case someone changed parameters

J="${1:-r_elbow_flex}"

C="cal_$J"
S=4
NS=calibration_slow_controllers
 
rosparam delete /$NS
rosparam load $(rospack find tams_pr2_refine_startup_calibration)/config/pr2_calibration_controllers_slow.yaml /$NS

rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- '$NS/$C'
strictness: 2"
sleep 1.0
rosservice call /pr2_controller_manager/unload_controller "name: '$NS/$C'"
# give time to update parameters
sleep 0.5
rosservice call /pr2_controller_manager/load_controller "name: '$NS/$C'"
sleep 0.5

# stop calibration controller on shutdown
trap ctrl_c INT SIGTERM
ctrl_c() {
  rosservice call /pr2_controller_manager/switch_controller "stop_controllers:
- '$NS/$C'
strictness: 2"
  exit 0
}

echo "starting calibration controller for $J every $S seconds"
while true
do
  rosservice call /pr2_controller_manager/switch_controller "start_controllers:
- '$NS/$C'
strictness: 2"
  sleep $S
done
