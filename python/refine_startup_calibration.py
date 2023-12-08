#!/usr/bin/env python

from typing import Any, List

import rosparam
import rospkg
import rospy
import sys

from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController, SwitchControllerRequest, ListControllers
from std_msgs.msg import String
from tams_pr2_refine_startup_calibration.srv import SetZeroOffset

class Say:
    def __init__(self):
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)

    def __call__(self, string) -> Any:
        self.say_pub.publish(String(data=string))
say = Say()

class RefineStartupCalibration:
    R = {
        'left_arm': {
            'trajectory': 'l_arm_controller',
            'joints': [
                'l_shoulder_pan',
                'l_shoulder_lift',
                'l_upper_arm_roll',
                'l_elbow_flex',
                'l_forearm_roll',
            ],
            # refinement is not supported, but they need position controllers and targets
            'extra_joints': [
                'l_wrist_flex',
                'l_wrist_roll',
            ],
        },
        'right_arm': {
            'trajectory': 'r_arm_controller',
            'supported_joints': [
                'r_shoulder_pan',
                'r_shoulder_lift',
                'r_upper_arm_roll',
                'r_elbow_flex',
                'r_forearm_roll',
            ],
            'extra_joints': []
        },
        'head': {
            'trajectory': 'head_traj_controller',
            'joints': [
                'head_pan',
                'head_tilt',
            ],
            'extra_joints': []
        },
    }

    def __init__(self):
        self.load_controller = rospy.ServiceProxy('/pr2_controller_manager/load_controller', LoadController)
        self.unload_controller = rospy.ServiceProxy('/pr2_controller_manager/unload_controller', UnloadController)
        self.switch_controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        self.list_controllers = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)

    def ensure_controllers(self, controllers, *, start) -> List[str]:
        '''
        Make sure that the given controllers are loaded
        '''
        sleep = False
        for c in controllers:
            if c not in state.controllers:
                if not self.load_controller(name= c).ok:
                    rospy.logfatal(f"Failed to load controller {c}")
                    sys.exit(1)
                sleep= True
        if sleep:
            rospy.sleep(rospy.Duration(0.5))

        state = self.list_controllers()
        for c in controllers:
            if c not in state.controllers:
                rospy.logfatal(f"controller {c} not loaded after loading it")
                sys.exit(1)

        if start:
            req = SwitchControllerRequest(
                    start_controllers= [c for c in controllers if state.state[state.controllers.index(c)] != 'running'],
                    stop_controllers= [],
                    strictness= SwitchControllerRequest.BEST_EFFORT)
            if not self.switch_controller(req).ok:
                rospy.logfatal(f"Failed to start controllers {req.start_controllers}")
                sys.exit(1)

    def unload_controllers(self, controllers):
        '''
        Make sure that the given controllers are unloaded
        '''
        state = self.list_controllers()

        controllers = [c for c in controllers if c in state.controllers]

        to_stop = [c for c in controllers if state.state[state.controllers.index(c)] == 'running']
        if to_stop:
            req = SwitchControllerRequest(
                    start_controllers= [],
                    stop_controllers= to_stop,
                    strictness= SwitchControllerRequest.BEST_EFFORT)
            if not self.switch_controller(req).ok:
                rospy.logfatal(f"Failed to stop controllers {req.stop_controllers}")
                sys.exit(1)
        for c in controllers:
            if c in state.controllers:
                if not self.unload_controller(name= c).ok:
                    rospy.logfatal(f"Failed to unload controller {c}")
                    sys.exit(1)

    @staticmethod
    def joints_from_goal(goal):
        '''
        figure out which joints to refine from string goal (e.g. 'all', 'left_arm', 'r_shoulder_pan_joint')
        '''
        all_joints = [j for group in RefineStartupCalibration.R for j in RefineStartupCalibration.R[group]['joints']]

        joints = []
        for g in goal:
            if g == 'all':
                joints.extend(all_joints)
            elif g in RefineStartupCalibration.R:
                joints.extend(RefineStartupCalibration.R[goal]['joints'])
            elif goal in all_joints:
                joints.append(goal)
            else:
                rospy.logfatal(f"Unknown goal to refine: {goal}")
                sys.exit(1)

        return joints

    def switch_to_position_controllers(self, group):
        if not self.switch_controller(
            start_controllers= [f"position_controllers/{j}_position_controller" for j in self.R[group]['joints']],
            stop_controllers= [self.R[group]['trajectory']],
            strictness= SwitchControllerRequest.STRICT
        ).ok:
            rospy.logfatal(f"Failed to switch {group} to position controllers")
            sys.exit(1)

    def switch_to_trajectory_controller(self, group):
        state = self.list_controllers()
        position_controllers = [f"position_controllers/{j}_position_controller" for j in self.R[group]['joints']]
        to_stop = [c for c in position_controllers if c in state.controllers and state.state[state.controllers.index(c)] == 'running']

        if not self.switch_controller(
            start_controllers= [self.R[group]['trajectory']],
            stop_controllers= to_stop,
            strictness= SwitchControllerRequest.STRICT
        ).ok:
            rospy.logfatal(f"Failed to switch {group} to trajectory controller")
            sys.exit(1)

    def refine_group_joints(self, group, joints):
        '''
        refine the given joints of the given group
        '''

        self.switch_to_position_controllers(group)

        for j in joints:
            say(f"Refine joint {joints}")

            # TODO: go on here

            # lookup joint positions of group for refinement of j

            # set position controllers to these positions

            # activate calibration controller repeatedly until user commits

            self.set_zero_offset = rospy.ServiceProxy(f'/set_zero_offset/{j}/set_zero_offset', SetZeroOffset)
            mean = 0.0
            self.set_zero_offset(mean)

        self.switch_to_trajectory_controller(group)

    def do(self, goal):

        joints = self.joints_from_goal(goal)

        rpack = rospkg.RosPack()

        config_path = rpack.get_path('tams_pr2_refine_startup_calibration')


        for params, ns in rosparam.load_file(f"{config_path}/config/pr2_set_zero_offset_controllers.yaml", namespace='set_zero_offset'):
            rosparam.upload_params(ns,params)
        set_zero_offset_controllers = [f"set_zero_offset/{j}" for j in joints]

        for params, ns in rosparam.load_file(f"{config_path}/config/pr2_calibration_controllers_for_refine.yaml", namespace='calibration_controllers_for_refine'):
            rosparam.upload_params(ns,params)
        calibration_controllers_for_refine = [f"calibration_controllers_for_refine/{j}" for j in joints]

        for params, ns in rosparam.load_file(rpack.get_path('tams_pr2_controller_configuration') + '/config/pr2_joint_position_controllers.yaml', namespace='/position_controllers')
            rosparam.upload_params(ns,params)
        position_controllers = [f"position_controllers/{j}_position_controller" for j in joints]

        self.ensure_controllers(
            calibration_controllers_for_refine +
            position_controllers +
            [traj_controller for group in self.CONTROLLERS for traj_controller in self.CONTROLLERS[group]['trajectory']],
            start= False
        )

        self.ensure_controllers(set_zero_offset_controllers, start= True)

        for group in self.R:
            group_joints = [j for j in self.R[group]['joints'] if j in joints]
            self.refine_group_joints(group, group_joints)

        self.unload_controllers(calibration_controllers_for_refine + position_controllers + set_zero_offset_controllers)

if __name__ == "__main__":
    rospy.init_node('refine_startup_calibration')
    sys.argv = rospy.myargv(argv=sys.argv)

    if len(sys.argv) < 2:
        rospy.logerr("Usage: refine_startup_calibration.py <all/left_arm/right_arm/head/joint_name>")
        sys.exit(1)

    RefineStartupCalibration().do(sys.argv[1:])