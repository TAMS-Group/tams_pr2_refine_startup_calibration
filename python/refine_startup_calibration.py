#!/usr/bin/env python

from typing import Any, List, NamedTuple

import numpy as np
import rosparam
import rospkg
import rospy
import sys
import threading

from actionlib import SimpleActionClient
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryResult
from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController, SwitchControllerRequest, ListControllers
from std_msgs.msg import String as StringMsg, Float32 as Float32Msg
from tams_pr2_refine_startup_calibration.srv import SetZeroOffset, SetZeroOffsetRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Say:
    def __init__(self):
        self.say_pub = rospy.Publisher('/say', StringMsg, queue_size=10)

    def __call__(self, string) -> Any:
        self.say_pub.publish(StringMsg(data=string))
        rospy.sleep(rospy.Duration(1.0))

say = Say()

class GroupWithJoints(NamedTuple):
    group: str
    joints: List[str]

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

    CALIBRATION_GROUP_POSITIONS = {
        'l_shoulder_pan': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(self):
        self.load_controller = rospy.ServiceProxy('/pr2_controller_manager/load_controller', LoadController)
        self.unload_controller = rospy.ServiceProxy('/pr2_controller_manager/unload_controller', UnloadController)
        self.switch_controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        self._list_controllers = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)

        self.trajectory_clients = {}
        for group in self.R:
            self.trajectory_clients[group] = SimpleActionClient(f"/{self.R[group]['trajectory']}/joint_trajectory_action", JointTrajectoryAction)

    def list_controllers(self):
        state = self._list_controllers()
        if not state.ok:
            rospy.logfatal("Failed to list controllers")
            sys.exit(1)
        return dict(zip(state.controllers, state.state))

    def ensure_controllers(self, controllers, *, start : bool) -> List[str]:
        '''
        Make sure that the given controllers are loaded
        '''
        controller_state = self.list_controllers()
        sleep = False
        for c in controllers:
            if c not in controller_state:
                if not self.load_controller(name= c).ok:
                    rospy.logfatal(f"Failed to load controller {c}")
                    sys.exit(1)
                sleep= True
        if sleep:
            # give pr2_controller_manager some time to load the controllers / that should not be necessary...
            rospy.sleep(rospy.Duration(0.5))

        controller_state = self.list_controllers()
        for c in controllers:
            if c not in controller_state:
                rospy.logfatal(f"controller {c} not loaded after loading it")
                sys.exit(1)

        if start:
            req = SwitchControllerRequest(
                    start_controllers= [c for c in controllers if controller_state[c] != 'running'],
                    stop_controllers= [],
                    strictness= SwitchControllerRequest.BEST_EFFORT)
            if not self.switch_controller(req).ok:
                rospy.logfatal(f"Failed to start controllers {req.start_controllers}")
                sys.exit(1)

    def unload_controllers(self, controllers):
        '''
        Make sure that the given controllers are unloaded
        '''
        controller_state = self.list_controllers()

        controllers = [c for c in controllers if c in controller_state]

        to_stop = [c for c in controllers if controller_state[c] == 'running']
        if to_stop:
            req = SwitchControllerRequest(
                    start_controllers= [],
                    stop_controllers= to_stop,
                    strictness= SwitchControllerRequest.BEST_EFFORT)
            if not self.switch_controller(req).ok:
                rospy.logfatal(f"Failed to stop controllers {req.stop_controllers}")
                sys.exit(1)
        for c in controllers:
            if c in controller_state:
                if not self.unload_controller(name= c).ok:
                    rospy.logfatal(f"Failed to unload controller {c}")
                    sys.exit(1)

    def joints_from_goal(self, goal) -> List[GroupWithJoints]:
        '''
        figure out which joints to refine from string goal (e.g. 'all', 'left_arm', 'r_shoulder_pan_joint')
        '''
        all_joints = [j for group in RefineStartupCalibration.R for j in RefineStartupCalibration.R[group]['joints']]

        joints_by_group : GroupWithJoints = []
        for g in goal:
            if g == 'all':
                joints_by_group.extend([
                    GroupWithJoints('left_arm', self.R['left_arm']['joints'][:]),
                    GroupWithJoints('right_arm', self.R['right_arm']['joints'][:]),
                    GroupWithJoints('head', self.R['head']['joints'][:]),
                    ])
            elif g in RefineStartupCalibration.R:
                joints_by_group.append(GroupWithJoints(g, RefineStartupCalibration.R[g]['joints'][:]))
            elif goal in all_joints:
                if joints_by_group and goal in self.R[joints_by_group[-1].group]['joints']:
                    joints_by_group[-1].joints.append(goal)
                else:
                    g = next(g for g in RefineStartupCalibration.R if goal in RefineStartupCalibration.R[g]['joints'])
                    joints_by_group.append(GroupWithJoints(g, [goal]))
            else:
                rospy.logfatal(f"Unknown goal to refine: {goal}")
                sys.exit(1)

        return joints_by_group

    def switch_to_position_controllers(self, group):
        controller_state = self.list_controllers()

        position_controllers = [f"position_controllers/{j}_position_controller" for j in self.R[group]['joints']]
        to_start = [c for c in position_controllers if controller_state[c] != 'running']
        to_stop = [self.R[group]['trajectory']] if controller_state[self.R[group]['trajectory']] == 'running' else []

        if not self.switch_controller(
            start_controllers= to_start,
            stop_controllers= to_stop,
            strictness= SwitchControllerRequest.STRICT
        ).ok:
            rospy.logfatal(f"Failed to switch {group} to position controllers")
            sys.exit(1)

    def switch_to_trajectory_controller(self, group):
        controller_state = self.list_controllers()
        position_controllers = [f"position_controllers/{j}_position_controller" for j in self.R[group]['joints']]

        to_stop = [c for c in position_controllers if controller_state[c] == 'running']
        to_start = [self.R[group]['trajectory']] if controller_state[self.R[group]['trajectory']] != 'running' else []

        if not self.switch_controller(
            start_controllers= to_start,
            stop_controllers= to_stop,
            strictness= SwitchControllerRequest.STRICT
        ).ok:
            rospy.logfatal(f"Failed to switch {group} to trajectory controller")
            sys.exit(1)

    def move_to_group_positions(self, group, positions):
        joints = self.R[group]['joints'] + self.R[group]['extra_joints']
        if len(positions) != len(joints):
            rospy.logfatal(f"Number of positions ({len(joints)}) does not match number of joints for {group}: {joints}")
            sys.exit(1)

        self.switch_to_trajectory_controller(group)
        if not self.trajectory_clients[group].wait_for_server(timeout= rospy.Duration(2.0)):
            rospy.logfatal(f"Failed to connect to trajectory action server for {group}")
            sys.exit(1)

        jt = JointTrajectory()
        jt.joint_names = joints
        jt.points.append(JointTrajectoryPoint(positions= positions, time_from_start= rospy.Duration(2.0)))
        self.trajectory_clients[group].send_goal(JointTrajectoryGoal(trajectory= jt))
        result = self.trajectory_clients[group].wait_for_result().result
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logerr(f"Failed to move {group} to positions {positions}")
            say(f"Ooops, group {group} could not move. Please press enter to continue nevertheless.")
            input("Press enter to continue...")

    def trigger_calibration_controller_during_refine(self, joint, *, start : bool):
        to_start = [f"calibration_controllers_for_refine/{joint}"]
        to_stop = [f"position_controllers/{joint}_position_controller"]

        if not start:
            to_start, to_stop = to_stop, to_start

        if not self.switch_controller(
            start_controllers= to_start,
            stop_controllers= to_stop,
            strictness= SwitchControllerRequest.STRICT
        ).ok:
            rospy.logfatal(f"Failed to {'start' if start else 'stop'} calibration controller for {joint}")
            sys.exit(1)

    def refine_joint(self, joint):
        zero_offsets = []
        new_zero_offset = threading.Event()
        def zero_offset_cb(msg):
            nonlocal new_zero_offset, zero_offsets
            zero_offsets.append(msg.data)
            new_zero_offset.set()
        sub = rospy.Subscriber(f"calibration_controllers_for_refine/{joint}/zero_offset", Float32Msg, callback= zero_offset_cb)

        stop_refinement = threading.Event()
        def repeated_refinement():
            nonlocal stop_refinement, new_zero_offset
            while not stop_refinement.is_set():
                self.trigger_calibration_controller_during_refine(joint, start= True)

                while not new_zero_offset.wait(timeout= .1):
                    if rospy.is_shutdown():
                        rospy.loginfo('Shutting down')
                        sys.exit(1)
                    if stop_refinement.is_set():
                        break
                new_zero_offset.clear()

                self.trigger_calibration_controller_during_refine(joint, start= False)
                rospy.sleep(rospy.Duration(0.1))

        repeated_refinement_thread = threading.Thread(target= repeated_refinement).start()
        input("Press enter when enough samples are collected...")
        sub.unregister()
        stop_refinement.set()
        repeated_refinement_thread.join()

        if not rospy.ServiceProxy(f"set_zero_offset/{joint}/set_zero_offset", SetZeroOffset)(SetZeroOffsetRequest(offset= np.mean(zero_offsets))):
            rospy.logfatal(f"Failed to set averaged zero offset for {joint}")
            sys.exit(1)

    def refine_group_joints(self, group, joints):
        '''
        refine the given joints of the given group
        '''

        for j in joints:
            say(f"Refining joint {j}")
            self.switch_to_trajectory_controller(group)

            if j not in self.CALIBRATION_GROUP_POSITIONS:
                rospy.logfatal(f"Missing group calibration position for {j}. Cannot refine the joint.")
                sys.exit(1)
            group_position_for_j = self.CALIBRATION_GROUP_POSITIONS[j]

            say(f"Will move {group} to calibration position for {j}. Please press enter.")
            rospy.loginfo(f"Will move {group} to calibration position for {j}.")
            input("Press enter to continue...")

            self.move_to_group_positions(group, group_position_for_j)

            self.switch_to_position_controllers(group)

            self.refine_joint(j)

        self.switch_to_trajectory_controller(group)

    def run(self, goal):
        joint_groups = self.joints_from_goal(goal)
        joints = [j for g in joint_groups for j in g.joints]
        all_joints = [j for g in joint_groups for j in self.R[g.group]['joints'] + self.R[g.group]['extra_joints']]

        rpack = rospkg.RosPack()

        config_path = rpack.get_path('tams_pr2_refine_startup_calibration')

        for params, ns in rosparam.load_file(f"{config_path}/config/pr2_set_zero_offset_controllers.yaml", namespace='set_zero_offset'):
            rosparam.upload_params(ns,params)
        set_zero_offset_controllers = [f"set_zero_offset/{j}" for j in joints]

        for params, ns in rosparam.load_file(f"{config_path}/config/pr2_calibration_controllers_for_refine.yaml", namespace='calibration_controllers_for_refine'):
            rosparam.upload_params(ns,params)
        calibration_controllers_for_refine = [f"calibration_controllers_for_refine/{j}" for j in joints]

        for params, ns in rosparam.load_file(rpack.get_path('tams_pr2_controller_configuration') + '/config/pr2_joint_position_controllers.yaml', default_namespace='/position_controllers'):
            rosparam.upload_params(ns,params)
        position_controllers = [f"position_controllers/{j}_position_controller" for j in all_joints]

        trajectory_controllers =  set(self.R[jg.group]['trajectory'] for jg in joint_groups)

        self.ensure_controllers(
            calibration_controllers_for_refine +
            position_controllers +
            trajectory_controllers,
            start= False
        )

        self.ensure_controllers(set_zero_offset_controllers, start= True)

        for jg in joint_groups:
            self.refine_group_joints(jg.group, jg.joints)

        self.switch_to_trajectory_controller(jg.group for jg in joint_groups)

        self.unload_controllers(calibration_controllers_for_refine + position_controllers + set_zero_offset_controllers)

if __name__ == "__main__":
    rospy.init_node('refine_startup_calibration')
    sys.argv = rospy.myargv(argv=sys.argv)

    if len(sys.argv) < 2:
        rospy.logfatal("Usage: refine_startup_calibration.py <all/left_arm/right_arm/head/joint_name> [...]")
        sys.exit(1)

    RefineStartupCalibration().run(sys.argv[1:])