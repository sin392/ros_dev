#!/usr/bin/env python2
# coding: UTF-8

import sys
import rospy
import moveit_commander as mc
from moveit_msgs.msg import Grasp
from grasp_detection_client import GraspDetectionClient
from std_srvs.srv import Empty

class MoveGroup(mc.MoveGroupCommander):
    def __init__(self, name, parent=None):
        super(MoveGroup, self).__init__(name)
        self.parent = parent
        # self.next = mc.MoveGroupCommander(next_name)

    def get_current_joint_dict(self):
        return dict(zip(self.get_active_joints, self.get_current_joint_values))

class MoveGroupHandler:
    def __init__(self, start_move_group, whole_move_group):
        self.start_move_group = start_move_group
        self.whole_move_group = whole_move_group
        self.current_move_group = self.start_move_group

    def reset_move_group(self):
        # TODO: also reset joint values
        self.current_move_group = self.start_move_group

    def initialize_whole_pose(self, target_name):
        target_joint_dict = self.whole_move_group.get_named_target_values(target_name)
        plan = self.whole_move_group.plan(target_joint_dict)
        self.whole_move_group.execute(plan)


    def plan(self, joints={}, **kwargs):
        # if plan failed, switch move_group & plan again
        merged_joints = self.whole_move_group.get_current_joint_dict()
        merged_joints.update(joints)
        merged_joints.update(kwargs)
        
        return self.current_move_group.plan(merged_joints)

    def execute(self, plan):
        self.current_move_group.execute(plan)

    def get_current_name(self):
        return self.current_move_group.get_name()

class PlanningSceneHandler(mc.PlanningSceneInterface):
    def __init__(self):
        super(PlanningSceneHandler, self).__init__(synchronous=True)

        rospy.wait_for_service('/clear_octomap')
        self.clear_octomap_proxy = rospy.ServiceProxy('/clear_octomap', Empty)

    def clear_octomap(self):
        self.clear_octomap_proxy()


class Myrobot:
    def __init__(self):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = PlanningSceneHandler()
        self.scene_handler.clear_octomap()

        mv_base_to_left_arm = MoveGroup("base_and_left_arm")
        # mv_base_to_right_arm = MoveGroup("base_and_right_arm")
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", parent=mv_base_to_left_arm)
        # mv_body_to_right_arm = MoveGroup("body_and_right_arm", parent=mv_base_to_right_arm)
        mv_left_arm = MoveGroup("left_arm", parent=mv_body_to_left_arm)
        # mv_right_arm = MoveGroup("right_arm", parent=mv_body_to_right_arm)

        mv_base_to_arms = MoveGroup("base_and_arms")

        self.mv_handler = MoveGroupHandler(mv_left_arm, mv_base_to_arms)
        
        self.gd_cli = GraspDetectionClient( 
            fps=1, 
            image_topic="/myrobot/body_camera/color/image_raw", 
            depth_topic="/myrobot/body_camera/aligned_depth_to_color/image_raw"
        )

    # TODO: to be able to change joint_body
    def initialize_whole_pose(self):
        self.mv_handler.initialize_whole_pose("base_and_arms_start")

    def plan(self, joints={}, **kwargs):
        return self.mv_handler.plan(joints, **kwargs)

    def execute(self, plan):
        return self.mv_handler.execute(plan)

    def detect(self):
        return self.gd_cli.detect()

    def info(self):
        print("/// robot commander ///")
        print("planinng frame: {}".format(self.robot.get_planning_frame()))
        print("group names: {}".format(self.robot.get_group_names()))
        print("/// scene interface ///")
        print("known objects: {}".format(self.scene_handler.get_known_object_names()))
        print("/// move_group commander ///")
        print("current group: {}".format(self.mv_handler.get_current_name()))
        print("end effector: {}".format(self.mv_handler.current_move_group.has_end_effector_link()))


if __name__ == "__main__":
    rospy.init_node("planning_node")
    myrobot = Myrobot()

    myrobot.info()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        objects = myrobot.detect()
        print("objects: {}".format(len(objects)))

        rate.sleep()