#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
import rospy
import moveit_commander as mc
from moveit_msgs.msg import Grasp as BaseGrasp
from grasp_detection_client import GraspDetectionClient
from geometry_msgs.msg import Vector3, Quaternion
# from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

from octomap_handler import OctomapHandler

class Angle:
    deg_to_rad_ratio = math.pi / 180

    @classmethod
    def deg_to_rad(cls, degree):
        return cls.deg_to_rad_ratio * degree

class MoveGroup(mc.MoveGroupCommander):
    def __init__(self, name, parent=None):
        super(MoveGroup, self).__init__(name)
        self.parent = parent
        # self.next = mc.MoveGroupCommander(next_name)

    def get_current_joint_dict(self):
        return dict(zip(self.get_active_joints(), self.get_current_joint_values()))

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


    def plan(self, joints={}, is_degree=False, **kwargs):
        # if plan failed, switch move_group & plan again
        new_joints = joints
        new_joints.update(kwargs)
        if is_degree:
            new_joints = { k:Angle.deg_to_rad(v)  for k,v in new_joints.items()}

        merged_joints = self.whole_move_group.get_current_joint_dict()
        merged_joints.update(new_joints)
        
        return self.whole_move_group.plan(merged_joints)

    def execute(self, plan, wait):
        return self.whole_move_group.execute(plan, wait=wait)

    def pick(self, object_name, grasps):
        self.current_move_group.pick(object_name, grasps)

    def get_current_name(self):
        return self.current_move_group.get_name()

class PlanningSceneHandler(mc.PlanningSceneInterface):
    def __init__(self, raw_point_topics):
        super(PlanningSceneHandler, self).__init__(synchronous=True)

        self.oh = OctomapHandler(raw_point_topics)

    def clear_octomap(self):
        self.oh.clear()

    def update_octomap(self):
        self.oh.update()

class Grasp(BaseGrasp):
    def __init__(self, position=None, orientation=None,  xyz=(0, 0, 0), rpy=(0, 0, 0), frame_id="base_link"):
        super(Grasp, self).__init__()
        self.grasp_pose.header.frame_id = frame_id
        if position is None:
            position = Vector3(xyz[0], xyz[1], xyz[2])
        if orientation is None:
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.grasp_pose.pose.position = position
        self.grasp_pose.pose.orientation = orientation
        # setting pre-grasp approach
        self.pre_grasp_approach.direction.header.frame_id = frame_id
        self.pre_grasp_approach.direction.vector.z = -1.0
        self.pre_grasp_approach.min_distance = 0
        self.pre_grasp_approach.desired_distance = 0.1
        # setting post-grasp retreat
        self.post_grasp_retreat.direction.header.frame_id = frame_id
        self.post_grasp_retreat.direction.vector.z = 1.0
        self.post_grasp_retreat.min_distance = 0
        self.post_grasp_retreat.desired_distance = 0.1
        # setting posture of eef before grasp
        # setting posture of eef during grasp


class Myrobot:
    def __init__(self, fps, image_topic, depth_topic, raw_point_topics, wait = True):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = PlanningSceneHandler(raw_point_topics)

        mv_base_to_left_arm = MoveGroup("base_and_left_arm")
        # mv_base_to_right_arm = MoveGroup("base_and_right_arm")
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", parent=mv_base_to_left_arm)
        # mv_body_to_right_arm = MoveGroup("body_and_right_arm", parent=mv_base_to_right_arm)
        mv_left_arm = MoveGroup("left_arm", parent=mv_body_to_left_arm)
        # mv_right_arm = MoveGroup("right_arm", parent=mv_body_to_right_arm)

        mv_base_to_arms = MoveGroup("base_and_arms")

        self.mv_handler = MoveGroupHandler(mv_left_arm, mv_base_to_arms)
        
        self.gd_cli = GraspDetectionClient( 
            fps=fps, 
            image_topic=image_topic, 
            depth_topic=depth_topic,
            wait=wait
        )

    # TODO: to be able to change joint_body
    def initialize_whole_pose(self):
        self.mv_handler.initialize_whole_pose("base_and_arms_start")

    def get_around_octomap(self, values=[-30, 30, 0], is_degree=False, should_reset=True):
        if should_reset:
            self.scene_handler.clear_octomap()
        for value in values:
            plan = self.plan(joint_back=value, is_degree=is_degree)
            self.execute(plan, wait=True)
            rospy.sleep(1)
            self.scene_handler.update_octomap()

    def plan(self, joints={}, is_degree=False, **kwargs):
        return self.mv_handler.plan(joints, is_degree, **kwargs)

    def execute(self, plan, wait=False):
        return self.mv_handler.execute(plan, wait)

    def pick(self, object_name, grasps):
        return self.mv_handler.pick(object_name, grasps)

    def detect(self):
        return self.gd_cli.detect()

    def info(self):
        print("-" * 30)
        print("/// robot commander ///")
        print("planinng frame: {}".format(self.robot.get_planning_frame()))
        print("group names: {}".format(self.robot.get_group_names()))
        print("/// scene interface ///")
        print("known objects: {}".format(self.scene_handler.get_known_object_names()))
        print("/// move_group commander ///")
        print("current group: {}".format(self.mv_handler.get_current_name()))
        print("end effector: {}".format(self.mv_handler.current_move_group.get_end_effector_link()))
        print("-" * 30)


if __name__ == "__main__":
    rospy.init_node("planning_node")

    # ref: http://zumashi.blogspot.com/2016/10/rosrun.html
    ns = rospy.get_param("robot_name", default="myrobot")
    fps = rospy.get_param("fps", default=1)
    image_topic = rospy.get_param("image_topic")
    depth_topic = rospy.get_param("depth_topic")
    sensors = rospy.get_param("~sensors", default=("body_camera", "left_camera", "right_camera"))
    raw_point_topics = ["/{}/{}/depth/color/points".format(ns, sensor_name) for sensor_name in sensors]

    wait = rospy.get_param("~wait_server", default=True)
    rospy.loginfo("################################################")

    print("initializing instances...")
    myrobot = Myrobot(fps=fps, image_topic=image_topic, depth_topic=depth_topic, raw_point_topics=raw_point_topics, wait=wait)
    myrobot.info()
    rospy.sleep(5)

    print("getting around octomap...")
    myrobot.get_around_octomap(values=[-30, 30, 0], is_degree=True, should_reset=True)

    # grasp = Grasp(xyz=(1.5, 0, 0.2), rpy=(0, math.pi, 0))

    print("stating detect flow...")
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        objects = myrobot.detect()
        print("objects: {}".format(len(objects)))
        if len(objects) == 0:
            continue

        obj = objects[0]
        obj_position_point = obj.center_pose.pose.position
        obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, 1)
        obj_orientation = obj.center_pose.pose.orientation
        print(obj_position_vector)
        print(obj_orientation)
        grasp = Grasp(position=obj_position_vector, rpy=(0, math.pi, 0))

        myrobot.scene_handler.add_sphere("object", obj.center_pose, radius=obj.short_radius)
        myrobot.mv_handler.current_move_group.pick("object", [grasp])
        myrobot.scene_handler.update_octomap()

        print("will initialize")
        myrobot.initialize_whole_pose()
        myrobot.scene_handler.update_octomap()

        break

        rate.sleep()