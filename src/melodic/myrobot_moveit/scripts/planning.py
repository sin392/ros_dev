#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
from random import randint
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
    def __init__(self, left_start_move_group, right_start_move_group, start_move_group, whole_move_group):
        self.left_start_move_group = left_start_move_group
        self.right_start_move_group = right_start_move_group
        self.start_move_group = start_move_group
        self.whole_move_group = whole_move_group
        self.whole_name = whole_move_group.get_name()
        self.set_current_move_group(self.start_move_group)

    def set_current_move_group(self, move_group):
        self.current_move_group = move_group
        rospy.loginfo("current move group is changed to '{}'".format(self.get_current_name()))

    def reset_move_group(self):
        # TODO: also reset joint values
        self.current_move_group = self.start_move_group

    def initialize_current_pose(self):
        group_name = self.get_current_name()
        target_name = "{}_default".format(group_name)
        target_joint_dict = self.current_move_group.get_named_target_values(target_name)
        plan = self.current_move_group.plan(target_joint_dict)
        self.current_move_group.execute(plan)

    def initialize_whole_pose(self):
        target_name = "{}_default".format(self.whole_name)
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
        res =  self.whole_move_group.execute(plan, wait=wait)
        self.whole_move_group.stop()
        self.whole_move_group.clear_pose_targets()
        return res

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
    def __init__(self, approach_desired_distance, approach_min_distance, 
                 retreat_desired_distance, retreat_min_distance, 
                 position=None, orientation=None, xyz=(0, 0, 0), rpy=(0, 0, 0), 
                 frame_id="base_link", finger_joints=[], allowed_touch_objects=[]):
        super(Grasp, self).__init__()
        # setting grasp-pose: this is for parent_link
        self.grasp_pose.header.frame_id = frame_id
        self.allowed_touch_objects = allowed_touch_objects
        if position is None:
            position = Vector3(xyz[0], xyz[1], xyz[2])
        if orientation is None:
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.grasp_pose.pose.position = position
        self.grasp_pose.pose.orientation = orientation
        # setting pre-grasp approach
        self.pre_grasp_approach.direction.header.frame_id = frame_id
        self.pre_grasp_approach.direction.vector.z = -1
        self.pre_grasp_approach.min_distance = approach_min_distance
        self.pre_grasp_approach.desired_distance = approach_desired_distance
        # setting post-grasp retreat
        self.post_grasp_retreat.direction.header.frame_id = frame_id
        self.post_grasp_retreat.direction.vector.z = 1
        self.post_grasp_retreat.min_distance = retreat_min_distance
        self.post_grasp_retreat.desired_distance = retreat_desired_distance
        # setting posture of eef before grasp
        self.pre_grasp_posture.joint_names = finger_joints
        # self.pre_grasp_posture.points = [JointTrajectoryPoint()]
        # self.pre_grasp_posture.points[0].positions = [0]
        # self.pre_grasp_posture.points[0].time_from_start = rospy.Duration(0.5)
        # setting posture of eef during grasp
        self.grasp_posture.joint_names = finger_joints
        # self.grasp_posture.points = [JointTrajectoryPoint()]
        # self.pre_grasp_posture.points[0].positions = [0]
        # self.pre_grasp_posture.points[0].time_from_start = rospy.Duration(0.5)


class Myrobot:
    def __init__(self, fps, image_topic, depth_topic, raw_point_topics, wait = True):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = PlanningSceneHandler(raw_point_topics)

        # left groups
        mv_base_to_left_arm = MoveGroup("base_and_left_arm")
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", parent=mv_base_to_left_arm)
        # mv_left_arm = MoveGroup("left_arm", parent=mv_body_to_left_arm)
        # right groups
        mv_base_to_right_arm = MoveGroup("base_and_right_arm")
        mv_body_to_right_arm = MoveGroup("body_and_right_arm", parent=mv_base_to_right_arm)
        # mv_right_arm = MoveGroup("right_arm", parent=mv_body_to_right_arm)
        # whole group
        mv_base_to_arms = MoveGroup("base_and_arms")

        self.mv_handler = MoveGroupHandler(mv_body_to_left_arm, mv_body_to_right_arm, mv_body_to_left_arm, mv_base_to_arms)
        
        self.gd_cli = GraspDetectionClient( 
            fps=fps, 
            image_topic=image_topic, 
            depth_topic=depth_topic,
            wait=wait
        )

    def initialize_current_pose(self):
        self.mv_handler.initialize_current_pose()

    def initialize_whole_pose(self):
        self.mv_handler.initialize_whole_pose()

    def get_around_octomap(self, values=[-30, 30, 0], is_degree=False, should_reset=True):
        if should_reset:
            self.scene_handler.clear_octomap()
        for value in values:
            plan = self.plan(joint_back=value, is_degree=is_degree)
            self.execute(plan, wait=True)
            self.scene_handler.update_octomap()

    def plan(self, joints={}, is_degree=False, **kwargs):
        return self.mv_handler.plan(joints, is_degree, **kwargs)

    def execute(self, plan, wait=False):
        res =  self.mv_handler.execute(plan, wait)
        return res

    def pick(self, object_name, object_msg, approach_desired_distance=0.05, approach_min_distance=0.01, retreat_desired_distance=0.05, retreat_min_distance=0.01):
        obj_position_point = object_msg.center_pose.pose.position
        z = max(obj_position_point.z - object_msg.length_to_center / 2, 0.01)
        print("z: {}".format(z))
        obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, z)
        radian = Angle.deg_to_rad(object_msg.angle)
        # TODO: change grsp frame_id from "base_link" to each hand frame

        arm_index = self.select_arm(obj_position_vector.y)
        finger_joints = ["left_finger_1_joint"] if arm_index == 0 else ["right_finger_1_joint"] 
        grasp = Grasp(
            position=obj_position_vector,
            rpy=(math.pi, 0, radian),
            approach_desired_distance=approach_desired_distance,
            approach_min_distance=approach_min_distance,
            retreat_desired_distance=retreat_desired_distance,
            retreat_min_distance=retreat_min_distance,
            finger_joints=finger_joints,
            allowed_touch_objects=[object_name]
        )
        res = self.mv_handler.pick(object_name, [grasp])
        return res

    def detect(self):
        return self.gd_cli.detect()

    def select_arm(self, y):
        # 0: left, 1: right
        arm_index =  0 if y > 0 else 1
        print("y: {}, arm_index: {}".format(y, arm_index))
        new_move_group = self.mv_handler.left_start_move_group if arm_index == 0 else self.mv_handler.right_start_move_group
        self.mv_handler.set_current_move_group(new_move_group)
        return arm_index

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
    myrobot.initialize_whole_pose()

    print("getting around octomap...")
    myrobot.get_around_octomap(values=[-30, 30, 0], is_degree=True, should_reset=True)

    print("stating detect flow...")
    registered_objects = []
    while not rospy.is_shutdown():
        objects = myrobot.detect()
        print("objects: {}".format(len(objects)))
        if len(objects) == 0:
            continue

        target_index = randint(0, len(objects) - 1)
        obj = objects[target_index]
        obj_name = "object_{}".format(len(registered_objects))

        # add object
        obj_pose = obj.center_pose
        # obj_pose.pose.position.z -= obj.length_to_center / 2
        obj_pose.pose.orientation = Quaternion()
        insert_depth = obj.length_to_center
        myrobot.scene_handler.add_cylinder(obj_name, obj_pose, height=insert_depth, radius=obj.long_radius)
        myrobot.scene_handler.update_octomap()
        
        # pick
        print("start pick")
        myrobot.pick(obj_name, obj, 
                     approach_desired_distance=insert_depth * 1.5,
                     retreat_desired_distance=insert_depth * 2,
                     approach_min_distance=insert_depth,
                     retreat_min_distance=insert_depth * 1.5
        )

        print("will initialize")
        myrobot.initialize_current_pose()

        myrobot.scene_handler.remove_attached_object("")
        myrobot.scene_handler.remove_world_object()

        myrobot.scene_handler.update_octomap()