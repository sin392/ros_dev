#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
# import copy
import rospy
import moveit_commander
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("pick_and_place")

robot = moveit_commander.RobotCommander()
print("planinng frame :", robot.get_planning_frame())
print("group names :", robot.get_group_names())

scene = moveit_commander.PlanningSceneInterface(synchronous=True)

move_group = moveit_commander.MoveGroupCommander("right_arm")
move_group_2 = moveit_commander.MoveGroupCommander("body_and_right_arm")
move_group_3 = moveit_commander.MoveGroupCommander("base_and_right_arm")
# sub group 複数含んでるとsetできないぽい
# move_group.set_end_effector_link("right_panda_hand_tip")
print("eef :", move_group.get_end_effector_link())

# clean the scene
scene.remove_world_object()
scene.remove_attached_object()
rospy.sleep(0.1)

# create table1
# table1_name = "table1"
# table1_pose =  geometry_msgs.msg.PoseStamped()
# table1_pose.header.frame_id = "world"
# # table1_pose.pose.orientation.w = 1.0
# table1_pose.pose.position.x = 0.5
# # table1_pose.pose.position.y = 0.25
# table1_pose.pose.position.y = 0
# table1_pose.pose.position.z = 0.2
# scene.add_box(table1_name, table1_pose, size=(0.2, 0.4, 0.4))
# rospy.sleep(0.1)

# create table2
# table2_name = "table2"
# table2_pose =  geometry_msgs.msg.PoseStamped()
# table2_pose.header.frame_id = "world"
# # table2_pose.pose.orientation.w = 1.0
# table2_pose.pose.position.x = 0
# # table2_pose.pose.position.y = -0.25
# table2_pose.pose.position.y = 0.5
# table2_pose.pose.position.z = 0.2
# scene.add_box(table2_name, table2_pose, size=(0.4, 0.2, 0.4))
# rospy.sleep(0.1)

# create grasp msg
grasp = Grasp()
# setting grasp pose
grasp.grasp_pose.header.frame_id = "world"
q = quaternion_from_euler(0, math.pi, 0)
grasp.grasp_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
grasp.grasp_pose.pose.position = Vector3(1.5, 0, 0.2)
# setting pre-grasp approach
grasp.pre_grasp_approach.direction.header.frame_id = "world"
grasp.pre_grasp_approach.direction.vector.z = -1.0
grasp.pre_grasp_approach.min_distance = 0
grasp.pre_grasp_approach.desired_distance = 0.1
# setting post-grasp retreat
grasp.post_grasp_retreat.direction.header.frame_id = "world"
grasp.post_grasp_retreat.direction.vector.z = 1.0
grasp.post_grasp_retreat.min_distance = 0
grasp.post_grasp_retreat.desired_distance = 0.1
# setting posture of eef before grasp
grasp.pre_grasp_posture.joint_names = ["right_panda_finger_joint1"]
grasp.pre_grasp_posture.points = [JointTrajectoryPoint()]
grasp.pre_grasp_posture.points[0].positions = [0.04]
grasp.pre_grasp_posture.points[0].time_from_start = rospy.Duration(0.5)
# setting posture of eef during grasp
grasp.grasp_posture.joint_names = ["right_panda_finger_joint1"]
grasp.grasp_posture.points = [JointTrajectoryPoint()]
grasp.grasp_posture.points[0].positions = [0.00]
grasp.grasp_posture.points[0].time_from_start = rospy.Duration(0.5)


# move_group.set_support_surface_name("table1")

# create object
box_pose = PoseStamped()
box_pose.header.frame_id = "world"
box_name = "object"
box_pose.pose.position = Vector3(1.5, 0, 0.1)
# scene.add_box(box_name, box_pose, size=(0.02, 0.02, 0.2))

# 把持ポーズの候補を与える
is_grasp_succeeded = move_group.pick("", [grasp])
print(is_grasp_succeeded, bool(is_grasp_succeeded))
if is_grasp_succeeded == -1:
    print("replanning with body")
    is_grasp_succeeded = move_group_2.pick("", [grasp])
if is_grasp_succeeded == -1:
    print("replanning with body and back")
    is_grasp_succeeded = move_group_3.pick("", [grasp])
move_group.clear_pose_targets()
move_group_2.clear_pose_targets()
move_group_3.clear_pose_targets()
# move_group.forget_joint_values()
# move_group_2.forget_joint_values()
# move_group_3.forget_joint_values()
current_joint_values = move_group_3.get_current_joint_values()
# move_group_3.set_joint_value_target(current_joint_values.update({"joint_back": 0, "joint_body": 0}))
# move_group_3.go()
active_joints = move_group_3.get_active_joints()
current_joint_values_dict = dict(zip(active_joints, current_joint_values))
update_dict = current_joint_values_dict
update_dict.update({"joint_back": 0, "joint_body": 0.358})

move_group_2.set_joint_value_target(update_dict)
move_group_2.go()
# move_group.pick("object")

# # setting place location pose
# place_location = [moveit_msgs.msg.PlaceLocation()]
# place_location[0].place_pose.header.frame_id = "right_sholder"
# orientation = quaternion_from_euler(0, 0, math.pi / 2)
# place_location[0].place_pose.pose.orientation.x = orientation[0]
# place_location[0].place_pose.pose.orientation.y = orientation[1]
# place_location[0].place_pose.pose.orientation.z = orientation[2]
# place_location[0].place_pose.pose.orientation.w = orientation[3]
# # While placing it is the exact location of the center of the object.
# place_location[0].place_pose.pose.position.x = 0
# place_location[0].place_pose.pose.position.y = -1
# place_location[0].place_pose.pose.position.z = 0.2

# # setting pre-place approach
# place_location[0].pre_place_approach.direction.header.frame_id = "right_sholder"
# place_location[0].pre_place_approach.direction.vector.z = -1.0 # 上から置く
# place_location[0].pre_place_approach.min_distance = 0.095
# place_location[0].pre_place_approach.desired_distance = 0.115

# # setting post-grasp retreat
# place_location[0].post_place_retreat.direction.header.frame_id = "right_sholder"
# place_location[0].post_place_retreat.direction.vector.y = -1.0 # オブジェクトからロボット側にひく
# # place_location[0].post_place_retreat.direction.vector.x = 1.0 # オブジェクトからロボット側にひく
# place_location[0].post_place_retreat.min_distance = 0.1
# # 複数の軸方向に個別の距離を指定することはできない？
# place_location[0].post_place_retreat.desired_distance = 0.25

# # setting posture of eef after placing object
# place_location[0].post_place_posture.joint_names = ["right_panda_finger_joint1", "right_panda_finger_joint2"]
# place_location[0].post_place_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
# place_location[0].post_place_posture.points[0].positions = [0.04, 0.04]
# place_location[0].post_place_posture.points[0].time_from_start = rospy.Duration(0.5)

# move_group.place("object", place_location)