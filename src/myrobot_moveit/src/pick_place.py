#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
# import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from tf.transformations import quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("pick_and_place")

robot = moveit_commander.RobotCommander()
print(robot.get_planning_frame())

scene = moveit_commander.PlanningSceneInterface(synchronous=True)

group_name = "right_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# clean the scene
scene.remove_world_object()
rospy.sleep(1)

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


# create object
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
# box_pose.pose.orientation.w = 1.0
# box_pose.pose.position.z = 0.07 # slightly above the end effector
# box_pose.pose.position.z = 0.3 # slightly above the end effector

box_pose.pose.position.x = 1
box_pose.pose.position.y = 0
box_pose.pose.position.z = 0.2
box_name = "object"
scene.add_box(box_name, box_pose, size=(0.02, 0.02, 0.2))
rospy.sleep(0.1)

# create grasp msg
grasp = moveit_msgs.msg.Grasp()
# setting grasp pose
grasp.grasp_pose.header.frame_id = "world"
# RPY convert (ロール・ピッチ・ヨーなことに注意)
# orientation = quaternion_from_euler(-math.pi / 2, -math.pi / 4, -math.pi / 2)
orientation = quaternion_from_euler(-math.pi / 2,  math.pi / 2, 0)
# orientationの展開忘れないように注意！！
grasp.grasp_pose.pose.orientation.x = orientation[0]
grasp.grasp_pose.pose.orientation.y = orientation[1]
grasp.grasp_pose.pose.orientation.z = orientation[2]
grasp.grasp_pose.pose.orientation.w = orientation[3]
# dinstance_to_object - half_width_of_object - distance_from_arm_head_to_palm - padding
# grasp.grasp_pose.pose.position.x = 0.915
grasp.grasp_pose.pose.position.x = 1
grasp.grasp_pose.pose.position.y = -(0.058 + 0.03)
grasp.grasp_pose.pose.position.z = 0.3
# setting pre-grasp approach
grasp.pre_grasp_approach.direction.header.frame_id = "world"
grasp.pre_grasp_approach.direction.vector.y = 1.0
grasp.pre_grasp_approach.min_distance = 0.095
grasp.pre_grasp_approach.desired_distance = 0.115
# setting post-grasp retreat
grasp.post_grasp_retreat.direction.header.frame_id = "world"
grasp.post_grasp_retreat.direction.vector.z = 1.0
grasp.post_grasp_retreat.min_distance = 0.1
grasp.post_grasp_retreat.desired_distance = 0.25
# setting posture of eef before grasp
grasp.pre_grasp_posture.joint_names = ["right_panda_finger_joint1", "right_panda_finger_joint2"]
grasp.pre_grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
grasp.pre_grasp_posture.points[0].positions = [0.04, 0.04]
grasp.pre_grasp_posture.points[0].time_from_start = rospy.Duration(0.5)
# setting posture of eef during grasp
grasp.grasp_posture.joint_names = ["right_panda_finger_joint1", "right_panda_finger_joint2"]
grasp.grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
grasp.grasp_posture.points[0].positions = [0.00, 0.00]
grasp.grasp_posture.points[0].time_from_start = rospy.Duration(0.5)


# move_group.set_support_surface_name("table1")


print(robot)
move_group.pick("object", [grasp])
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