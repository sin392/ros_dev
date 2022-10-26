#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
import random
# import copy
import rospy
import moveit_commander
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from myrobot_moveit.msg import DetectedObjectsStamped
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty

def callback(msg):
    pass

rospy.wait_for_service("/clear_octomap")
clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_test")



# robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface(synchronous=True)
scene.remove_world_object()
scene.remove_attached_object()

robot = moveit_commander.RobotCommander()

# class tokade matometai
move_group_1 = moveit_commander.MoveGroupCommander("right_arm")
move_group_2 = moveit_commander.MoveGroupCommander("body_and_right_arm")
move_group_3 = moveit_commander.MoveGroupCommander("base_and_right_arm")
# move_group_4 = moveit_commander.MoveGroupCommander("left_arm")
# move_group_5 = moveit_commander.MoveGroupCommander("body_and_left_arm")
# move_group_6 = moveit_commander.MoveGroupCommander("base_and_left_arm")

topic = rospy.get_param("objects_topic", "/detected_objects")


clear_octomap()
while True:
    msg = rospy.wait_for_message(topic, DetectedObjectsStamped, timeout=None)
    if len(msg.objects) > 0:
        break
    
objects = msg.objects
target_object = random.choice(objects)
# rospy.loginfo(target_object)
pose = target_object.center
height = target_object.height 
radius = target_object.radius
# pose.pose.position.z -= height
# scene.add_cylinder("object", pose=pose, height=height, radius=radius)
scene.add_sphere("object", pose, radius=radius)
# move_group_1.set_support_surface_name("object")
# move_group_2.set_support_surface_name("object")
# move_group_3.set_support_surface_name("object")

table1_pose = PoseStamped()
table1_pose.header.frame_id = "world"
table1_pose.pose.position.x = 0
table1_pose.pose.position.y = -1
table1_pose.pose.position.z = 0.1
scene.add_box("table1", table1_pose, size=(0.5, 0.5, 0.2))
move_group_1.set_support_surface_name("table1")
move_group_2.set_support_surface_name("table1")
move_group_3.set_support_surface_name("table1")
rospy.sleep(0.1)

rospy.sleep(1)

rospy.loginfo(pose.pose)
rospy.loginfo(radius)
# create grasp msg
grasp = Grasp()
# setting grasp pose
grasp.grasp_pose.header.frame_id = "world"
q = quaternion_from_euler(0, math.pi, 0)
grasp.grasp_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
grasp.grasp_pose.pose.position = pose.pose.position
grasp.grasp_pose.pose.position.z += radius + 0.05
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

is_grasp_succeeded = move_group_1.pick("object", [grasp])
move_group_1.clear_pose_targets()
if is_grasp_succeeded == -1:
    print("replanning with body")
    is_grasp_succeeded = move_group_2.pick("object", [grasp])
    move_group_2.clear_pose_targets()
if is_grasp_succeeded == -1:
    print("replanning with body and back")
    is_grasp_succeeded = move_group_3.pick("object", [grasp])
    move_group_3.clear_pose_targets()
# # move_group.forget_joint_values()
# # move_group_2.forget_joint_values()
# # move_group_3.forget_joint_values()
current_joint_values = move_group_3.get_current_joint_values()
# move_group_3.set_joint_value_target(current_joint_values.update({"joint_back": 0, "joint_body": 0}))
# move_group_3.go()
# active_joints = move_group_3.get_active_joints()
# current_joint_values_dict = dict(zip(active_joints, current_joint_values))
# update_dict = current_joint_values_dict
# update_dict.update({"joint_back": 0, "joint_body": 0.358})

# move_group_2.set_joint_value_target(update_dict)
# move_group_2.go()


# setting place location pose
place_location = [PlaceLocation()]
place_location[0].place_pose.header.frame_id = "right_sholder"
orientation = quaternion_from_euler(0, 0, math.pi / 2)
place_location[0].place_pose.pose.orientation.x = orientation[0]
place_location[0].place_pose.pose.orientation.y = orientation[1]
place_location[0].place_pose.pose.orientation.z = orientation[2]
place_location[0].place_pose.pose.orientation.w = orientation[3]
# While placing it is the exact location of the center of the object.
place_location[0].place_pose.pose.position.x = 0
place_location[0].place_pose.pose.position.y = -1
place_location[0].place_pose.pose.position.z = 0.2 + radius + 0.1

# setting pre-place approach
place_location[0].pre_place_approach.direction.header.frame_id = "right_sholder"
place_location[0].pre_place_approach.direction.vector.z = -1.0 # 上から置く
place_location[0].pre_place_approach.min_distance = 0
place_location[0].pre_place_approach.desired_distance = 0.1

# setting post-grasp retreat
place_location[0].post_place_retreat.direction.header.frame_id = "right_sholder"
place_location[0].post_place_retreat.direction.vector.z = 1.0 # オブジェクトからロボット側にひく
# place_location[0].post_place_retreat.direction.vector.x = 1.0 # オブジェクトからロボット側にひく
place_location[0].post_place_retreat.min_distance = 0
# 複数の軸方向に個別の距離を指定することはできない？
place_location[0].post_place_retreat.desired_distance = 0.1

# setting posture of eef after placing object
place_location[0].post_place_posture.joint_names = ["right_panda_finger_joint1", "right_panda_finger_joint2"]
place_location[0].post_place_posture.points = [JointTrajectoryPoint()]
place_location[0].post_place_posture.points[0].positions = [0.04, 0.04]
place_location[0].post_place_posture.points[0].time_from_start = rospy.Duration(0.5)

move_group_3.place("object", place_location)