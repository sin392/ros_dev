#!/usr/bin/env python2
# coding: UTF-8

import sys
# import copy
import rospy
import moveit_commander
# import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("add_box_node")

scene = moveit_commander.PlanningSceneInterface(synchronous=True)

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1
box_pose.pose.position.x = 1
box_pose.pose.position.z = 0.2
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
rospy.sleep(3)
scene.remove_world_object(box_name)
