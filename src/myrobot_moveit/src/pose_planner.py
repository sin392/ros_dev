#!/usr/bin/env python2
# coding: UTF-8
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3


def main():
    # MoveitCommanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの生成
    rospy.init_node("pose_planner")

    # MoveGroupCommanderの準備
    move_group = moveit_commander.MoveGroupCommander("right_arm") # for open_manipulator
    # ref: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/
    move_group.set_end_effector_link("right_J6")
    print(move_group.get_current_pose())

    # エンドポイントの姿勢でゴール状態を指定
    # pose_goal = geometry_msgs.msg.Pose()
    pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.position = Vector3(0.0, 0.14, 0.3)
    pose_goal.header.frame_id = "world"
    pose_goal.pose.position = Vector3(1, -0.4, 0.6)
    q = tf.transformations.quaternion_from_euler(0, pi / 2, 0)
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose_goal.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    move_group.set_pose_target(pose_goal)

    # 追記：実行可能かの確認
    plan = move_group.plan()
    if not plan.joint_trajectory.points:
        rospy.logerr("No motion plan found")

    # モーションプランの計画と実行
    move_group.go(wait=True)

    # 後処理
    move_group.stop()
    move_group.clear_pose_targets()


if __name__ == "__main__":
    main()