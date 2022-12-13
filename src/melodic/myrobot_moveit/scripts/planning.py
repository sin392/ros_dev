#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
from random import randint
import rospy
from std_msgs.msg import Header
import moveit_commander as mc
from moveit_msgs.msg import Grasp as BaseGrasp, PlaceLocation as BasePlaceLocation, Constraints, OrientationConstraint
from grasp_detection_client import GraspDetectionClient
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

from octomap_handler import OctomapHandler

class Angle:
    deg_to_rad_ratio = math.pi / 180

    @classmethod
    def deg_to_rad(cls, degree):
        return cls.deg_to_rad_ratio * degree

class MoveGroup(mc.MoveGroupCommander):
    def __init__(self, name, parent=None, constraint=Constraints()):
        super(MoveGroup, self).__init__(name)
        self.constraint = constraint
        self.parent = parent
        # self.next = mc.MoveGroupCommander(next_name)
        self.set_path_constraints(constraint)

    def get_current_joint_dict(self):
        return dict(zip(self.get_active_joints(), self.get_current_joint_values()))

class MoveGroupHandler:
    def __init__(self, left_start_move_group, right_start_move_group, start_move_group, whole_move_group):
        self.left_start_move_group = left_start_move_group
        self.left_eef_default_pose = left_start_move_group.get_current_pose().pose
        self.right_start_move_group = right_start_move_group
        self.right_eef_default_pose = right_start_move_group.get_current_pose().pose
        self.start_move_group = start_move_group
        self.start_eef_default_pose = start_move_group.get_current_pose().pose
        self.whole_move_group = whole_move_group
        self.whole_name = whole_move_group.get_name()
        self.set_current_move_group(self.start_move_group, self.start_eef_default_pose)

    def set_current_move_group(self, move_group, default_eef_pose=None):
        self.current_move_group = move_group
        self.current_eef_default_pose = default_eef_pose
        rospy.loginfo("current move group is changed to '{}'".format(self.get_current_name()))

    def reset_move_group(self):
        # TODO: also reset joint values
        self.current_move_group = self.start_move_group
        self.current_eef_default_pose = self.start_eef_default_pose
        
    def initialize_current_pose(self, cartesian_mode=False, c_eef_step=0.01, c_jump_threshold=0.0, wait=True):
        group_name = self.get_current_name()
        target_name = "{}_default".format(group_name)
        target_joint_dict = self.current_move_group.get_named_target_values(target_name)
        if cartesian_mode:
            waypoints = [self.current_eef_default_pose]
            plan, _ = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
        else:
            plan = self.current_move_group.plan(target_joint_dict)
        self.current_move_group.execute(plan, wait=wait)

    def initialize_whole_pose(self, wait=True):
        target_name = "{}_default".format(self.whole_name)
        target_joint_dict = self.whole_move_group.get_named_target_values(target_name)
        plan = self.whole_move_group.plan(target_joint_dict)
        self.whole_move_group.execute(plan, wait=wait)


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

    def pick(self, object_name, grasps, pre_move=False, c_eef_step=0.01, c_jump_threshold=0.0):
        if pre_move:
            pre_pose = self.current_eef_default_pose
            grasp_position = grasps[0].grasp_pose.pose.position # x, y are same among grasps
            apploach_desired_distance = grasps[0].pre_grasp_approach.desired_distance
            pre_pose.position.x = grasp_position.x
            pre_pose.position.y = grasp_position.y
            pre_pose.position.y = apploach_desired_distance
            waypoints = [pre_pose]
            plan, _ = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
            self.execute(plan, wait=True)
        
        return self.current_move_group.pick(object_name, grasps)

    def place(self, object_name, locations):
        return self.current_move_group.place(object_name, locations)

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
        self.pre_grasp_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(2.0))]
        # setting posture of eef during grasp
        self.grasp_posture.joint_names = finger_joints
        self.grasp_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(2.0))]

class PlaceLocation(BasePlaceLocation):
    def __init__(self, approach_desired_distance, approach_min_distance, 
                 retreat_desired_distance, retreat_min_distance, 
                 position=None, orientation=None, xyz=(0, 0, 0), rpy=(0, 0, 0), 
                 frame_id="base_link", finger_joints=[], allowed_touch_objects=[]):
        super(PlaceLocation, self).__init__()
        # setting place-pose: this is for parent_link
        self.place_pose.header.frame_id = frame_id
        self.allowed_touch_objects = allowed_touch_objects
        if position is None:
            position = Vector3(xyz[0], xyz[1], xyz[2])
        if orientation is None:
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.place_pose.pose.position = position
        self.place_pose.pose.orientation = orientation
        # setting pre-place approach
        self.pre_place_approach.direction.header.frame_id = frame_id
        self.pre_place_approach.direction.vector.z = -1
        self.pre_place_approach.min_distance = approach_min_distance
        self.pre_place_approach.desired_distance = approach_desired_distance
        # setting post-place retreat
        self.post_place_retreat.direction.header.frame_id = frame_id
        self.post_place_retreat.direction.vector.z = 1
        self.post_place_retreat.min_distance = retreat_min_distance
        self.post_place_retreat.desired_distance = retreat_desired_distance
        # setting posture of eef after place
        self.post_place_posture.joint_names = finger_joints
        self.post_place_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(0.5))]


class Myrobot:
    def __init__(self, fps, image_topic, depth_topic, raw_point_topics, wait = True, use_constraint = False):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = PlanningSceneHandler(raw_point_topics)

        # constraints
        if use_constraint:
            constraint_rpy = (0, math.pi, math.pi / 4) # TODO: compute z from finger property (now 45 for 4 fingers)
            constraint_xyz_tolerance = (0.45, 0.45, 3.6) # TODO: update this value
            left_hand_constraint = self._create_constraint("left_hand_constraint", link_name="left_soft_hand_tip", 
                                                        rpy=constraint_rpy, xyz_tolerance=constraint_xyz_tolerance)
            right_hand_constraint = self._create_constraint("right_hand_constraint", link_name="right_soft_hand_tip", 
                                                        rpy=constraint_rpy, xyz_tolerance=constraint_xyz_tolerance)
        else:
            left_hand_constraint = Constraints()
            right_hand_constraint = Constraints()

        # left groups
        mv_base_to_left_arm = MoveGroup("base_and_left_arm", constraint=left_hand_constraint)
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", parent=mv_base_to_left_arm, constraint=left_hand_constraint)
        # mv_left_arm = MoveGroup("left_arm", parent=mv_body_to_left_arm, constraint=constraint)
        # right groups
        mv_base_to_right_arm = MoveGroup("base_and_right_arm", constraint=right_hand_constraint)
        mv_body_to_right_arm = MoveGroup("body_and_right_arm", parent=mv_base_to_right_arm, constraint=right_hand_constraint)
        # mv_right_arm = MoveGroup("right_arm", parent=mv_body_to_right_arm, constraint=constraint)
        # whole group
        mv_base_to_arms = MoveGroup("base_and_arms")

        self.mv_handler = MoveGroupHandler(mv_body_to_left_arm, mv_body_to_right_arm, mv_body_to_left_arm, mv_base_to_arms)

        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position =Vector3(0, -1, 0)
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        box_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.scene_handler.add_box("table", box_pose, size=(0.5, 0.5, 0.2))
            
        self.gd_cli = GraspDetectionClient( 
            fps=fps, 
            image_topic=image_topic, 
            depth_topic=depth_topic,
            wait=wait
        )

    def _create_constraint(self, name, link_name, rpy, base_frame_id="base_link", xyz_tolerance=(0.05, 0.05, 3.6)):
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        constraint = Constraints(
            name=name,
            orientation_constraints = [OrientationConstraint(
                header=Header(frame_id=base_frame_id),
                link_name=link_name,
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                # allow max rotations
                absolute_x_axis_tolerance = xyz_tolerance[0],
                absolute_y_axis_tolerance = xyz_tolerance[1],
                absolute_z_axis_tolerance = xyz_tolerance[2],
                weight = 1
            )]
        )
        return constraint

    def initialize_current_pose(self, cartesian_mode=False, c_eef_step=0.01, c_jump_threshold=0.0):
        self.mv_handler.initialize_current_pose(cartesian_mode, c_eef_step, c_jump_threshold)
        self.mv_handler.reset_move_group()

    def initialize_whole_pose(self):
        self.mv_handler.initialize_whole_pose()
        self.mv_handler.reset_move_group()

    def get_around_octomap(self, values=[-30, 30, 0], sleep_time=0, is_degree=False, should_reset=True):
        if should_reset:
            self.scene_handler.clear_octomap()
        for value in values:
            plan = self.plan(joint_back=value, is_degree=is_degree)
            self.execute(plan, wait=True)
            rospy.sleep(sleep_time)
            self.scene_handler.update_octomap()

    def plan(self, joints={}, is_degree=False, **kwargs):
        return self.mv_handler.plan(joints, is_degree, **kwargs)

    def execute(self, plan, wait=False):
        res =  self.mv_handler.execute(plan, wait)
        return res

    def pick(self, object_name, object_msg, 
             pre_move=False, c_eef_step=0.01, c_jump_threshold=0.0,
             approach_desired_distance=0.1, approach_min_distance=0.05, retreat_desired_distance=0.1, retreat_min_distance=0.05):
        obj_position_point = object_msg.center_pose.pose.position
        z = max(obj_position_point.z - object_msg.length_to_center / 2, 0.01)
        print("z: {}".format(z))
        obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, z)
        # TODO: change grsp frame_id from "base_link" to each hand frame
        arm_index = self.select_arm(obj_position_vector.y)
        finger_joints = ["left_finger_1_joint"] if arm_index == 0 else ["right_finger_1_joint"] 
        grasps = [Grasp(
            position=obj_position_vector,
            rpy=(math.pi, 0, Angle.deg_to_rad(angle)),
            approach_desired_distance=approach_desired_distance,
            approach_min_distance=approach_min_distance,
            retreat_desired_distance=retreat_desired_distance,
            retreat_min_distance=retreat_min_distance,
            finger_joints=finger_joints,
            allowed_touch_objects=[object_name]
        )  for angle in object_msg.angles]
        res = self.mv_handler.pick(object_name, grasps, pre_move, c_eef_step, c_jump_threshold)
        print("pick res: {}".format(res))
        return res, arm_index

    def place(self, arm_index, object_name, approach_desired_distance=0.05, approach_min_distance=0.01, retreat_desired_distance=0.05, retreat_min_distance=0.01):
        finger_joints = ["left_finger_1_joint"] if arm_index == 0 else ["right_finger_1_joint"] 
        location = PlaceLocation(xyz=(0, -1.25, 1.2), 
                                 approach_desired_distance=approach_desired_distance,
                                 approach_min_distance=approach_min_distance,
                                 retreat_desired_distance=retreat_desired_distance,
                                 retreat_min_distance=retreat_min_distance,
                                 finger_joints=finger_joints,
                                 allowed_touch_objects=[object_name])
        # res = self.mv_handler.place(object_name, [location])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (0, -1.25, 0.4)

        self.mv_handler.set_current_move_group(self.mv_handler.current_move_group.parent, self.mv_handler.current_eef_default_pose) # tmp
        res = self.mv_handler.place(object_name, pose)
        return res        

    def detect(self):
        return self.gd_cli.detect()

    def select_arm(self, y):
        # 0: left, 1: right
        arm_index =  0 if y > 0 else 1
        print("y: {}, arm_index: {}".format(y, arm_index))
        if arm_index == 0:
            new_move_group = self.mv_handler.left_start_move_group
            new_eef_default_pose = self.mv_handler.left_eef_default_pose
        else:
            new_move_group = self.mv_handler.right_start_move_group
            new_eef_default_pose = self.mv_handler.right_eef_default_pose

        self.mv_handler.set_current_move_group(new_move_group, new_eef_default_pose)
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
    sensors = rospy.get_param("sensors", default=("body_camera", "left_camera", "right_camera"))
    raw_point_topics = ["/{}/{}/depth/color/points".format(ns, sensor_name) for sensor_name in sensors]

    wait = rospy.get_param("wait_server", default=True)
    use_constraint = rospy.get_param("use_constraint", default=False)
    rospy.loginfo("################################################")

    print("initializing instances...")
    myrobot = Myrobot(fps=fps, image_topic=image_topic, depth_topic=depth_topic, raw_point_topics=raw_point_topics, wait=wait, use_constraint=use_constraint)
    myrobot.info()
    myrobot.initialize_whole_pose()

    print("getting around octomap...")
    myrobot.get_around_octomap(values=[-30, 30, 0], sleep_time=0.3, is_degree=True, should_reset=True)

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
        # TODO: pull up arm index computation from pick
        res, arm_index = myrobot.pick(obj_name, obj, pre_move=False,
                     approach_desired_distance=insert_depth * 2,
                     retreat_desired_distance=insert_depth * 2,
                     approach_min_distance=insert_depth * 1.2,
                     retreat_min_distance= insert_depth * 1.2
        )
        print(res)
        # print("start place")
        # res = myrobot.place(arm_index, obj_name)

        print("will initialize")
        myrobot.initialize_current_pose(cartesian_mode=True)

        myrobot.scene_handler.remove_attached_object("")
        myrobot.scene_handler.remove_world_object(obj_name)

        myrobot.scene_handler.update_octomap()