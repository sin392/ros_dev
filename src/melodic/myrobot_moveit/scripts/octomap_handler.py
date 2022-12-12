#!/usr/bin/env python2
# coding: UTF-8

import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

# to achive octomap updating by custom timing
class OctomapHandler:
    def __init__(self, from_topics):
        self.from_topics = from_topics
        self.to_topics = [from_topic.replace("points", "republished_points") for from_topic in self.from_topics]
        self.publishers = [rospy.Publisher(to_topic, PointCloud2, queue_size=5) for to_topic in self.to_topics]

        rospy.wait_for_service('/clear_octomap')
        self.clear_octomap_proxy = rospy.ServiceProxy('/clear_octomap', Empty)

    def clear(self):
        self.clear_octomap_proxy()

    def update(self):
        for from_topic, publisher in zip(self.from_topics, self.publishers):
            msg = rospy.wait_for_message(from_topic, PointCloud2)
            publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node("octomap_handler")

    fps = rospy.get_param("fps", default=1)
    ns = rospy.get_param("robot_name", default="myrobot")
    sensors = rospy.get_param("~sensors", default=("left_camera", "right_camera", "body_camera"))

    from_topics = ["/{}/{}/depth/color/points".format(ns, sensor_name) for sensor_name in sensors]

    oh = OctomapHandler(from_topics)

    rate = rospy.Rate(fps)

    while not rospy.is_shutdown():
        oh.update()
        rate.sleep()
    