#!/usr/bin/env python2
import message_filters as mf
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from actionlib import SimpleActionClient
from detect.msg import GraspDetectionAction, GraspDetectionGoal

class GraspDetectionClient:
    def __init__(self, name, fps, image_topic, depth_topic):
        delay = 1 / fps * 0.5

        # Subscribers
        subscribers = [mf.Subscriber(topic, Image) for topic in (image_topic, depth_topic)]
        # Action Client
        self.cli = SimpleActionClient(name, GraspDetectionAction)
        # Others
        self.bridge = CvBridge()
        self.request = None

        self.ts = mf.ApproximateTimeSynchronizer(subscribers, 10, delay)
        self.ts.registerCallback(self.callback)

        self.cli.wait_for_server()
        
    def callback(self, img_msg, depth_msg):
        try:
            self.request = GraspDetectionGoal(img_msg, depth_msg)
        except Exception as err:
            rospy.logerr(err)

    def detect(self):
        last_added = self.ts.last_added
        # wait for getting new images
        while self.request is None or self.ts.last_added <= last_added:
            pass
        self.cli.send_goal_and_wait(self.request)
        res = self.cli.get_result().objects
        return res


if __name__ == "__main__":
    rospy.init_node("grasp_detection_client", log_level=rospy.INFO)

    fps = rospy.get_param("fps")
    image_topic = rospy.get_param("image_topic")
    depth_topic = rospy.get_param("depth_topic")

    cli = GraspDetectionClient(
        "grasp_detection_server",
        fps=fps,
        image_topic=image_topic,
        depth_topic=depth_topic,
    )

    rate = rospy.Rate(1)

    while not rospy.is_shutdown(): 
        cli.detect()
        rate.sleep()