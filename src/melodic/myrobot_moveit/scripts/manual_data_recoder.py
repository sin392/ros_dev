#!/usr/bin/env python2
# coding: UTF-8
import pickle
import os
from datetime import datetime
import numpy as np
import rospy
import message_filters as mf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from modules.const import WHOLE_OUTPUTS_PATH
from modules.ros.utils import multiarray2numpy
from detect.msg import InstanceSegmentationActionResult, GraspDetectionActionResult, \
    InstanceSegmentationResult, GraspDetectionResult, GraspDetectionDebugInfo

class Client(object):
    def __init__(self, image_topic, depth_topic, instances_topic, objects_topic):
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        # Subscribers
        self.is_subscriber = rospy.Subscriber(instances_topic, InstanceSegmentationActionResult, self.is_callback)
        self.gd_subscriber = rospy.Subscriber(objects_topic, GraspDetectionActionResult, self.gd_callback)
        # Publishers
        self.is_publisher = rospy.Publisher("/is_result", InstanceSegmentationResult, queue_size=10)
        self.gd_publisher = rospy.Publisher("/gd_result", GraspDetectionResult, queue_size=10)
        # Filters
        subscribers = [
            # mf.Subscriber(image_topic, Image), 
            # mf.Subscriber(depth_topic, Image),
            mf.Subscriber("/is_result", InstanceSegmentationResult), 
            mf.Subscriber("/gd_result", GraspDetectionResult),
            mf.Subscriber("{}/debug".format(objects_topic), GraspDetectionDebugInfo),
        ]
        # Others
        self.bridge = CvBridge()

        self.is_empty = True
        self.ts = mf.TimeSynchronizer(subscribers, 10)
        self.ts.registerCallback(self.ts_callback)

        print("waiting ...")
        while self.is_empty:
            pass
           

    def is_callback(self, msg):
        self.is_publisher.publish(msg.result)

    def gd_callback(self, msg):
        self.gd_publisher.publish(msg.result)

    def ts_callback(self, 
    # img_msg, depth_msg, 
    instances_goal_msg, objects_goal_msg, objects_debug_msg):
        try:
            self.img_msg = rospy.wait_for_message(self.image_topic, Image)
            self.depth_msg = rospy.wait_for_message(self.depth_topic, Image)
            self.instances_msg = instances_goal_msg
            self.objects_msg = objects_goal_msg
            self.objects_debug_msg = objects_debug_msg

            self.is_empty = False
        except Exception as err:
            rospy.logerr(err)

    def get_data(self):
        img = self.bridge.imgmsg_to_cv2(self.img_msg)
        depth = self.bridge.imgmsg_to_cv2(self.depth_msg)
        objects = []
        for instance_msg, candidates_msg in zip(self.instances_msg.instances, self.objects_debug_msg.candidates_list):
            mask = self.bridge.imgmsg_to_cv2(instance_msg.mask)
            contour = multiarray2numpy(int, np.int32, instance_msg.contour)
            center = instance_msg.center
            candidates = []
            for candidate_msg in candidates_msg.candidates:
                candidate = [point.uv for point in candidate_msg.points]
                candidates.append(candidate)
            objects.append({"mask": mask, "contour": contour, "center": center, "candidates": candidates})

        return {"img": img, "depth": depth, "objects": objects}

if __name__ == "__main__":
    rospy.init_node("manual_data_recorder")

    now = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    output_dir = "{}/saved_data/{}".format(WHOLE_OUTPUTS_PATH, now)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    image_topic = rospy.get_param("image_topic")
    depth_topic = rospy.get_param("depth_topic")
    debug = rospy.get_param("debug")

    if not debug:
        print("Please set 'debug:=false' for bringup.launch in 'detect'")
        exit()

    instances_topic = "/instance_segmentation_server/result"
    objects_topic = "/grasp_detection_server/result"

    cli = Client(
        image_topic=image_topic,
        depth_topic=depth_topic,
        instances_topic=instances_topic,
        objects_topic=objects_topic,
    )
    rospy.sleep(1)

    print("Please press key!! (only Enter: save current data, q: quit)")
    count = 1
    while not rospy.is_shutdown():
        char = raw_input() # python2's input()
        if char == "": # Enter
            res = cli.get_data()            
            filename = "data_{}.pkl".format(count)
            save_path= "{}/{}".format(output_dir, filename)
            with open(save_path, mode="wb") as f:
                pickle.dump(res, f)
            print("current data is saved to '{}'".format(filename))

            count += 1

        elif char == "q":
            break