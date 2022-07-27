#!/usr/bin/env python2
# coding: UTF-8

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates

    

def main():
    rospy.init_node("spawn_cabbags_node")

    # # delete exsited models
    # delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    # # get message once | ref: https://qiita.com/yudaikubota-me/items/dd420b92c9cdb5019c05
    # models = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None).name
    # for model in models:
    #     if model.startswith("cabbage"): delete_model_client(model)

    last_model = [model for model in rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None).name if model.startswith("cabbage")][-1]
    sub_length = len("cabbage_")
    last_number = int(last_model[sub_length:]) + 1
    # return 

    # spawn models
    spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    model_xml = open('myrobot_description/sdf/cabbage/model.sdf', 'r').read()
    initial_pose = Pose()
    initial_pose.position.z = 0.8
    x_y_pairs = [(0, 0), (0.2, 0.2), (-0.2, 0.2), (0.2, -0.2), (-0.2, -0.2)]
    n = len(x_y_pairs)

    for i in range(10):
        x, y = x_y_pairs[int(i % n)]
        initial_pose.position.x = x
        initial_pose.position.y = y

        spawn_model_client(
            model_name='cabbage_{}'.format(last_number + i),
            model_xml=model_xml,
            initial_pose=initial_pose,
            reference_frame='world'
        )
        # 大量のモデルが一度に出現して物理演算が重くなるのをさけるため
        if i % n == 0:
            rospy.sleep(0.5)



if __name__ == "__main__":
    main()