#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped
import gym_duckietown
from gym_duckietown.simulator import Simulator


class RosWrapperSimNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RosWrapperSimNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # get vehicle name
        vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        rospy.loginfo("Vehicle name is %s" % vehicle_name)

        # construct image publisher
        image_topic = "/%s/camera_node/image/compressed" % vehicle_name
        self.image_pub = rospy.Publisher(image_topic, CompressedImage, queue_size=10)
        rospy.loginfo("Publisher set to %s" % image_topic)
        # construct camera_info publisher
        camera_info_topic = "/%s/camera_node/camera_info" % vehicle_name
        self.info_pub = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=10)
        rospy.loginfo("Publisher set to %s" % camera_info_topic)
        

        # construct subscriber
        wheels_command_topic = "/%s/wheels_driver_node/wheels_cmd" % vehicle_name
        self.wheels_command_sub = rospy.Subscriber(wheels_command_topic, WheelsCmdStamped, self.wheel_commands_callback)
        rospy.loginfo("Subscribed to %s" % wheels_command_topic)

        # initialise action
        self.action = [0, 0]

    def wheel_commands_callback(self, data):
        # function gets called when sub receives a message containing 'data'
        rospy.loginfo("Received message:\n%s", data)
        self.action = [data.vel_left, data.vel_right]

    def publish_image(self, observation):
        # to do!
        pass

    def run(self):
        # publish image 16 times per second
        rate = rospy.Rate(16) 

        while not rospy.is_shutdown():
            observation, reward, done, _ = env.step(self.action)
            env.render()
            if done:
                observation = env.reset()

            # publish image to image topic
            image_msg = CompressedImage()
            image_msg.header.stamp = rospy.Time.now()
            image_msg.format = "jpeg"
            image = cv2.cvtColor(np.ascontiguousarray(observation), cv2.COLOR_BGR2RGB)
            image_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
            self.image_pub.publish(image_msg)

            # publish camera info to camera info topic
            self.info_pub.publish(CameraInfo())

            rate.sleep()


if __name__ == '__main__':
    # Initialise the simulator
    rospy.loginfo("Initialising simulator ...")
    env = Simulator(
        seed=123, # random seed
        map_name="loop_empty",
        max_steps=500001, # we don't want the gym to reset itself
        domain_rand=0,
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4, # start close to straight
        full_transparency=True,
        distortion=True,
    )
    observation = env.reset()
    rospy.loginfo("Simulator initialised")

    # create the node
    rospy.loginfo("Initialising node ...")
    node = RosWrapperSimNode(node_name='ros_wrapper_sim_node')
    rospy.loginfo("Node initialised")

    # run node
    node.run()