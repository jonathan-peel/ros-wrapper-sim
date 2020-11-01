#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class RosWrapperSimNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RosWrapperSimNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher as a object contained in self
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        # construct subscriber
        self.sub = rospy.Subscriber('chatter', String, self.callback)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            message = "Hello World!"
            rospy.loginfo("Publishing message: '%s'" % message)
            self.pub.publish(message)
            rate.sleep()

    def callback(self, data):
        rospy.loginfo("Received message %s", data.data)

if __name__ == '__main__':
    # create the node
    node = RosWrapperSimNode(node_name='ros_wrapper_sim_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()