#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher("rvo_input", Float32MultiArray, queue_size=10)
rospy.init_node('main_test')
rate = rospy.Rate(1)

msg = Float32MultiArray()

while not rospy.is_shutdown():

    msg.data = [0, 0, 0, 0, 0, 0, 1, 1, 1]
    msg.data += [5, 5, 5, 0, 0, 0, -1, -1, -1]
    pub.publish(msg)
    rate.sleep();
