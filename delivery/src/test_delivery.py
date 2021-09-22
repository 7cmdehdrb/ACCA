#!/usr/bin/env python

import sys
import rospy
import tf
import math as m
import time
from std_msgs.msg import UInt8
import threading


msg = UInt8()


def inputData():
    global msg

    while not rospy.is_shutdown():
        data = input("0 or 1: ")

        msg.data = int(data)


if __name__ == "__main__":
    rospy.init_node("delevery_test")

    th = threading.Thread(target=inputData)
    th.start()

    pub = rospy.Publisher("/sign_pub", UInt8, queue_size=1)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()
