#!/usr/bin/env python

import rospy
import rospkg
import threading
import tf
import csv
import math as m
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path


class PathSubscriber(object):
    def __init__(self):
        super(PathSubscriber, self).__init__()
        self.msg = Path()

    def pathCallback(self, msg):
        self.msg = msg

        poses = (self.msg.poses)

        print(type(poses[0]))


if __name__ == "__main__":
    rospy.init_node("path_subscriber")

    path_sub = PathSubscriber()

    rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan",
                     Path, path_sub.pathCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        r.sleep()
