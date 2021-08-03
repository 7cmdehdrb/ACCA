#!/usr/bin/env python

import rospy
import math as m
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Path


class GetPose(object):
    def __init__(self):
        super(GetPose, self).__init__()

        self.startTime = rospy.Time.now()

        self.xs = [0.0]
        self.ys = [0.0]

        self.Path = PoseArray()

        self.Path.header.seq = 0
        self.Path.header.frame_id = "map"

    def poseCallback(self, msg):
        temp_x = msg.pose.pose.position.x
        temp_y = msg.pose.pose.position.y

        self.xs.append(temp_x)
        self.ys.append(temp_y)

        print(self.xs)
        print(self.ys)

        print("")


if __name__ == "__main__":
    rospy.init_node("get_pose")

    pose = GetPose()

    path_pub = rospy.Publisher("test_path", PoseArray, queue_size=1)

    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, pose.poseCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        r.sleep()
