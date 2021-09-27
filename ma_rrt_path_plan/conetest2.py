#!/usr/bin/env python


import sys
import tf
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from vehicle_msgs.msg import TrackCone, Track

class Obstacles(object):
    def __init__(self, x, y):

        self.x = x
        self.y = y


class DongjinTest(object):
    def __init__(self):
        super(DongjinTest, self).__init__()

        self.obstacles = []

    def obstaclesCallback(self, msg):
        temp = []

        poses = msg.poses

        try:
            for pose in poses:
                new_obstacle = PoseStamped()

                new_obstacle.header.frame_id = "laser"
                new_obstacle.header.stamp = rospy.Time(0)

                new_obstacle.pose = pose

                new_obstacle = tf_node.transformPose("odom", new_obstacle)

                new_obstacle = Obstacles(
                    x=new_obstacle.pose.position.x, y=new_obstacle.pose.position.y)

                temp.append(new_obstacle)

            self.obstacles = temp

        except Exception as ex:
            pass

    def publishObstacles(self, publisher):

        msg = Track()
        msg.cones = []

        for obstacle in self.obstacles:

            conexy = TrackCone()

            conexy.x = obstacle.x
            conexy.y = obstacle.y

            msg.cones.append(conexy)

        publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("dongjin_test")

    dj = DongjinTest()

    tf_node = tf.TransformListener()

    ob_pub = rospy.Publisher("/track", Track, queue_size=1)

    rospy.Subscriber("/cone_position", PoseArray, dj.obstaclesCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        dj.publishObstacles(publisher=ob_pub)
        r.sleep()