#!/usr/bin/env python


import sys
import tf
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from vehicle_msgs.msg import TrackCone, Track

class Obstacles(object):
    def __init__(self, x, y,z):

        self.x = x
        self.y = y
        self.z = z


class DongjinTest(object):
    def __init__(self):
        super(DongjinTest, self).__init__()

        self.obstacles = []

    def obstaclesCallback(self, msg):
        temp = []

        poses = msg.poses

        for pose in poses:
            if pose.position.x >0.0:

                new_obstacle = PoseStamped()

                new_obstacle.header.frame_id = "velodyne"
                new_obstacle.header.stamp = rospy.Time(0)

                new_obstacle.pose = pose

                new_obstacle = tf_node.transformPose("odom", new_obstacle)

                new_obstacle = Obstacles(
                    x=new_obstacle.pose.position.x, y=new_obstacle.pose.position.y, z=0)

                temp.append(new_obstacle)

            self.obstacles = temp
            print(len(self.obstacles))


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

    rospy.Subscriber("adaptive_clustering/poses", PoseArray, dj.obstaclesCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        dj.publishObstacles(publisher=ob_pub)
        r.sleep()
        