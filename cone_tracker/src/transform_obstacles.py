#!/usr/bin/env python


import sys
import tf
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray, Pose, PoseStamped


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from util_class import Obstacle
except Exception as ex:
    print("UTIL CLASS IMPORT ERROR")
    print(ex)


class TransformObstacles(object):
    def __init__(self):
        super(TransformObstacles, self).__init__()

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

                new_obstacle = Obstacle(
                    x=new_obstacle.pose.position.x, y=new_obstacle.pose.position.y)

                temp.append(new_obstacle)

            self.obstacles = temp

        except Exception as ex:
            pass

    def publishObstacles(self, publisher):
        msg = PoseArray()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"

        msg.poses = []

        for obstacle in self.obstacles:
            pose = Pose()

            pose.position.x = obstacle.x
            pose.position.y = obstacle.y
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            msg.poses.append(pose)

        publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("transform_obstacles")

    tf_ob = TransformObstacles()

    tf_node = tf.TransformListener()

    ob_pub = rospy.Publisher("/track", PoseArray, queue_size=1)

    rospy.Subscriber("/obstacles", PoseArray, tf_ob.obstaclesCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        tf_ob.publishObstacles(publisher=ob_pub)
        r.sleep()
