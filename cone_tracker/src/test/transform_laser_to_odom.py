#!/usr/bin/env python

import rospy
import sys
import os
import tf
import math as m
import numpy as np
from geometry_msgs.msg import PoseArray, Pose

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from transform_node import TransformNode
except Exception as ex:
    print(ex)


class TransformLaser(TransformNode):
    def __init__(self, parent="odom", child="laser"):
        super(TransformLaser, self).__init__(parent=parent, child=child)

        self.subscribedPoseArray = PoseArray()

    def poseArrayCallback(self, msg):
        self.subscribedPoseArray = msg

    def transformPoseArray(self, poseArray):
        poses = self.subscribedPoseArray.poses

        temp_poses = []

        for pose in poses:
            p = Pose()

            p.position.x = pose.position.x + self.trans[0]
            p.position.y = pose.position.x + self.trans[1]
            p.position.z = 0.0

            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0

            temp_poses.append(p)

        return temp_poses

    def publishPoseArray(self, publisher):
        new_poses = self.transformPoseArray()

        newPoseArray = PoseArray()

        newPoseArray.header.stamp = rospy.Time.now()
        newPoseArray.header.frame_id = "odom"
        newPoseArray.poses = new_poses

        publisher.publish(newPoseArray)


if __name__ == "__main__":
    rospy.init_node("transform_laser_to_odom")

    transformNode = TransformLaser()

    rospy.Subscriber("obstacles", PoseArray,
                     callback=transformNode.poseArrayCallback)

    poseArrayPub = rospy.Publisher(
        "/transformed_obstacles", PoseArray, queue_size=1)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        transform_node.listenTF()
        transformNode.publishPoseArray(publisher=poseArrayPub)
        r.sleep()
