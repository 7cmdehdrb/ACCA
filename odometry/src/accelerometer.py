#!/usr/bin/env python


import sys
import rospy
import numpy as np
import math as m
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


class Accelerometer(object):
    def __init__(self):

        self.yaw = 0.0

        self.acc = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.dt = (self.current_time - self.last_time).to_sec()

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        PoseWithCovarianceStamped().pose.pose.

    def accelCallback(self, msg):
        self.acc = msg.vector.x

    def imuCallback(self, msg):
        msg = Imu()

        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        _, _, yaw = tf.transformations.euler_from_quaternion([x, y, z, w])

        self.yaw = yaw

    def update(self):
        self.current_time = rospy.Time.now()

        self.dt = (self.current_time - self.last_time).to_sec()

    def publishOdometry(self):
        self.odom.header.stamp = rospy.Time.now()

        odom_quat = [0.0, 0.0, 0.0, 1.0]

        self.odom.pose.pose = Pose(
            Point(0.0, 0.0, 0.0), Quaternion(*odom_quat)
        )

        self.odom.twist.twist = Twist(
            Vector3()
        )
