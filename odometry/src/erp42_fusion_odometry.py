#!/usr/bin/env python

import sys
import rospy
import numpy as np
import tf
import math as m
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
from odometry.msg import encoderMsg


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils/")
    from state import State
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()


current_time = None
last_time = None
doTransform = True


class FusionState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(FusionState, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.init_yaw = 0.0

        # Subscriber

        rospy.Subscriber("/erp42_encoder", encoderMsg,
                         callback=self.encoderCallback)
        rospy.Subscriber("/imu/data", Imu, callback=self.imuCallback)

        # msg

        self.encoderMsg = encoderMsg()
        self.imuMsg = Imu()

    def imuCallback(self, msg):
        self.imuMsg = msg

        if self.init_yaw == 0.0:

            quat = [
                self.imuMsg.orientation.x,
                self.imuMsg.orientation.y,
                self.imuMsg.orientation.z,
                self.imuMsg.orientation.w,
            ]

            _, _, YAW = tf.transformations.euler_from_quaternion(quat)
            self.init_yaw = YAW

    def encoderCallback(self, msg):
        self.encoderMsg = msg

    def handleData(self):

        vel = self.encoderMsg.speed  # m/s

        x = self.imuMsg.orientation.x
        y = self.imuMsg.orientation.y
        z = self.imuMsg.orientation.z
        w = self.imuMsg.orientation.w

        quat = [x, y, z, w]

        return vel, quat

    def update(self):
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()

        vel, quat = self.handleData()
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        yaw = yaw - self.init_yaw

        self.v = vel
        self.dx = vel * m.cos(yaw)
        self.dy = vel * m.sin(yaw)
        self.dyaw = yaw - self.last_yaw

        self.x += self.dx * dt
        self.y += self.dy * dt
        self.yaw = yaw

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw

        # print(self.x, self.y)


if __name__ == "__main__":
    rospy.init_node("erp42_fusion_odometry")

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    state = FusionState(x=0.0, y=0.0, yaw=0.0, v=0.0)

    odom_pub = rospy.Publisher(
        "/erp42_fusion_odometry", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    odom = Odometry()

    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        state.update()

        odom.header.stamp = rospy.Time.now()

        odom.pose.pose = Pose(
            Point(state.x, state.y, 0.0), Quaternion(*state.quat)
        )

        odom.twist.twist = Twist(
            Vector3(state.dx, state.dt, 0.0),
            Vector3(0.0, 0.0, state.dyaw)
        )

        if doTransform is True:
            odom_broadcaster.sendTransform(
                (state.x, state.y, 0.0),
                state.quat,
                current_time,
                "base_link",
                "odom"
            )

        odom_pub.publish(odom)

        last_time = rospy.Time.now()

        r.sleep()
