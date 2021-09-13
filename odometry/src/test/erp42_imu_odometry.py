#!/usr/bin/env python

import sys
import rospy
import tf
import math as m
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
from odometry.msg import encoderMsg


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/hojin/catkin_ws/src")


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils/")
    from state import State
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()


class IMUOdometry(object):
    def __init__(self):

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.ind = 0
        self.ave_x = 0.0
        self.ave_y = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.init_yaw = 0.0

        self.dx = 0.0
        self.dy = 0.0
        self.dth = 0.0

        self.ax = 0.0
        self.ay = 0.0

        self.dt = 0.0

        self.encoderV = 0.0

        self.doTransform = False

    def encoder_callback(self, msg):

        self.encoderV = msg.speed

    def imu_callback(self, msg):
        data = msg

        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w

        quaternion_array = [x, y, z, w]

        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion_array)

        if yaw != 0.0 and self.init_yaw == 0.0:
            self.init_yaw = yaw
            print("IMU initialize success : %f" % self.init_yaw)
        else:
            self.yaw = yaw - self.init_yaw
            # self.yaw = self.normalize_angle(yaw)

        self.dth = data.angular_velocity.z
        # print(yaw)

    def acc_callback(self, msg):
        data = msg

        ax = data.vector.x
        ay = data.vector.y

<<<<<<< HEAD
=======
        if self.encoderV == 0.0:
            ax = 0.0
            ay = 0.0

>>>>>>> f6480b12743d080885afbb47fa1dc72a74e33e2f
        self.ax = ax
        self.ay = ay

        self.ind += 1

        self.ave_x = self.ave_x * \
            (float(self.ind - 1.0) / float(self.ind)) + \
            ax * (1.0 / float(self.ind))
        self.ave_y = self.ave_y * \
            (float(self.ind - 1.0) / float(self.ind)) + \
            ay * (1.0 / float(self.ind))

        print(self.ave_x)

        self.ax -= self.ave_x

        # print(self.ave_x, self.ave_y)

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def update(self):
        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

<<<<<<< HEAD
        # self.dx += self.ax * m.cos(self.yaw) * self.dt
        # self.dy += self.ax * m.sin(self.yaw) * self.dt

        # if abs(self.ax) < 0.1:
        #     self.ax = 0.0

        # if self.ay < 0.1:
        #     self.ay = 0.0
=======
        self.x += self.dx * self.dt + 0.5 * \
            m.cos(self.yaw) * self.ax * (self.dt ** 2)
        self.y += self.dy * self.dt + 0.5 * \
            m.sin(self.yaw) * self.ax * (self.dt ** 2)
>>>>>>> f6480b12743d080885afbb47fa1dc72a74e33e2f

        self.dx += self.ax * m.cos(self.yaw) * self.dt
        self.dy += self.ax * m.sin(self.yaw) * self.dt

<<<<<<< HEAD
        # print(self.dx, self.dy)

        self.x += self.dx * self.dt + 0.5 * \
            self.ax * m.cos(self.yaw) * (self.dt ** 2)
        self.y += self.dy * self.dt + 0.5 * \
            self.ax * m.sin(self.yaw) * (self.dt ** 2)

        # print(self.x, self.y)
=======
        print(self.x, self.y)
>>>>>>> f6480b12743d080885afbb47fa1dc72a74e33e2f


if __name__ == "__main__":
    rospy.init_node("erp42_imu_odometry")

    my_odom = IMUOdometry()

    odom_pub = rospy.Publisher("erp42_imu_odometry", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    """ Subscriber """
    # IMU subscribe
    rospy.Subscriber("/filter/free_acceleration",
                     Vector3Stamped, my_odom.acc_callback)
    rospy.Subscriber("/imu/data", Imu, my_odom.imu_callback)
    rospy.Subscriber("/erp42_encoder", encoderMsg, my_odom.encoder_callback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        """ Odometry """

        my_odom.update()

        odom_quat = tf.transformations.quaternion_from_euler(
            0.0, 0.0, my_odom.yaw)

        odom.header.stamp = rospy.Time.now()

        # ODOMETRY

        odom.pose.pose = Pose(
            Point(my_odom.x, my_odom.y, 0.0), Quaternion(*odom_quat))

        odom.twist.twist = Twist(Vector3(my_odom.dx, my_odom.dy, 0.0),
                                 Vector3(0.0, 0.0, my_odom.dth))

        odom_pub.publish(odom)

        # ODOMETRY END

        if my_odom.doTransform is True:
            odom_broadcaster.sendTransform(
                (my_odom.x, my_odom.y, 0.0),
                odom_quat,
                my_odom.currentTime,
                "base_link",
                "odom"
            )

        my_odom.lastTime = rospy.Time.now()

        r.sleep()
