#!/usr/bin/env python

import sys
import rospy
import tf
import math as m
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")


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

        self.ind = 0.0

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

        self.doTransform = False

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

    def acc_callback(self, msg):
        data = msg
        # ax = data.vector.x[i]
        # ay = data.vector.y[i]

        i = 0  # indedx
        while (i > 1):
            sum = 0.0
            for i in range(len(data.vector.x)):
                sum = sum + data.vector.x[i]
            ax = sum / len(data.vector.x)

            if len(i) > 30:
                self.ind = i
            i += 1
        return self.ind

        while (i > 1):
            sum = 0.0
            for i in range(len(data.vector.y)):
                sum = sum + data.vector.y[i]
            ay = sum / len(data.vector.y)

            if len(i) > 30:
                self.ind = i
            i += 1
        return self.ind

        self.ax = ax
        self.ay = ay

        # if abs(ax) < 0.05:
        #     ax = 0.0

        # if abs(ay) < 0.05:
        #     ay = 0.0

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

        # self.dx += self.ax * m.cos(self.yaw) * self.dt
        # self.dy += self.ax * m.sin(self.yaw) * self.dt

        if self.ax < 0.1:
            self.ax = 0.0
        if self.ay < 0.1:
            self.ay = 0.0

        self.dy *= self.ax * self.dt
        self.dx *= self.ax * self.dt

        self.x += self.dx*self.dt + 0.5*self.ax*(self.dt**2)
        self.y += self.dy*self.dt + 0.5*self.ay*(self.dt**2)

        print(self.x, self.y)


if __name__ == "__main__":
    rospy.init_node("erp42_imu_odometry")

    my_odom = IMUOdometry()

    odom_pub = rospy.Publisher("erp42_imu_odometry", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    odom = Odometry()
    odom.header.frame_id = "map"
    odom.child_frame_id = "base_link"

    """ Subscriber """
    # IMU subscribe
    rospy.Subscriber("/filter/free_acceleration",
                     Vector3Stamped, my_odom.acc_callback)
    rospy.Subscriber("/imu/data", Imu, my_odom.imu_callback)

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
