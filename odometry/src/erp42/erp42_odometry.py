#!/usr/bin/env python

import rospy
import math as m
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class my_odometry():
    def __init__(self):

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.x = 0.0
        self.y = 0.0

        self.v = 0.0
        self.th = 0.0

        self.init_yaw = 0.0

        self.dx = 0.0
        self.dy = 0.0
        self.dth = 0.0

        self.dt = 0.0

    def encoder_callback(self, msg):
        """
            data = [self.feedback_AorM, self.feedback_gear,
                    self.feedback_speed, self.feedback_steer, self.feedback_brake]
        """

        data = msg

        gear = int(data.data[1])    # 2: D / 1: N / 0: B
        gear = 1.0 if gear == 2 else -1.0

        self.v = data.data[2] * gear  # mps

    def imu_callback(self, msg):
        data = msg

        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w

        quaternion_array = [x, y, z, w]

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion_array)

        if yaw != 0.0 and self.init_yaw == 0.0:
            self.init_yaw = yaw
            print("IMU initialize success : %f" % self.init_yaw)
        else:
            self.th = yaw - self.init_yaw

        self.dth = data.angular_velocity.z

    def update(self):
        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        self.dx = self.v * m.cos(self.th)
        self.dy = self.v * m.sin(self.th)

        self.x += self.dx * self.dt
        self.y += self.dy * self.dt

        self.lastTime = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("erp42_odometry")

    my_odom = my_odometry()

    odom_pub = rospy.Publisher("erp42_odometry", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()
    odom = Odometry()

    """ Subscriber """
    # IMU subscribe
    rospy.Subscriber("/imu/data", Imu, my_odom.imu_callback)

    # Encoder subscibe
    # rospy.Subscriber("/erp42/erp42_encoder",
    #                  Float32MultiArray, my_odom.encoder_callback)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():

        """ Odometry """

        my_odom.update()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, my_odom.th)

        odom_broadcaster.sendTransform(
            (my_odom.x, my_odom.y, 0.0),
            odom_quat,
            my_odom.currentTime,
            "base_link",
            "odom"
        )

        odom.header.stamp = my_odom.currentTime
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position information

        odom.pose.pose = Pose(
            Point(my_odom.x, my_odom.y, 0.0), Quaternion(*odom_quat))

        odom.twist.twist = Twist(Vector3(my_odom.dx, my_odom.dy, 0.0),
                                 Vector3(0.0, 0.0, my_odom.dth))

        odom_pub.publish(odom)

        my_odom.lastTime = rospy.Time.now()
