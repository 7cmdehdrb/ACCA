#!/usr/bin/env python

import rospy
import math as m
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odometry.msg import encoderMsg


wheelbase = 1.040
wheeltrack = 0.985
TICK2RAD = 0.06283185307
wheel_radius = 0.265


R_err_cal = 0.8254
L_err_cal = 0.7202


class my_odometry(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(my_odometry, self).__init__()

        self.encoder_msg = encoderMsg()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.initial_enc = 0
        self.last_encoder = 0

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def encoder_callback(self, msg):
        self.encoder_msg = msg

    def publishOdometry(self, publisher, broadcaster):
        self.current_time = rospy.Time.now()

        dt = (self.current_time - self.last_time).to_sec()

        delta_encoder = self.encoder_msg.Encoder - self.last_encoder
        self.last_encoder = self.encoder_msg.Encoder + self.initial_enc

        wheel_pos = TICK2RAD * delta_encoder
        delta_pos = wheel_radius * wheel_pos

        linear_vel = delta_pos / dt
        self.v = linear_vel

        # transformation from FL to base_link
        DEG = (self.encoder_msg.steer)

        if (DEG) != 0:
            A = wheelbase / \
                m.tan(m.radians(abs(DEG))) - wheeltrack / 2
            th1 = m.atan(wheelbase / (wheeltrack + A))
            th2 = m.atan(wheelbase / A)
            r1 = wheelbase / m.sin(th1)
            r2 = wheelbase / m.sin(th2)
            rb = wheelbase / m.tan(m.radians(abs(DEG)))
            if DEG < 0:
                delta_pos *= rb / r1
                delta_pos *= R_err_cal
                linear_vel *= rb / r1
                linear_vel *= R_err_cal
            else:
                delta_pos *= rb / r2
                delta_pos *= L_err_cal
                linear_vel *= rb / r2
                linear_vel *= L_err_cal
            self.v = linear_vel

        angular_vel = m.tan(m.radians(DEG)) * \
            linear_vel / wheelbase

        self.yaw += angular_vel * dt
        self.x += delta_pos * m.cos(self.yaw)
        self.y += delta_pos * m.sin(self.yaw)
        vx = linear_vel * m.cos(self.yaw)
        vy = linear_vel * m.sin(self.yaw)

        odom_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.yaw)

        broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.),
                              Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            Vector3(vx, vy, 0), Vector3(0, 0, angular_vel))

        self.last_time = rospy.Time.now()
        publisher.publish(odom)


if __name__ == "__main__":
    rospy.init_node("erp42_odometry")

    my_odom = my_odometry(x=0.0, y=0.0, yaw=0.0, v=0.0)

    odom_pub = rospy.Publisher("erp42_odometry", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    """ Subscriber """
    # Encoder subscibe
    rospy.Subscriber("/erp42_encoder",
                     encoderMsg, my_odom.encoder_callback)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        """ Odometry """

        my_odom.publishOdometry(
            publisher=odom_pub, broadcaster=odom_broadcaster)

        r.sleep()
