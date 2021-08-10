#!/usr/bin/env python

import rospy
import tf
from dynamic_reconfigure.server import Server
# import threading
# import serial
# from multiprocessing import Queue
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from erp42_core.msg import DriveCmd
from erp42_core.msg import SerialFeedBack
import math
from math import sin, cos, pi, degrees, atan, tan, radians, sqrt
from path_planner.msg import stanleyMsg

# import time

wheelbase = 1.040
wheeltrack = 0.985
TICK2RAD = 0.06283185307
wheel_radius = 0.265
# static constexpr int8_t MAX_KPH = 5;
#     static constexpr int8_t MAX_DEGREE = 20;

#     // 3.6[deg] * 3.14159265359 / 180 = 0.06283185307
#     TICK2RAD = 0.06283185307;
#  m_wheel_radius = 0.265;
#         double m_wheel_tread = 0.985;

#     double wheel_radius {0.265};
#     double wheel_base {1.040};
#     double wheel_tread {0.985};
#     double max_vel {5.0};
#     double min_vel {-5.0};
#     dmax_steer_angle {28.169};
#     double min_steer_angle {-28.169};


class control():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_encoder = 0
        self.odom_yaw = 0.0
        self.command_msg = DriveCmd()
        self.cmd_vel_msg = Twist()
        self.feedback_msg = SerialFeedBack()
        self.linear_vel = 0.0
        self.cmd_vel_msg.linear.x = 0.0
        self.vel_last_time = rospy.Time.now()
        self.odom_last_time = rospy.Time.now()
        self.cumError = 0.0
        self.initial_enc = 0

        self.stanley_msg = stanleyMsg()

    def vel_pid(self):
        current_time = rospy.Time.now()
        elapsed_time = current_time - self.vel_last_time

        # initialize parameters at manual mode
        if self.feedback_msg.AorM != 1:
            self.cumError = 0
            self.vel_last_time = current_time
            self.command_msg.KPH = 0
            return

        error = self.cmd_vel_msg.linear.x - self.linear_vel

        # deceleration condition
        if (error < BrakeThreshold and self.linear_vel > 0) or\
           (error > -BrakeThreshold and self.linear_vel < 0) or\
           self.cmd_vel_msg.linear.x == 0:
            self.command_msg.Brake = 60
        else:
            self.command_msg.Brake = 1

        self.cumError += error * elapsed_time.to_sec()
        # rateError = (error - lastError)/elapsed_time

        # self.command_msg.KPH += kp*error + ki*cumError + kd*rateError
        self.command_msg.KPH = (
            self.cmd_vel_msg.linear.x + kp*error + ki*self.cumError) * 3600/1000

        print(self.command_msg.KPH)

        # lastError = error
        self.vel_last_time = current_time

    def cmd_vel_to_command(self):
        if self.cmd_vel_msg.linear.x > 0:
            self.command_msg.Gear = 2
        elif self.cmd_vel_msg.linear.x < 0:
            self.command_msg.Gear = 0
        elif self.cmd_vel_msg.linear.x == 0:
            self.command_msg.Gear = 1

        if self.cmd_vel_msg.linear.x != 0 and self.linear_vel != 0:
            self.command_msg.Deg = (-degrees(atan(wheelbase /
                                    self.linear_vel * self.cmd_vel_msg.angular.z)))
            self.command_msg.Deg += SteerOffset
        else:
            self.command_msg.Deg = 0

    def calc_vel_and_pub_odom(self):
        # # to save the orgin of odom frame
        # if self.feedback_msg.Encoder != 0 and self.last_encoder == 0:
        #     self.initial_enc = self.feedback_msg.Encoder
        #     self.last_encoder = self.feedback_msg.Encoder

        # # sometimes, prior feedback msgs received
        # if self.feedback_msg.Encoder == self.last_encoder and self.linear_vel != 0:
        #     self.linear_vel = 0
        #     return

        # calculate linear velocity with incremental encoder
        current_time = rospy.Time.now()
        dt = (current_time - self.odom_last_time).to_sec()
        delta_encoder = self.feedback_msg.Encoder - self.last_encoder
        self.last_encoder = self.feedback_msg.Encoder + self.initial_enc

        wheel_pos = TICK2RAD * delta_encoder
        delta_pos = wheel_radius * wheel_pos

        linear_vel = delta_pos / dt
        self.linear_vel = linear_vel

        # transformation from FL to base_link
        if self.feedback_msg.Deg != 0:
            A = wheelbase / \
                tan(radians(abs(self.feedback_msg.Deg))) - wheeltrack / 2
            th1 = atan(wheelbase / (wheeltrack + A))
            th2 = atan(wheelbase / A)
            r1 = wheelbase / sin(th1)
            r2 = wheelbase / sin(th2)
            rb = wheelbase / tan(radians(abs(self.feedback_msg.Deg)))
            if self.feedback_msg.Deg < 0:
                delta_pos *= rb / r1
                delta_pos *= R_err_cal
                linear_vel *= rb / r1
                linear_vel *= R_err_cal
            else:
                delta_pos *= rb / r2
                delta_pos *= L_err_cal
                linear_vel *= rb / r2
                linear_vel *= L_err_cal
            self.linear_vel = linear_vel

        angular_vel = tan(radians(self.feedback_msg.Deg)) * \
            linear_vel / wheelbase

        self.odom_yaw += angular_vel * dt
        self.x += delta_pos * cos(self.odom_yaw)
        self.y += delta_pos * sin(self.odom_yaw)
        vx = linear_vel * cos(self.odom_yaw)
        vy = linear_vel * sin(self.odom_yaw)

        self.odom_last_time = current_time

        odom_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.odom_yaw)

        odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.),
                              Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            Vector3(vx, vy, 0), Vector3(0, 0, angular_vel))

        odom_pub.publish(odom)
        vel_pub.publish(self.linear_vel)

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg
        self.cmd_vel_to_command()

    def feedback_callback(self, msg):
        self.feedback_msg = msg
        self.calc_vel_and_pub_odom()
        self.vel_pid()

    def stanleyCallback(self, msg):
        self.stanley_msg = msg


if __name__ == "__main__":
    # initialize node
    rospy.init_node('erp42_driver')

    # get parameters from rosparam server
    kp = rospy.get_param('~p_gain', 3)
    ki = rospy.get_param('~i_gain', 1)
    kd = rospy.get_param('~d_gain', 1)
    R_err_cal = rospy.get_param('~RightSteerErrCal', 0.8254)
    L_err_cal = rospy.get_param('~LeftSteerErrCal', 0.7202)
    SteerOffset = rospy.get_param('/SteerOffset', 0)
    BrakeThreshold = rospy.get_param('~BrakeThresholdErr', -1)
    BrakeLimit = rospy.get_param('~BrakeLimit', 100)

    driver = control()

    rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist,
                     driver.cmd_vel_callback)
    rospy.Subscriber('/erp42_feedback', SerialFeedBack,
                     driver.feedback_callback)
    command_pub = rospy.Publisher('/erp42_command', DriveCmd, queue_size=1)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
    vel_pub = rospy.Publisher("/erp42_feedback_vel", Float32, queue_size=1)

    odom_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        command_pub.publish(driver.command_msg)
        rate.sleep()
