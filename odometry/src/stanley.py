#!/usr/bin/env python

"""
Path tracking simulation with Stanley steering control and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
"""
import rospy
import math as m
import numpy as np
import tf
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import cubic_spline_planner


k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
L = 1.040  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle


class PathFinder(object):
    def __init__(self):
        super(PathFinder, self).__init__()
        self.counter = True
        self.xs = [0.0, 100.0]
        self.ys = [0.0, 0.0]

    def pathCallback(self, msg):
        poses = msg.poses

        if self.counter is True:
            # print(msg.header)

            self.xs = [0.0]
            self.ys = [0.0]

            for pose in poses:
                print(pose)
                temp_x = pose.pose.position.x
                temp_y = pose.pose.position.y

                temp_quat_x = pose.pose.orientation.x
                temp_quat_y = pose.pose.orientation.y
                temp_quat_z = pose.pose.orientation.z
                temp_quat_w = pose.pose.orientation.w

                self.xs.append(temp_x)
                self.ys.append(temp_y)
                self.counter = False
                break

            self.counter = False


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)

    def odometryCallback(self, msg):
        data = msg

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        quaternion_array = [x, y, z, w]

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion_array)

        self.yaw = yaw

        self.dx = data.twist.twist.linear.x
        self.dy = data.twist.twist.linear.y
        self.v = m.sqrt(self.dx ** 2 + self.dy ** 2)


def pid_control(target, current):
    """
    Proportional control for the speed.
    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
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


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -
                      np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


if __name__ == "__main__":
    rospy.init_node("stanley_method")

    # Initial state
    state = State(x=-0.0, y=0, yaw=np.radians(0.0), v=0.0)
    path = PathFinder()

    rospy.Subscriber("/my_odom", Odometry, state.odometryCallback)
    rospy.Subscriber(
        "/move_base/TebLocalPlannerROS/global_plan", Path, path.pathCallback
    )

    steer_pub = rospy.Publisher("steer_pub", Float32, queue_size=1)

    # ax = path.xs
    # ay = path.ys

    ax = [
        0.0,
        -12.502059936523438,
        -25.434463500976562,
        -33.93553161621094,
        -46.96923065185547,
        -56.460845947265625,
    ]
    ay = [
        0.0,
        -14.23615837097168,
        -29.223724365234375,
        -35.460575103759766,
        -31.393835067749023,
        -21.68545913696289,
    ]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    # print(cx)
    # print(cy)

    last_idx = len(cx) - 1

    target_idx, _ = calc_target_index(state, cx, cy)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        steer_pub.publish(di)
        r.sleep()
