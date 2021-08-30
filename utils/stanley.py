#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32


"""

Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer

"""


class Stanley(object):
    def __init__(self):
        super(Stanley, self).__init__()

        self.k = rospy.get_param("/c_gain", 0.5)  # control gain
        self.L = 1.040  # [m] Wheel base of vehicle

        self.ctr_publisher = rospy.Publisher(
            "stanley_ctr", Float32, queue_size=1)
        self.hdr_publisher = rospy.Publisher(
            "stanley_hdr", Float32, queue_size=1)

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(
            state, cx, cy)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(cyaw[current_target_idx] - state.yaw)

        # theta_d corrects the cross track error

        theta_d = np.arctan2(self.k * error_front_axle, state.v)
        # Steering control
        delta = theta_e + theta_d

        self.ctr_publisher.publish(theta_e)
        self.hdr_publisher.publish(theta_d)

        return delta, current_target_idx

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

    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + self.L * np.cos(state.yaw) / 2.0
        fy = state.y + self.L * np.sin(state.yaw) / 2.0

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -
                          np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot(
            [dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
