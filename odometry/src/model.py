#!/usr/bin/env python

import rospy
import math as m


class erp42(object):
    # BASE ERP42 MODEL DEFINE
    def __init__(self, x, y, yaw, v):
        # Initial x, y, yaw, velocity
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        # param
        self.max_steer = m.radians(rospy.get_param("/max_steer", 30.0))
        self.wheel_base = rospy.get_param("/wheel_base", 1.040)
        self.dt = 0.1

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > m.pi:
            angle -= 2.0 * m.pi

        while angle < -m.pi:
            angle += 2.0 * m.pi

        return angle
