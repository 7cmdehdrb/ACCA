#!/usr/bin/env python


import sys
import rospy
import numpy as np
import math as m

# 0: STOP 1: STRAIGHT 2: LEFT


class Line(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class TrafficStopLine(object):
    def __init__(self, state):

        self.state = state
        self.lines = []

    def getNearestLine(self):
        result = None
        min_dist = float("inf")

        for line in self.lines:
            distance = np.hypot(line.x - self.state.x, line.y - self.state.y)

            if distance < min_dist:
                min_dist = distance
                result = line

        return result

    def isEND(self):
        nearest_line = self.getNearestLine()

        if nearest_line is None:
            return False

        line_VEC = np.array([
            nearest_line.x - self.state.x, nearest_line.y - self.state.y
        ])

        car_VEC = np.array([
            m.cos(self.state.yaw), m.sin(self.state.yaw)
        ])

        dot = np.dot(line_VEC, car_VEC)

        if dot < 0:
            return True
        else:
            return False


def distinguishTraffic(traffic):

    stop = False
    straight = False
    left = False

    for t in traffic:
        if t == 0:
            stop = True
        elif t == 1:
            straight = True
        elif t == 2:
            left = True
        else:
            raise ValueError

    return stop, straight, left


def isTrafficStraight(traffic=[]):
    try:
        stop, straight, left = distinguishTraffic(traffic=traffic)

        if straight is True:
            return True

        if stop is True:
            return False

        # if left is True and straight is False:
        #     return False

    except ValueError:
        return False

    return True


def isTrafficLeft(traffic=[]):
    try:
        stop, straight, left = distinguishTraffic(traffic=traffic)

        if left is True:
            return True

        if straight is True and left is False:
            return False

        # if straight is True and left is False:
        #     return False

    except ValueError:
        return False

    return True
