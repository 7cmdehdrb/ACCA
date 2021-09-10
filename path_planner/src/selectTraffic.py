#!/usr/bin/env python


import sys
import rospy

# 0: STOP 1: STRAIGHT 2: LEFT


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

        if left is True and straight is False:
            return False

    except ValueError:
        return False

    return True


def isTrafficLeft(traffic=[]):
    try:
        stop, straight, left = distinguishTraffic(traffic=traffic)

        if left is True:
            return True

        if straight is True:
            return False

        if straight is True and left is False:
            return False

    except ValueError:
        return False

    return True
