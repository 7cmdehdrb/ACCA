#!/usr/bin/env python

import sys
import rospy
import tf
import math as m
import numpy as np
import time
from std_msgs.msg import UInt8, Int32
from nav_msgs.msg import Path


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")
ODOMETRY_TOPIC = rospy.get_param("/odometry_topic", "/odom")

SPEED = rospy.get_param("/delivery_speed", 5.0)
max_steer = rospy.get_param("/max_steer", 30.0)  # DEG


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils/")
    from state import State
    from loadPose import LoadPose
    from stanley import Stanley
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()


class Goal(object):
    def __init__(self, x, y, yaw, state):
        self.x = x
        self.y = y
        self.yaw = yaw

        self.state = state

    def isEND(self):

        car_VEC = np.array([
            m.cos(self.state.yaw), m.sin(m.sin(self.state.yaw))
        ])

        goal_VEC = np.array([
            self.x - self.state.x,
            self.y - self.state.y
        ])

        dot = np.dot(car_VEC, goal_VEC)


        if dot < 0.0:
            return True

        return False


class Delivery(object):
    def __init__(self, cmd_pub, cmd_msg, state, file_name):
        self.load = LoadPose(file_name=file_name)
        self.state = state

        self.stanley = Stanley()

        self.cmd_pub = cmd_pub
        self.cmd_msg = cmd_msg

        self.Mode = -1

        self.deliveryFlag = False
        self.isEnd = False

        self.last_idx = len(self.load.cx) - 1
        self.target_idx = 0
        self.final_goal = None

        """ Subscriber """

        rospy.Subscriber("/delivery_sign", UInt8,
                         callback=self.deliverySignCallback)
        rospy.Subscriber("/hdl_state", Int32, self.stateCallback)

        """ Publisher """

        self.path_pub = rospy.Publisher("delivery_path", Path, queue_size=10)

    def stateCallback(self, msg):
        data = msg.data

        if data != -1:
            self.Mode = data

    def deliverySignCallback(self, msg):
        data = msg.data

        if self.Mode == 7 or self.Mode == 8:

            if self.deliveryFlag is False:

                if data == 0:
                    self.deliveryFlag = False
                elif data == 1:
                    self.deliveryFlag = True

        else:
            self.reset()
            # print("RESET")

    def doBrake(self, value=100):
        self.cmd_msg.speed = 0.0
        self.cmd_msg.steer = 0.0
        self.cmd_msg.brake = value

        self.cmd_pub.publish(self.cmd_msg)

    def reset(self):
        self.target_idx = 0
        self.deliveryFlag = False
        self.isEnd = False
        self.final_goal = None

    def main(self):

        self.load.pathPublish(pub=self.path_pub)

        if self.final_goal is not None:

            print("FINAL GOAL IS NOT NONE")

            self.isEnd = self.final_goal.isEND()
            print(self.isEnd)

            if self.isEnd is True:

                print("IS END")

                self.doBrake(value=50)
                self.isEnd = True
                return

            di, _ = self.stanley.stanley_control(
                self.state, self.load.cx, self.load.cy, self.load.cyaw, self.target_idx
            )

            di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

            self.cmd_msg.speed = SPEED
            self.cmd_msg.steer = -di
            self.cmd_msg.brake = 1

            self.cmd_pub.publish(self.cmd_msg)

            return

        if self.deliveryFlag is True:

            print("DELIVERY FLAG IS TRUE")

            idx = self.target_idx + 0

            if idx > self.last_idx:
                idx = self.last_idx

            self.target_idx = idx

            self.final_goal = Goal(
                x=self.load.cx[idx],
                y=self.load.cy[idx],
                yaw=self.load.cyaw[idx],
                state=self.state
            )

            return

        di, self.target_idx = self.stanley.stanley_control(
            self.state, self.load.cx, self.load.cy, self.load.cyaw, self.target_idx
        )

        if self.target_idx == self.last_idx:
            self.isEnd = True
            return

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        self.cmd_msg.speed = SPEED
        self.cmd_msg.steer = -di
        self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)
