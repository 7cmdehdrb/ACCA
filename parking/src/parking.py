#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math as m
from path_selector import PathSelector
from path_planner.msg import stanleyMsg
from nav_msgs.msg import Odometry, Path


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from stanley import Stanley
    from state import State
except Exception as ex:
    print("GLOBAL STANLEY IMPORT ERROR")
    print(ex)


desired_speed = rospy.get_param("/parking_speed", 3.0)
max_steer = rospy.get_param("/max_steer". 30.0)


class PathFinder(object):
    def __init__(self, load):
        super(PathFinder, self).__init__()

        self.load = load

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw

    def update(self, load):
        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw


class Parking(object):
    def __init__(self, state, cmd_msg, cmd_publisher):
        super(Parking, self).__init__()

        self.state = state
        self.stanley = Stanley()
        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_publisher
        self.path_pub = rospy.Publisher("parking_path", Path, queue_size=1)

        self.path_selector = PathSelector()
        self.pathIdx = self.path_selector.getIdx
        self.main_path = PathFinder(
            self.path_selector.getPath)  # class : LoadPose

        self.lastInx = len(self.main_path.cx) - 1
        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.main_path.cx, self.main_path.cy
        )

    def checkGoal(self, speed, di):
        if self.target_idx == self.lastInx:
            self.cmd_msg.speed = 0.0
            self.cmd_msg.steer = 0.0
            self.cmd_msg.brake = 100

        else:
            self.cmd_msg.speed = speed
            self.cmd_msg.steer = -di
            self.cmd_msg.brake = 1

    def checkMainPath(self):
        if self.pathIdx != self.path_selector.getIdx:
            self.pathIdx = self.path_selector.getIdx
            self.main_path.update(load=self.path_selector.getPath)
            self.target_idx, _ = self.stanley.calc_target_index(
                self.state, self.main_path.cx, self.main_path.cy
            )

    def main(self):
        self.checkMainPath()

        target_idx = self.target_idx

        di, target_idx = self.stanley.stanley_control(
            self.state, self.path.cx, self.path.cy, self.path.cyaw, target_idx)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        self.checkGoal(speed=desired_speed, di=di)

        self.cmd_pub.publish(self.cmd_msg)
        self.main_path.load.pathPublish(pub=self.path_pub)

        print(self.cmd_msg)


if __name__ == "__main__":
    rospy.init_node("parking")

    state = State(x=0.0, y=0.0, v=0.0, yaw=0.0)

    cmd_msg = stanleyMsg()
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)

    rospy.Subscriber("/odom", Odometry, callback=state.odometryCallback)

    parking = Parking(state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        parking.main()
        r.sleep()
