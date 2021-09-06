#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math as m
from path_selector import PathSelector
from path_planner.msg import stanleyMsg
from nav_msgs.msg import Odometry, Path

ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")
ODOMETRY_TOPIC = rospy.get_param("/odometry_topic", "/odom")

desired_speed = rospy.get_param("/parking_speed", 0.5)
max_steer = rospy.get_param("/max_steer", 30.0)

backward_distance = rospy.get_param("/backward_distance", 8.0)

try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils")
    from stanley import Stanley
    from state import State
    from pure_pursuit import PurePursuit
except Exception as ex:
    print("GLOBAL STANLEY IMPORT ERROR")
    print(ex)


"""

Subscribe 'parking' and select path and
Publish 'Control_msg'

Subscriber is located in 'PathSelector' class

"""


class PathFinder(object):
    def __init__(self, load):
        super(PathFinder, self).__init__()

        self.load = load

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw

    def update(self, load):
        self.load = load

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
        self.pathIdx = self.path_selector.getIdx()
        self.main_path = PathFinder(
            self.path_selector.getPath)  # class : LoadPose

        self.pp = PurePursuit(state=self.state, cmd_msg=self.cmd_msg,
                              cmd_pub=self.cmd_pub, load=self.main_path, speed=desired_speed)

        self.last_idx = len(self.main_path.cx) - 1

        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.main_path.cx, self.main_path.cy
        )

    def checkGoal(self, speed, di):
        if self.target_idx >= self.last_idx - 1:
            self.cmd_msg.speed = 0.0
            self.cmd_msg.steer = 0.0
            self.cmd_msg.brake = 100

            self.state.parkingFlag = False

            return True

        return False

    def checkGoal2(self):
        distance = np.hypot(
            self.main_path.cx[-1] - self.state.x, self.main_path.cy[-1] - self.state.y)

        if distance < 0.2:
            self.cmd_msg.speed = 0.0
            self.cmd_msg.steer = 0.0
            self.cmd_msg.brake = 100

            self.state.parkingFlag = False

            return True

        return False

    def goBack(self, speed):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            distance = np.hypot(
                self.main_path.cx[self.last_idx] - self.state.x, self.main_path.cy[self.last_idx] - self.state.y)

            self.cmd_msg.speed = speed * -1.0
            self.cmd_msg.steer = 0.0
            self.cmd_msg.brake = 1

            self.cmd_pub.publish(self.cmd_msg)

            if distance > backward_distance:
                break

            rate.sleep()

        self.state.backwardFlag = False

    def checkMainPath(self):
        if self.pathIdx != self.path_selector.getIdx():
            return True

        return False

    def main(self):
        if self.checkMainPath() is True:
            self.pathIdx = self.path_selector.getIdx()
            self.main_path.update(load=self.path_selector.getPath)
            self.target_idx, _ = self.stanley.calc_target_index(
                self.state, self.main_path.cx, self.main_path.cy
            )

        target_idx = self.target_idx

        di, target_idx = self.stanley.stanley_control(
            self.state, self.main_path.cx, self.main_path.cy, self.main_path.cyaw, target_idx)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        if self.checkGoal(speed=desired_speed, di=di) is not True:
            self.cmd_msg.speed = desired_speed
            self.cmd_msg.steer = -di
            self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)
        self.main_path.load.pathPublish(pub=self.path_pub)

        print(self.cmd_msg)

    def main2(self):
        if self.checkMainPath() is True:
            self.pathIdx = self.path_selector.getIdx()
            self.main_path.update(load=self.path_selector.getPath)
            self.pp.re_calcalc_point()

        if self.pp.ind == 0:
            self.target_idx = self.pp.closest_path_point()
            # OOP

        cx, cy = self.pp.find_goal_point()
        di = self.pp.set_steering()
        self.target_idx = self.pp.ind

        if self.checkGoal2() is not True:
            self.cmd_msg.speed = desired_speed
            self.cmd_msg.steer = di
            self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)
        self.main_path.load.pathPublish(pub=self.path_pub)

        print(self.cmd_msg)


if __name__ == "__main__":
    rospy.init_node("parking")

    state = State(x=0.0, y=0.0, v=0.0, yaw=0.0)
    state.parkingFlag = True
    state.backwardFlag = True

    cmd_msg = stanleyMsg()
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)

    rospy.Subscriber(ODOMETRY_TOPIC, Odometry, callback=state.odometryCallback)

    parking = Parking(state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        parking.main()
        r.sleep()
