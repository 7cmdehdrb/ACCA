#!/usr/bin/env python

import sys
import os
import rospy
import tf
import math as m
import numpy as np
import time as t
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from path_planner.msg import stanleyMsg
from loadPose import LoadPose


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    from pure_pursuit import pure_pursuit_control
    from pure_pursuit_global import TargetCourse, pure_pursuit_steer_control
except Exception as ex:
    print(ex)


WB = 1.040
desired_speed = rospy.get_param("/desired_speed", 5.0)  # KPH
k_gain = rospy.get_param("/k_gain", 1.0)
max_steer = rospy.get_param("/max_steer", 30.0)  # DEG


class PurePursuitState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(PurePursuitState, self).__init__(x, y, yaw, v)

        self.rear_x = 0.0
        self.rear_y = 0.0

    def updateRear(self):
        self.rear_x = self.x - ((WB / 2) * m.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * m.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return np.hypot(dx, dy)


class PathFinder(object):
    def __init__(self, load):
        super(PathFinder, self).__init__()

        self.path = Path()

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw


class GlobalPurePursuit(object):
    def __init__(self, state, cmd_msg, cmd_publisher):

        self.state = state
        self.load = LoadPose("odometry1.csv")
        self.path = PathFinder(load=self.load)

        self.target_idx = 10
        self.goal = [0.0, 0.0]

        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_publisher

        self.path_pub = rospy.Publisher(
            "/cublic_global_path", Path, queue_size=1)

        """ MAIN 1 """

        t.sleep(3.0)

        # target_idx = self.target_idx

        self.target_course = TargetCourse(self.path.cx, self.path.cy)

        # target_ind, _ = self.target_course.search_target_index(self.state)

        # self.target_idx = target_ind

    def calc_pure_target_idx(self, cx, cy):
        dx = [self.state.x - icx for icx in cx]
        dy = [self.state.y - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

    def calc_target_idx(self):
        # Search nearest point index
        dx = [self.state.x - icx for icx in self.path.cx]
        dy = [self.state.y - icy for icy in self.path.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

    def getTempGoal(self, gap=100):
        last_idx = len(self.path.cx) - 1
        target_idx = self.calc_target_idx() + gap

        if target_idx > last_idx:
            target_idx = last_idx - 1

        return self.path.cx[target_idx], self.path.cy[target_idx]

    def main(self):
        self.state.updateRear()

        new_target_idx = 0

        # Calc control input
        di, new_target_idx = pure_pursuit_steer_control(
            state, self.target_course, self.target_idx)

        print(new_target_idx)

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        self.target_idx = new_target_idx

        """ PUBLISH """

        self.cmd_msg.speed = desired_speed
        self.cmd_msg.steer = di
        self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)
        self.load.pathPublish(pub=self.path_pub)

        print(self.cmd_msg)

    def main2(self):
        self.state.updateRear()

        self.goal = self.getTempGoal(gap=50)

        steer = pure_pursuit_control(state=self.state, goal=self.goal)

        self.cmd_msg.speed = desired_speed
        self.cmd_msg.steer = steer * k_gain
        self.cmd_msg.brake = 1

        # print(self.cmd_msg)

        self.cmd_pub.publish(self.cmd_msg)
        self.load.pathPublish(pub=self.path_pub)


if __name__ == "__main__":
    rospy.init_node("global_pure_pursuit")

    state = PurePursuitState(x=-7.487, y=-5.565, yaw=-2.182, v=0.0)
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)

    cmd_msg = stanleyMsg()

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    global_pure_pursuit = GlobalPurePursuit(
        state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        global_pure_pursuit.main()
        r.sleep()
