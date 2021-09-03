#!/usr/bin/env python

import sys
import rospy
# import tf
# import os
import math as m
import numpy as np
from nav_msgs.msg import Odometry, Path
# from geometry_msgs.msg import Twist
from path_planner.msg import stanleyMsg


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from stanley import Stanley
    from state import State
    from loadPose import LoadPose
except Exception as ex:
    print("GLOBAL STANLEY IMPORT ERROR")
    print(ex)


"""

Export module for global path and stanley control

"""


desired_speed = rospy.get_param("/desired_speed", 5.0)  # KPH
max_steer = rospy.get_param("/max_steer", 30.0)  # DEG


class PathFinder(object):
    def __init__(self, load):
        super(PathFinder, self).__init__()

        self.path = Path()

        self.load = load

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw

    def update(self, load):
        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw


class GlobalStanley(object):
    def __init__(self, state, cmd_msg, cmd_publisher):
        super(GlobalStanley, self).__init__()

        self.state = state
        self.stanley = Stanley()
        self.load = LoadPose(file_name="path.csv")
        self.path = PathFinder(load=self.load)

        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_publisher

        self.path_pub = rospy.Publisher(
            "/cublic_global_path", Path, queue_size=1)

        self.last_idx = len(self.path.cx) - 1
        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.path.cx, self.path.cy)

    def main(self):
        target_idx = self.target_idx
        di, target_idx = self.stanley.stanley_control(
            self.state, self.path.cx[:target_idx+100], self.path.cy[:target_idx+100], self.path.cyaw[:target_idx+100], target_idx)

        self.target_idx = target_idx
        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        speed, brake = checkGoal(
            last_idx=self.last_idx, current_idx=self.target_idx)

        self.cmd_msg.speed = speed
        self.cmd_msg.steer = -di
        self.cmd_msg.brake = brake

        self.cmd_pub.publish(self.cmd_msg)
        self.path.load.pathPublish(pub=self.path_pub)
        # self.load.pathPublish(pub=self.path_pub)

        print(self.cmd_msg)


def checkGoal(last_idx, current_idx):
    global desired_speed

    temp_speed = 0.0
    temp_brake = 1

    if abs(last_idx - current_idx) < 10:
        temp_speed = 0.0
        temp_brake = 80

    else:
        temp_speed = desired_speed
        temp_brake = 1

    return temp_speed, temp_brake


if __name__ == "__main__":
    rospy.init_node("global_stanley")

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)

    cmd_msg = stanleyMsg()

    rospy.Subscriber("/odom", Odometry, state.odometryCallback)

    global_stanley = GlobalStanley(
        state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        global_stanley.main()
        r.sleep()
