#!/usr/bin/env python


import sys
import rospy
import tf
import math as m
import numpy as np
import time as t
import threading
from path_planner.msg import obTF, stanleyMsg
from nav_msgs.msg import Path, Odometry


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")
ODOMETRY_TOPIC = rospy.get_param("/odometry_topic", "/odom")

max_steer = rospy.get_param("/max_steer", 30.0)  # DEG


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils")
    from stanley import Stanley
    from state import State
    from loadPose import LoadPose
except Exception as ex:
    print("GLOBAL STANLEY IMPORT ERROR")
    print(ex)


class PathSwitcher(object):
    def __init__(self, state, cmd_pub, cmd_msg=stanleyMsg(), file_name="path"):

        self.paths = []
        self.file_name = file_name

        # MSG
        self.ob_TF = obTF()
        self.cmd_msg = cmd_msg

        # Subscriber
        rospy.Subscriber("/ob_TF", obTF, callback=self.obstacleCallback)

        # Publisher
        self.cmd_pub = cmd_pub
        self.path_pub = rospy.Publisher(
            "/path_switcher_path", Path, queue_size=10)

        # Object
        self.state = state
        self.stanley = Stanley()

        # Custom Field
        self.current_path_idx = -1
        self.current_path = None
        self.is_doing_switching = False
        self.errTolerance_HDR = 0.1
        self.errTolerance_CTR = 0.01
        self.changeTime = rospy.Time.now()

        self.speed = rospy.get_param("/desired_speed", 2.0)
        self.brake = 1

        """ INIT """

        self.stanley_init()

        """ TEST """

        # self.doTest()

    def appendPath(self, path):
        self.paths.append(path)

    def setPath(self):
        while not rospy.is_shutdown():
            idx = input("INPUT IDX: ")

            try:
                idx = int(idx)

                if idx == 9:
                    sys.exit()
                    return

                if not (idx == 0 or idx == 1):
                    raise ValueError

            except Exception as ex:
                print(ex)
                continue

            self.is_doing_switching = True
            self.changeTime = rospy.Time.now()

            self.current_path_idx = idx
            self.current_path = self.paths[self.current_path_idx]

            self.target_idx, _ = self.stanley.calc_target_index(
                self.state, self.current_path.cx, self.current_path.cy
            )

    def setSpeed(self, value):
        self.speed = value

    def setBrake(self, value):
        self.brake = value

    def obstacleCallback(self, msg):
        self.ob_TF = msg

    def doSwitching(self):

        # When call this function, path will be changed to another path
        # And then, is_doing_switching field will be changed to True

        try:

            if self.current_path_idx == 0 or 1:
                self.current_path_idx = 0 if self.current_path_idx == 1 else 1
                self.current_path = self.paths[self.current_path_idx]

                self.target_idx, _ = self.stanley.calc_target_index(
                    self.state, self.current_path.cx, self.current_path.cy
                )

                self.current_path.pathPublish(pub=self.path_pub)

                self.is_doing_switching = True

        except ValueError:
            print("WRONG IDX")

    def isDoingSwitching(self):

        # Call in While
        # 1. Check CTR and HDR and check is path switching is finished
        # 2. Check ob_TF and determine should change path or not

        if self.is_doing_switching is True:

            now = rospy.Time.now()
            gap = (now - self.changeTime).to_sec()

            if gap < 1.0:
                return

            hdr = self.stanley.getHDR()
            ctr = self.stanley.getCTR()

            # (-0.33111521135259675, 0.00355392724477499) =>
            # (-0.05175765554117815, 0.0003130744525169138)

            print(hdr, ctr)

            if abs(hdr) < self.errTolerance_HDR and abs(ctr) < self.errTolerance_CTR:
                self.changeTime = rospy.Time.now()
                self.is_doing_switching = False

        else:
            if self.ob_TF.front_left == 1 or self.ob_TF.front_right == 1:
                self.doSwitching()

    def stanley_init(self):
        for i in range(2):
            ns = self.file_name + str(i) + ".csv"
            self.appendPath(LoadPose(file_name=ns))

        self.current_path_idx = 0
        self.current_path = self.paths[self.current_path_idx]

        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.current_path.cx, self.current_path.cy
        )

    def doTest(self):
        th = threading.Thread(target=self.setPath)
        th.start()

    def main(self):
        self.isDoingSwitching()

        # hdr = self.stanley.getHDR()
        # ctr = self.stanley.getCTR()

        # print(hdr, ctr)

        di, self.target_idx = self.stanley.stanley_control(
            self.state, self.current_path.cx, self.current_path.cy, self.current_path.cyaw, self.target_idx
        )

        self.cmd_msg.speed = self.speed
        self.cmd_msg.steer = -di
        self.cmd_msg.brake = self.brake

        self.cmd_pub.publish(self.cmd_msg)


if __name__ == "__main__":
    rospy.init_node("path_switcher")

    # Object
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    # Publisher
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)

    # Subscriber
    rospy.Subscriber(ODOMETRY_TOPIC, Odometry, callback=state.odometryCallback)

    # Main
    path_switcher = PathSwitcher(
        state=state, cmd_pub=cmd_pub, file_name="test_path")

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        path_switcher.main()
        r.sleep
