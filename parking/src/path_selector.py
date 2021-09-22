#!/usr/bin/env python

import sys
import rospy
import time as t
from std_msgs.msg import Int32MultiArray

ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils")
    from loadPose import LoadPose
except ImportError as ie:
    print(ie)
    print("UTILS IMPORT ERROR")


class PathSelector(object):
    def __init__(self):
        super(PathSelector, self).__init__()

        rospy.Subscriber("/parking", Int32MultiArray,
                         self.parkingCallback)

        self.flag = False
        self.idx = 0

        self.len = -1
        self.pathArray = []
        self.result = []
        self.main_path = None

        while ((self.flag is not True) and (not rospy.is_shutdown())):
            rospy.loginfo("WAIT FOR PARKING TOPIC...")
            t.sleep(0.1)

        for i in range(self.len):
            try:
                f = "kcity_parking" + str(i) + ".csv"
                self.pathArray.append(LoadPose(file_name=f))
            except IOError as ioe:
                self.pathArrsave_file_nameay.append(
                    LoadPose(file_name="no_path.csv"))
                print("IO ERROR ON CSV FILE")

    def getIdx(self):
        return self.idx

    def setIdx(self, idx):
        self.idx = idx
        return self.idx

    @property
    def getPath(self):
        return self.pathArray[self.getIdx()]

    def parkingCallback(self, msg):

        temp = msg.data

        if self.flag is False and len(temp) != 0:
            self.len = len(temp)
            self.flag = True

            for i in range(self.len):
                self.result.append(0)

            return

        for i in range(len(temp)):
            if temp[i] == 1:
                self.result[i] = 1

        for i in range(len(self.result)):
            if self.result[i] == 0:
                self.setIdx(i)
                break

        # print(self.getIdx)
