#!/usr/bin/env python

import sys
import rospy
import time as t
from std_msgs.msg import UInt8MultiArray


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from loadPose import LoadPose
except ImportError as ie:
    print(ie)
    print("UTILS IMPORT ERROR")


class PathSelector(object):
    def __init__(self):
        super(PathSelector, self).__init__()

        rospy.Subscriber("/parking", UInt8MultiArray,
                         self.parkingCallback)

        self.flag = False
        self.__idx = 0

        self.len = -1
        self.pathArray = []
        self.main_path = None

        while ((self.flag is not True) and (not rospy.is_shutdown())):
            rospy.loginfo("WAIT FOR PARKING TOPIC...")
            t.sleep(0.1)

        for i in range(self.len):
            try:
                f = "parking_path" + str(i) + ".csv"
                self.pathArray.append(LoadPose(file_name=f))
            except IOError as ioe:
                self.pathArrsave_file_nameay.append(
                    LoadPose(file_name="no_path.csv"))
                print("IO ERROR ON CSV FILE")

    @property
    def getIdx(self):
        return self.__idx

    @property
    def setIdx(self, idx):
        self.__idx = idx
        return self.__idx

    @property
    def getPath(self):
        return self.pathArray[self.getIdx]

    def parkingCallback(self, msg):
        data = msg.data

        if self.flag is False and len(data) != 0:
            self.len = len(data)
            self.flag = True

        for i in range(len(data)):
            if data[i] == 1:
                self.setIdx(i)
                break
