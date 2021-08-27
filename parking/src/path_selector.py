#!/usr/bin/env python

import sys
import rospy
from loadPose import LoadPose
from std_msgs.msg import UInt8MultiArray


class PathSelector(object):
    def __init__(self):
        super(PathSelector, self).__init__()

        rospy.Subscriber("/parking", UInt8MultiArray,
                         self.parkingCallback)

        self.__idx = 0

        self.pathArray = []
        self.main_path = None

        for i in range(4):
            try:
                f = "parking_path" + str(i) + ".csv"
                self.pathArray.append(LoadPose(file_name=f))
            except IOError as ioe:
                self.pathArray.append(LoadPose(file_name="no_path.csv"))
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

        for i in range(len(data)):
            if data[i] == 1:
                self.setIdx(i)
                break
