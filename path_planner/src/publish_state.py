#!/usr/bin/env python

import rospy
import rospkg
import sys
import os
import tf
import csv
import math as m
import numpy as np
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Empty, Int32
from visualization_msgs.msg import MarkerArray, Marker


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
except Exception as ex:
    try:
        sys.path.insert(0, "ADD PATH HERE")
        from state import State
    except Exception as ex:
        print(ex)

"""

State publisher for STATE MACHINE
This node reads /path_planner/saved_path/area.csv

The CSV format is 'x_center_position, y_center_position, radius, ID'

IDs are depends on STATE MACHINE

"""


class Area(object):
    def __init__(self, x, y, r, idx):
        super(Area, self).__init__()

        self.__idx = idx

        self.x = x
        self.y = y
        self.r = r

    @property
    def getIdx(self):
        return self.__idx

    @property
    def setIdx(self, idx):
        self.__idx = idx
        return self.__idx

    def checkArea(self, state):
        distance = np.hypot(state.x - self.x, state.y - self.y)

        if distance < self.r / 2:
            return self.__idx
        else:
            return -1


class PublishState(State):
    def __init__(self, x, y, yaw, v):
        super(PublishState, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.areas = []

    def checkAreas(self):
        result_idx = -1

        for area in self.areas:
            idx = area.checkArea(state=self)

            if idx != -1:
                result_idx = idx

        return result_idx

    def publishAreas(self, publisher):
        msg = MarkerArray()

        for area in self.areas:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.lifetime = rospy.Duration(0.2)
            marker.ns = str(area.getIdx) + str(area.x)

            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.scale.x = area.r
            marker.scale.y = area.r
            marker.scale.z = 0.1

            marker.color.a = 0.5
            marker.color.r = 1.0 if int(area.getIdx) == 1 else 0.0
            marker.color.g = 1.0 if int(area.getIdx) == 2 else 0.0
            marker.color.b = 1.0 if int(area.getIdx) == 3 else 0.0

            marker.pose.position.x = area.x
            marker.pose.position.y = area.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            msg.markers.append(marker)

        publisher.publish(msg)


def read_csv(state):
    output_file_path = rospkg.RosPack().get_path(
        'path_planner')+"/saved_path/area.csv"

    rospy.loginfo("LOADING CSV FILE...")
    with open(output_file_path, "r") as csvFile:
        reader = csv.reader(csvFile, delimiter=",")
        for row in reader:
            area = Area(x=float(row[0]), y=float(row[1]),
                        r=float(row[2]), idx=int(row[3]))
            state.areas.append(area)

    rospy.loginfo("LOADING FINISHED")


if __name__ == "__main__":
    rospy.init_node("publish_state")

    state = PublishState(x=0.0, y=0.0, yaw=0.0, v=0.0)

    read_csv(state)

    rospy.Subscriber("/odom", Odometry, callback=state.odometryCallback)

    hdl_state_publisher = rospy.Publisher("hdl_state", Int32, queue_size=1)

    hdl_area_publisher = rospy.Publisher(
        "hdl_areas", MarkerArray, queue_size=1)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        idx = state.checkAreas()
        hdl_state_publisher.publish(idx)
        state.publishAreas(publisher=hdl_area_publisher)
        r.sleep()
