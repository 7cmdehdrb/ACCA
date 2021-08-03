#!/usr/bin/env python

import rospy
import math as m
import tf
from nav_msgs.msg import OccupancyGrid


class MapReceiver:
    def __init__(self):
        self.msg = OccupancyGrid()

    def girdMapCallback(self, msg):
        self.msg = msg

        print(self.msg.header)
        print(self.msg.info)


if __name__ == "__main__":
    rospy.init_node("map_receiver")

    map_receiver = MapReceiver()

    rospy.Subscriber(
        "/hdl_global_localization/bbs/gridmap",
        OccupancyGrid,
        map_receiver.girdMapCallback,
    )

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
