#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid


class Map(object):
    def __init__(self):
        super(Map, self).__init__()
        self.mapServer = OccupancyGrid()
        self.hdlMap = OccupancyGrid()

    @property
    def mapCallback(self, msg):
        try:
            self.mapServer = msg
            print(self.mapServer.header)
        except Exception as ex:
            print(ex)

    def hdlCallback(self, msg):
        try:
            self.hdlMap = msg
            print(self.hdlMap.header)
        except Exception as ex:
            print(ex)


if __name__ == "__main__":
    rospy.init_node("map_receiver")

    map = Map()

    try:

        rospy.Subscriber("/map", OccupancyGrid, OccupancyGrid, map.mapCallback)
        rospy.Subscriber("/hdl_global_localization/bbs/gridmap",
                         OccupancyGrid, map.hdlCallback)

    except Exception as ex:
        print(ex)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()
