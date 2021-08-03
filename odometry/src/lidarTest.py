#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


class Laser(object):
    def __init__(self):
        super(Laser, self).__init__()
        self.ranges = []
        self.result = []
        self.pubRanges = LaserScan()

    def laserCallback(self, msg):
        self.pubRanges = msg
        self.ranges = msg.ranges
        self.laserHandle(self.ranges)

    def laserHandle(self, ranges):
        self.result = []

        """ Change this part to detect obstacle """

        for r in ranges:
            if r < 0.3:
                self.result.append(r)
            else:
                self.result.append(0.0)

        """ END """

        self.pubRanges.ranges = self.result

        scan_pub.publish(self.pubRanges)


if __name__ == "__main__":
    rospy.init_node("lms_test")

    laser = Laser()

    scan_pub = rospy.Publisher("test_scan", LaserScan, queue_size=1)

    rospy.Subscriber("/scan", LaserScan, laser.laserCallback)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
