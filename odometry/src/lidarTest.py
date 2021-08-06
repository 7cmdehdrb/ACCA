#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class Laser(object):
    def __init__(self):
        super(Laser, self).__init__()
        self.ranges = []
        self.result = []
        self.laserData = LaserScan()

    def laserCallback(self, msg):
        self.laserData = msg
        self.laserHandle(self.laserData)

    def laserHandle(self, ranges):
        result = []

        distance = 9999

        """ Change this part to detect obstacle """

        cnt = 0

        for r in self.laserData.ranges:
            if 270 - 60 < cnt and cnt < 270 + 60:
                if r != 0:
                    result.append(r)
            cnt += 1

        """ END """

        distance = self.gerLargest(result)

        dis_pub.publish(distance)

    def gerLargest(array):
        largest = -9999
        for num in array:
            if num > largest:
                largest = num

        return largest


if __name__ == "__main__":
    rospy.init_node("lms_control")

    laser = Laser()

    # scan_pub = rospy.Publisher("test_scan", LaserScan, queue_size=1)
    dis_pub = rospy.Publisher("laser_distance", Float32, queue_size=1)

    rospy.Subscriber("/scan", LaserScan, laser.laserCallback)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
