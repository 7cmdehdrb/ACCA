#!/usr/bin/env python

import rospy
from math import tan
import math
import tf
from time import sleep
from genpy.message import SerializationError
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import LaserScan


class Laser(object):
    def __init__(self):
        super(Laser, self).__init__()
        self.ranges = []
        self.result = []
        self.pubRanges = LaserScan()
        self.block = []
        self.block_index = []
        self.angle = []
        self.x = []
        self.y = []
        self.a = []
        self.b = []
        self.tot_x = []
        self.tot_y = []
        self.rr = []
        self.meanx = []
        self.meany = []
        self.cmeanx = []

    def laserCallback(self, msg):
        self.pubRanges = msg
        self.ranges = msg.ranges
        self.laserHandle(self.ranges)

    def laserHandle(self, ranges):
        self.result = []
        self.block = []
        self.block_index = []

        """ Change this part to detect obstacle """
        for i in range(len(self.ranges)-1):
            if self.ranges[i-1] == 0 and self.ranges[i] != 0:
                self.block.append(i)
            if self.ranges[i] != 0 and self.ranges[i+1] == 0:
                if len(self.block) == 0:
                    self.block.append(0)
                self.block.append(i)
            if len(self.block) == 2:
                self.block_index.append(self.block)
                self.block = []

        if len(self.block) == 1:
            self.block_index.append([self.block[0], len(self.ranges)])
        # print(self.block_index, self.ranges)

        for j in range(len(self.ranges)):
            self.angle = float(j/2)
            self.x = self.ranges[j] * math.cos(math.radians(self.angle))
            self.y = self.ranges[j] * math.sin(math.radians(self.angle))
            self.a.append(self.x)
            self.b.append(self.y)

        if len(self.block_index) != 0:
            self.Cent_po()
            self.mean()
            self.result = [self.meanx, self.meany]

        else:
            self.meanx = 0.0
            self.meany = 0.5

        self.cmeanx = -self.meanx
        """END"""

        self.pubRanges.ranges = self.result
        # scan_pub.publish(self.pubRanges)

    def Cent_po(self):
        self.tot_x = []
        self.tot_y = []
        cen_x = 0
        cen_y = 0
        self.rr = []

        for i in range(len(self.block_index)):
            a = self.block_index[i][0]
            b = self.block_index[i][1]
            cen_x = 0
            cen_y = 0
            for j in range(a, b+1):
                cen_x = cen_x + self.a[j]
                cen_y = cen_y + self.b[j]
            self.tot_x.append(cen_x/int(b-a+1))
            self.tot_y.append(cen_y/int(b-a+1))

    def mean(self):
        self.meanx = 0
        self.meany = 0
        a = 0
        b = 0
        self.cmeanx = 0
        for i in range(len(self.tot_x)):
            a = a + self.tot_x[i]
            b = b + self.tot_y[i]
        self.meanx = a/int(len(self.tot_x))
        self.meany = b/int(len(self.tot_y))

        rospy.loginfo(str(self.meany) + ", " + str(self.cmeanx))

        """ END """

    def pubResults(self, publisher):
        cone_pose = PoseStamped()

        try:

            cone_pose.header.stamp = rospy.Time.now()
            cone_pose.header.frame_id = "laser"

            cone_pose.pose.position.x = self.meany
            cone_pose.pose.position.y = self.cmeanx
            cone_pose.pose.position.z = 0.0

            # print(self.meany, self.cmeanx)

            cone_pose.pose.orientation.x = 0.0
            cone_pose.pose.orientation.y = 0.0
            cone_pose.pose.orientation.z = 0.0
            cone_pose.pose.orientation.w = 1.0

            publisher.publish(cone_pose)

        except Exception as ex:
            print(ex)


if __name__ == "__main__":
    rospy.init_node("lms_test")

    laser = Laser()

    cone_pub = rospy.Publisher("cone_position", PoseStamped, queue_size=1)
    rospy.Subscriber("/scan_filtered", LaserScan, laser.laserCallback)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        laser.pubResults(publisher=cone_pub)
        r.sleep()
