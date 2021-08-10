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
        self.block = []
        self.block_index = []
        self.alp = []
        self.bet = []
        self.tot_x = []
        self.tot_y = []
        self.meanx = []
        self.meany = []
        self.cmeanx = []

    def laserCallback(self, msg):
        self.ranges = msg.ranges
        self.laserHandle(self.ranges)

    def laserHandle(self, ranges):
        self.block = []
        self.block_index = []
        self.alp = []
        self.bet = []
        """ Change this part to detect obstacle """
        for i in range(1, len(self.ranges)-1):
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
            angle = j/2
            x = self.ranges[j] * math.cos(math.radians(angle))
            y = self.ranges[j] * math.sin(math.radians(angle))
            self.alp.append(x)
            self.bet.append(y)

        if len(self.block_index) == 0:

            self.meanx = 0.0
            self.meany = 0.5

        else:
            self.Cent_po()
            self.mean()

        self.cmeanx = -self.meanx

        """END"""

    def Cent_po(self):
        self.tot_x = []
        self.tot_y = []

        for i in range(len(self.block_index)):
            a = self.block_index[i][0]
            b = self.block_index[i][1]
            cen_x = 0
            cen_y = 0
            for j in range(a, b+1, 1):
                cen_x += self.alp[j]
                cen_y += self.bet[j]
            self.tot_x.append(cen_x/(b-a+1))
            self.tot_y.append(cen_y/(b-a+1))

    def mean(self):
        self.meanx = []
        self.meany = []
        a = 0
        b = 0
        for i in range(len(self.tot_x)):
            a += self.tot_x[i]
            b += self.tot_y[i]
        self.meanx = a/len(self.tot_x)
        self.meany = b/len(self.tot_y)

        # print(self.meany,self.cmeanx)

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
