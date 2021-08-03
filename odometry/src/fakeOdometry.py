#!/usr/bin/env python

import rospy
import math as m
import tf
from model import erp42
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class FakeOdometry(erp42):
    def __init__(self, x, y, yaw, v):
        super(FakeOdometry, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()
        self.dt = 0.0

    def update(self):
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)
        self.x += self.dx * self.dt
        self.y += self.dy * self.dt

    def publishOdom(self):
        odom = Odometry()

        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        self.update()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

        odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0), odom_quat, self.currentTime, "base_link", "odom"
        )

        odom.header.stamp = self.currentTime
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))

        odom.twist.twist = Twist(
            Vector3(self.dx, self.dy, 0.0), Vector3(0.0, 0.0, (0.0))
        )

        odom_pub.publish(odom)

        self.lastTime = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("fake_odometry")

    fake_odom = FakeOdometry(x=0.0, y=0.0, yaw=0.0, v=0.0)

    odom_pub = rospy.Publisher("/fake_odom", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        fake_odom.publishOdom()
        r.sleep()