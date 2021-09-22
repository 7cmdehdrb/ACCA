#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import tf
from model import erp42
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from path_planner.msg import stanleyMsg


MAX_STEER = rospy.get_param("/max_steer", 30.0)


class FakeOdometry(erp42):
    def __init__(self, x, y, yaw, v):
        super(FakeOdometry, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()
        self.dt = 0.1

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

        odom.pose.pose = Pose(Point(self.x, self.y, 0.0),
                              Quaternion(*odom_quat))

        odom.twist.twist = Twist(
            Vector3(self.dx, self.dy, 0.0), Vector3(
                0.0, 0.0, self.yaw / self.dt)
        )

        odom_pub.publish(odom)

        self.lastTime = rospy.Time.now()

    def cmd_callback(self, msg):
        data = Twist()
        data = msg

        steer = np.clip(data.angular.z, -m.radians(30), m.radians(30))

        self.v = data.linear.x
        self.yaw += (self.v / 1.040) * m.tan(-steer) * self.dt

    def cmd_callback2(self, msg):
        data = stanleyMsg()
        data = msg

        speed = data.speed
        steer = np.clip(data.steer, -m.radians(MAX_STEER),
                        m.radians(MAX_STEER))
        brake = data.brake

        # print(speed, m.degrees(steer), brake)

        self.v = speed
        self.yaw += (self.v / 1.040) * m.tan(-steer) * self.dt


if __name__ == "__main__":
    rospy.init_node("fake_odometry")

    _, _, yaw = tf.transformations.euler_from_quaternion(
        [0.0, 0.0, 0.74161089767, 0.670830288864])

    fake_odom = FakeOdometry(x=-7.487, y=-5.565, yaw=-2.182, v=0.0)
    # fake_odom = FakeOdometry(
    #     x=0.932420730591, y=0.035516500473, yaw=yaw, v=0.0)
    # fake_odom = FakeOdometry(-7.3254506274086735, -
    #                          8.26976220919032, -2.33271992886, 0.0)

    odom_pub = rospy.Publisher("/fake_odom", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    # rospy.Subscriber("/rrt_star_cmd",
    #                  stanleyMsg, fake_odom.cmd_callback2)
    rospy.Subscriber("/Control_msg",
                     stanleyMsg, fake_odom.cmd_callback2)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        fake_odom.publishOdom()
        r.sleep()
