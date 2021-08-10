#!/usr/bin/env python

import rospy
import rospkg
import tf
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Vector3, Twist, Quaternion
from model import erp42

"""
    TF test:
    change odom frame => map frame
"""


class TF_map(object):
    def __init__(self):
        super(TF_map, self).__init__()

        self.trans = [0.0, 0.0, 0.0]
        self.rot = [0.0, 0.0, 0.0, 1.0]

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.subscribedMsg = Odometry()

    def subscribeOdom(self, msg):
        self.subscribedMsg = msg

    def listenTF(self):
        try:
            (trans, rot) = listener.lookupTransform(
                "/map", "/odom", rospy.Time(0))
            self.trans = trans
            self.rot = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        except Exception as ex:
            print(ex)

    def publishOdom(self):
        msg = Odometry()

        self.currentTime = rospy.Time.now()

        x, y, yaw = self.transform()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        msg.header.stamp = self.currentTime
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose = Pose(
            Point(x, y, 0.0), Quaternion(*odom_quat)
        )

        msg.twist.twist = Twist(
            Vector3(
                self.subscribedMsg.twist.twist.linear.x,
                self.subscribedMsg.twist.twist.linear.y,
                0.0
            ),
            Vector3(
                0.0,
                0.0,
                self.subscribedMsg.twist.twist.angular.z
            )
        )

        odom_pub.publish(msg)

    def transform(self):
        (_, _, rot) = tf.transformations.euler_from_quaternion(
            [self.rot[0], self.rot[1], self.rot[2], self.rot[3]])

        x = self.subscribedMsg.pose.pose.position.x
        y = self.subscribedMsg.pose.pose.position.y

        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [
                self.subscribedMsg.pose.pose.orientation.x,
                self.subscribedMsg.pose.pose.orientation.y,
                self.subscribedMsg.pose.pose.orientation.z,
                self.subscribedMsg.pose.pose.orientation.w,
            ]
        )

        new_x = x + self.trans[0]
        new_y = y + self.trans[1]
        new_yaw = yaw + rot
        # new_yaw = yaw

        return new_x, new_y, new_yaw


if __name__ == "__main__":
    rospy.init_node("tf_map")

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    tf_map = TF_map()

    rospy.Subscriber("/erp42/erp42_odometry", Odometry, tf_map.subscribeOdom)

    odom_pub = rospy.Publisher("transformed_odom", Odometry, queue_size=1)

    r = rospy.Rate(300.0)
    while not rospy.is_shutdown():
        tf_map.listenTF()
        tf_map.publishOdom()
        r.sleep()
