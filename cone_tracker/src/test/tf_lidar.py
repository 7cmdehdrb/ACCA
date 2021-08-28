#!/usr/bin/env python

import rospy
import tf
import math as m
from geometry_msgs.msg import PoseStamped, Pose

"""
    LiDAT TF:
    PoseArray
"""


class TF_lidar(object):
    def __init__(self):
        super(TF_lidar, self).__init__()

        self.trans = [0.0, 0.0, 0.0]
        self.rot = [0.0, 0.0, 0.0, 1.0]

        self.subscribedMsg = PoseStamped()

    def subscribePose(self, msg):
        self.subscribedMsg = msg

    def listenTF(self):
        try:
            (trans, rot) = listener.lookupTransform(
                "/odom", "/laser", rospy.Time(0))
            self.trans = trans
            self.rot = rot

            print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        except Exception as ex:
            print(ex)

    def publishTransformedPose(self, publisher):
        new_msg = self.transform()
        publisher.publish(new_msg)

    def transform(self):
        new_msg = PoseStamped()

        new_msg.header.stamp = rospy.Time.now()
        new_msg.header.frame_id = "odom"

        new_msg.pose.position.x = self.subscribedMsg.pose.position.x + \
            self.trans[0]
        new_msg.pose.position.y = self.subscribedMsg.pose.position.y + \
            self.trans[1]
        new_msg.pose.position.z = 0.0

        new_msg.pose.orientation.x = 0.0
        new_msg.pose.orientation.y = 0.0
        new_msg.pose.orientation.z = 0.0
        new_msg.pose.orientation.w = 1.0

        return new_msg


if __name__ == "__main__":
    rospy.init_node("cone_tf_publisher")

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    tf_pose = TF_lidar()

    rospy.Subscriber("/cone_position", PoseStamped, tf_pose.subscribePose)

    transformed_pub = rospy.Publisher(
        "transformed_cone_position", PoseStamped, queue_size=1)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        tf_pose.listenTF()
        tf_pose.publishTransformedPose(publisher=transformed_pub)
        r.sleep()
