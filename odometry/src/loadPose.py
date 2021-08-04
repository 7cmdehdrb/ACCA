#!/usr/bin/env python

import rospy
import rospkg
import threading
import tf
import csv
import math as m
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped


class LoadPose(object):
    def __init__(self):
        super(LoadPose, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []

    def readCSV(self):
        output_file_path = rospkg.RosPack().get_path('odometry')+"/saved_path/pose.csv"

        rospy.loginfo("LOADING CSV FILE...")
        with open(output_file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                self.cx.append(float(row[0]))
                self.cy.append(float(row[1]))
                self.cyaw.append(float(row[2]))

        rospy.loginfo("LOADING FINISHED")

    def posePublish(self):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        # use planner, create cx, cy, cyaw

        for i in range(0, len(self.cx)):
            pose = Pose()

            quat = tf.transformations.quaternion_from_euler(0, 0, self.cyaw[i])

            pose.position.x = self.cx[i]
            pose.position.y = self.cy[i]
            pose.position.z = 0.0

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            msg.poses.append(pose)

        path_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("load_pose")

    load_pose = LoadPose()
    load_pose.readCSV()

    msg = PoseArray()

    path_pub = rospy.Publisher("stanley_path", PoseArray, queue_size=1)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        load_pose.posePublish()
        r.sleep()
