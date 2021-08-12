#!/usr/bin/env python

import rospy
import rospkg
import sys
import os
import threading
import csv
import tf
import math as m
import time as t
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    import cubic_spline_planner
except Exception as ex:
    print(ex)


class GetPose(object):
    def __init__(self):
        super(GetPose, self).__init__()

        self.deleteFlag = True

        self.initial_xs = []
        self.initial_ys = []

        self.Path = PoseArray()

        self.Path.header.seq = 0
        self.Path.header.frame_id = "map"

    def poseCallback(self, msg):
        """ 
            Subscribe initial pose
            add to initials 
        """

        temp_x = msg.pose.position.x
        temp_y = msg.pose.position.y

        self.initial_xs.append(temp_x)
        self.initial_ys.append(temp_y)

        rospy.loginfo("HELLO")

    def posePublish(self):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        # use planner, create cx, cy, cyaw

        cx = []
        cy = []
        cyaw = []

        try:
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
                self.initial_xs, self.initial_ys, ds=0.1)
        except Exception as ex:
            pass

        for i in range(0, len(cx)):
            pose = Pose()

            quat = tf.transformations.quaternion_from_euler(0, 0, cyaw[i])

            pose.position.x = cx[i]
            pose.position.y = cy[i]
            pose.position.z = 0.0

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            msg.poses.append(pose)

        path_pub.publish(msg)

    def savePoseArray(self):
        data = rospy.wait_for_message("/save_path", Empty)
        rospy.loginfo("TRYING TO SAVE PATH...")

        output_file_path = rospkg.RosPack().get_path(
            'path_planner')+"/saved_path/path.csv"

        with open(output_file_path, 'w') as csvfile:
            for pose in msg.poses:
                position_x = pose.position.x
                position_y = pose.position.y

                x = pose.orientation.x
                y = pose.orientation.y
                z = pose.orientation.z
                w = pose.orientation.w

                (_, _, yaw) = tf.transformations.euler_from_quaternion(
                    [x, y, z, w])

                csvfile.write(str(position_x) + "," +
                              str(position_y) + "," + str(yaw) + "\n")

        rospy.loginfo("SAVING FINISHED")

    def deleteOne(self):
        while not rospy.is_shutdown():
            if self.deleteFlag is True:
                rospy.wait_for_message("/delete_path", Empty)
                rospy.loginfo("DELETE LATEST POINT... PLZ WAIT")

                self.initial_xs.pop()
                self.initial_ys.pop()

                self.deleteFlag = False

                t.sleep(5.0)

                self.deleteFlag = True

                rospy.loginfo("SUCCESS")


if __name__ == "__main__":
    rospy.init_node("create_global_path")

    get_pose = GetPose()

    msg = PoseArray()

    path_pub = rospy.Publisher("create_global_path", PoseArray, queue_size=1)

    rospy.Subscriber(
        "/move_base_simple/goal", PoseStamped, get_pose.poseCallback)

    th1 = threading.Thread(target=get_pose.savePoseArray)
    th2 = threading.Thread(target=get_pose.deleteOne)
    th1.start()
    th2.start()

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        get_pose.posePublish()
        r.sleep()
