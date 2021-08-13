#!/usr/bin/env python

import rospy
import rospkg
import sys
import os
import tf
import math as m
import numpy as np
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Empty


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    import cubic_spline_planner
except Exception as ex:
    print(ex)


class OdometryPath(State):
    def __init__(self, x, y, yaw, v):
        super(OdometryPath, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.xs = []
        self.ys = []

    def update(self):
        if self.x == 0.0 and self.y == 0.0:
            rospy.loginfo("WAIT FOR INITIALIZE")
            return

        if len(self.xs) == 0 and (self.x != 0.0 or self.y != 0.0):
            self.xs.append(self.x)
            self.ys.append(self.y)
            return

        distance = np.hypot(self.xs[-1] - self.x, self.ys[-1] - self.y)

        if distance > 1.0:
            self.xs.append(self.x)
            self.ys.append(self.y)

    def publishPath(self, publisher):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        try:
            cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(
                x=self.xs, y=self.ys, ds=0.1
            )
        except Exception as ex:
            return

        for i in range(len(cx)):
            pose = Pose()

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, cyaw[i])

            pose.position.x = cx[i]
            pose.position.y = cy[i]
            pose.position.z = 0.0

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            msg.poses.append(pose)

        publisher.publish(msg)

    def savePoseArray(self):
        data = rospy.wait_for_message("/save_path", Empty)
        rospy.loginfo("TRYING TO SAVE PATH...")

        output_file_path = rospkg.RosPack().get_path(
            'path_planner')+"/saved_path/odometry_path.csv"

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


if __name__ == "__main__":
    rospy.init_node("odometry_path_generator")

    msg = PoseArray()

    state = OdometryPath(x=0.0, y=0.0, yaw=0.0, v=0.0)

    path_pub = rospy.Publisher("create_global_path", PoseArray, queue_size=1)

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    th = threading.Thread(target=state.savePoseArray)
    th.start()

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        state.update()
        state.publishPath(publisher=path_pub)
        r.sleep()
