#!/usr/bin/env python

import rospy
import rospkg
import tf
import math as m
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry


class PathPlanner(object):
    def __init__(self):
        super(PathPlanner, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []

    def readCSV(self):
        output_file_path = rospkg.RosPack().get_path(
            'path_planner')+"/saved_path/path.csv"

        rospy.loginfo("LOADING CSV FILE...")

        with open(output_file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                self.cx.append(float(row[0]))
                self.cy.append(float(row[1]))
                self.cyaw.append(float(row[2]))

        rospy.loginfo("LOADING FINISHED")

    def publishGlobalPath(self, pub):
        msg = Path()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        for i in range(0, len(self.cx)):
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            quat = tf.transformations.quaternion_from_euler(0, 0, self.cyaw[i])

            pose.pose.position.x = self.cx[i]
            pose.pose.position.y = self.cy[i]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            msg.poses.append(pose)

        pub.publish(msg)

    def calc_target_idx(self, state):
        # Search nearest point index
        dx = [state.x - icx for icx in self.cx]
        dy = [state.y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx


class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()

        self.data = Odometry()

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.dt = 0.1

        self.x = x
        self.y = y
        self.yaw = yaw

        self.last_x = x
        self.last_y = y

        self.v = v
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)

    def odometryCallback(self, msg):
        self.data = msg

        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        if self.dt == 0.0:
            return

        self.x = self.data.pose.pose.position.x
        self.y = self.data.pose.pose.position.y

        x = self.data.pose.pose.orientation.x
        y = self.data.pose.pose.orientation.y
        z = self.data.pose.pose.orientation.z
        w = self.data.pose.pose.orientation.w

        quaternion_array = [x, y, z, w]

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion_array)

        self.yaw = yaw

        self.dx = (self.x - self.last_x) / self.dt
        self.dy = (self.x - self.last_y) / self.dt
        self.v = m.sqrt((self.dx ** 2) + (self.dy ** 2))

        self.last_x = self.x
        self.last_y = self.y
        self.lastTime = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("path_planner")

    state = State(x=0.0, y=0.0, yaw=m.radians(0.0), v=0.0)
    path_planner = PathPlanner()
    path_planner.readCSV()

    rospy.Subscriber("/odom", Odometry, state.odometryCallback)

    global_path_pub = rospy.Publisher("stanley_path", Path, queue_size=1)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        path_planner.publishGlobalPath(pub=global_path_pub)
        r.sleep()
