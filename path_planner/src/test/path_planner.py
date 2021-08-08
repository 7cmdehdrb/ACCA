#!/usr/bin/env python

import rospy
import rospkg
import tf
import csv
import math as m
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from dwa import Config, dwa_control


class PathPlanner(object):
    def __init__(self):
        super(PathPlanner, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.msg = Path()

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

        for i in range(0, len(self.cx)):
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            quat = tf.transformations.quaternion_from_euler(
                0, 0, self.cyaw[i])

            pose.pose.position.x = self.cx[i]
            pose.pose.position.y = self.cy[i]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            msg.poses.append(pose)

        pub.publish(msg)

    def publishLocalPath(self, paths, pub):
        msg = Path()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        if len(paths) == 1:
            rospy.loginfo("CANNOT FIND PATH...")
            return

        for path in paths:
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            quat = tf.transformations.quaternion_from_euler(0, 0, path[2])

            pose.pose.position.x = path[0]
            pose.pose.position.y = path[1]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            msg.poses.append(pose)

        pub.publish(msg)

    def publishTempGoal(self, pub, goal):
        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0, 0, goal[2])

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        pub.publish(msg)

    def calc_target_idx(self, state):
        # Search nearest point index
        dx = [state.x - icx for icx in self.cx]
        dy = [state.y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

    def getTempGoal(self, gap=15):
        last_idx = len(self.cx) - 1
        target_idx = self.calc_target_idx(state=state) + gap

        if target_idx > last_idx:
            target_idx = last_idx - 1

        return np.array([self.cx[target_idx], self.cy[target_idx], self.cyaw[target_idx]])


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
        self.last_yaw = yaw

        self.v = v
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)
        self.dyaw = 0.0

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
        self.dyaw = (self.yaw - self.last_yaw) / self.dt

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw

        self.lastTime = rospy.Time.now()

    def getX(self):
        return [self.x, self.y, self.yaw, self.v, self.dyaw]


if __name__ == "__main__":
    rospy.init_node("path_planner")

    config = Config()
    state = State(x=0.0, y=0.0, yaw=m.radians(0.0), v=0.0)
    path_planner = PathPlanner()
    path_planner.readCSV()

    rospy.Subscriber("/odom", Odometry, state.odometryCallback)

    global_path_pub = rospy.Publisher("cublic_global_path", Path, queue_size=1)
    local_path_pub = rospy.Publisher("dwa_local_path", Path, queue_size=1)
    # goal_pub = rospy.Publisher("temp_goal", PoseStamped, queue_size=1)

    x = state.getX()
    goal = path_planner.getTempGoal()   # [x, y, yaw]
    ob = config.ob

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        goal = path_planner.getTempGoal(gap=100)

        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = state.getX()  # simulate robot

        path_planner.publishGlobalPath(pub=global_path_pub)
        path_planner.publishLocalPath(
            paths=predicted_trajectory, pub=local_path_pub)
        # path_planner.publishTempGoal(pub=goal_pub, goal=goal)

        r.sleep()
