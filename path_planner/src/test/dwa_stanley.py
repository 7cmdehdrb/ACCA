#!/usr/bin/env python

import rospy
import rospkg
import tf
import csv
import math as m
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from loadPose import LoadPose
from dwa import Config, dwa_control


k = 2.0  # control gain
L = 1.040  # [m] Wheel base of vehicle
max_steer = np.radians(25.0)  # [rad] max steering angle


class PathFinder(object):
    def __init__(self):
        super(PathFinder, self).__init__()

        self.path = Path()

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.di = 0.0

    def publishLocalPath(self, paths, pub):
        if len(paths) == 1:
            rospy.loginfo("CANNOT FIND PATH...")
            pub.publish(self.path)
            return

        msg = Path()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        poses_temp = []
        x_temp = []
        y_temp = []
        yaw_temp = []

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

            x_temp.append(path[0])
            y_temp.append(path[1])
            yaw_temp.append(path[2])

            poses_temp.append(pose)

        msg.poses = poses_temp

        di = stanley_control(
            state=state, cx=x_temp, cy=y_temp, cyaw=yaw_temp)

        self.path = msg
        self.di = np.clip(di, -m.radians(30.0), m.radians(30.0))

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


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def stanley_control(state, cx, cy, cyaw):

    if len(cx) == 0 or len(cy) == 0 or len(cyaw) == 0:
        return 0.0

    current_target_idx, error_front_axle = calc_error(state, cx, cy)

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[-1] - state.yaw)

    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta


def calc_error(state, cx, cy):
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    # dx = [fx - icx for icx in cx]
    # dy = [fy - icy for icy in cy]

    dx = cx[-1]
    dy = cy[-1]

    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -
                      np.sin(state.yaw + np.pi / 2)]
    # error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    error_front_axle = np.dot([dx, dy], front_axle_vec)

    return target_idx, error_front_axle


if __name__ == "__main__":
    rospy.init_node("dwa_stanley_method")

    load = LoadPose()
    load.readCSV()

    state = State(x=0.0, y=0.0, yaw=m.radians(0.0), v=0.0)
    path = PathFinder()
    config = Config()

    cmd_msg = Twist()

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    cmd_pub = rospy.Publisher("/dwa_stanley_cmd", Twist, queue_size=1)
    path_pub = rospy.Publisher("/dwa_local_path", Path, queue_size=1)

    desired_speed = 1.0  # kph

    path.cx = load.cx
    path.cy = load.cy
    path.cyaw = load.cyaw

    x = state.getX()
    goal = path.getTempGoal()   # [x, y, yaw]
    ob = config.ob

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        goal = path.getTempGoal(gap=50)
        rospy.loginfo(goal)

        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = state.getX()

        path.publishLocalPath(
            paths=predicted_trajectory, pub=path_pub)

        cmd_msg.linear.x = desired_speed
        cmd_msg.linear.y = 0.0
        cmd_msg.linear.z = 0.0

        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = -path.di

        cmd_pub.publish(cmd_msg)

        rospy.loginfo(-m.degrees(path.di))

        r.sleep()
