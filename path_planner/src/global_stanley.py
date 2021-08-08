#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from loadPose import LoadPose


k = 100.0  # control gain
L = 1.040  # [m] Wheel base of vehicle
max_steer = np.radians(25.0)  # [rad] max steering angle


class PathFinder(object):
    def __init__(self):
        super(PathFinder, self).__init__()
        self.counter = True

        self.path = Path()

        self.cx = []
        self.cy = []
        self.cyaw = []


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
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

    def odometryCallback(self, msg):
        self.data = msg

        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        if self.dt == 0:
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
        self.dyaw = (self.yaw - self.last_yaw) / self.dt
        self.v = m.sqrt((self.dx ** 2) + (self.dy ** 2))

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw
        self.lastTime = rospy.Time.now()


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)

    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d
    # print(theta_d)

    return delta, current_target_idx


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


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -
                      np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


if __name__ == "__main__":
    rospy.init_node("stanley_method")

    # Initial state
    load = LoadPose()
    load.readCSV()

    state = State(x=-0.0, y=0.0, yaw=np.radians(180.0), v=0.0)
    path = PathFinder()

    cmd_msg = Twist()

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    cmd_pub = rospy.Publisher("/stanley_cmd", Twist, queue_size=1)
    path_pub = rospy.Publisher("/cublic_global_path", Path, queue_size=1)

    desired_speed = 5.0  # kph

    path.cx = load.cx
    path.cy = load.cy
    path.cyaw = load.cyaw

    last_idx = len(path.cx) - 1

    target_idx, _ = calc_target_index(state, path.cx, path.cy)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        di, target_idx = stanley_control(
            state, path.cx, path.cy, path.cyaw, target_idx)

        di = np.clip(di, -m.radians(30.0), m.radians(30.0))

        cmd_msg.linear.x = desired_speed if last_idx != target_idx else 0.0
        cmd_msg.linear.y = 0.0
        cmd_msg.linear.z = 0.0

        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = -di

        cmd_pub.publish(cmd_msg)

        rospy.loginfo((-m.degrees(di), target_idx))

        load.pathPublish(pub=path_pub)
        r.sleep()
