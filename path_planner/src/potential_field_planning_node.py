#!/usr/bin/env python


import rospy
import rospkg
import tf
import math as m
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from path_planner.msg import stanleyMsg
from potential_field_planning import *
from stanley_control import *
import cubic_spline_planner
from loadPose import LoadPose

grid_size = 0.5  # potential grid size [m]
robot_radius = 2.0  # robot radius [m]

k = rospy.get_param("/c_gain", 30.0)  # control gain
desired_speed = rospy.get_param("/desired_speed", 2.0)  # kph
L = 1.040  # [m] Wheel base of vehicle


class PotentialFieldPlanning(object):
    def __init__(self):
        super(PotentialFieldPlanning, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_idx = 0

    def calc_target_idx(self, state):
        # Search nearest point index
        dx = [state.x - icx for icx in self.cx]
        dy = [state.y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

    def getTempGoal(self, state, gap=100):
        last_idx = len(self.cx) - 1
        target_idx = self.calc_target_idx(state=state) + gap

        if target_idx > last_idx:
            target_idx = last_idx - 1

        return self.cx[target_idx], self.cy[target_idx]

    def publishPath(self, publisher, rx, ry, ryaw):
        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = []

        for i in range(0, len(rx)):
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            pose.pose.position.x = rx[i]
            pose.pose.position.y = ry[i]
            pose.pose.position.z = 0.0

            # yaw = m.atan2(rx[i + 1] - rx[i], ry[i + 1] - ry[i])
            # quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            quat = tf.transformations.quaternion_from_euler(0, 0, ryaw[i])

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)

        publisher.publish(path)


class State(object):
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


if __name__ == '__main__':
    rospy.init_node("potential_field_planning")

    potential_planning = PotentialFieldPlanning()
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    load = LoadPose()

    cmd_msg = stanleyMsg()

    load.readCSV()

    potential_planning.cx = load.cx
    potential_planning.cy = load.cy
    potential_planning.cyaw = load.cyaw

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    path_pub = rospy.Publisher(
        "/potential_field_planning", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/potential_stanley_cmd",
                              stanleyMsg, queue_size=1)

    ox = [100.0]  # obstacle x position list [m]
    oy = [100.0]  # obstacle y position list [m]

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        target_idx = 0

        sx = state.x
        sy = state.y

        gx, gy = potential_planning.getTempGoal(state=state, gap=50)

        rx, ry = potential_field_planning(
            sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(
            rx, ry, ds=0.1)

        while True:
            distance = np.hypot(gx - state.x, gy - state.y)

            if distance < 3.0:
                break

            di, target_idx = stanley_control(
                state, cx, cy, cyaw, target_idx
            )

            di = np.clip(di, -m.radians(30.0), m.radians(30.0))

            cmd_msg.speed = desired_speed
            cmd_msg.steer = -di
            cmd_msg.brake = 1

            cmd_pub.publish(cmd_msg)

            potential_planning.publishPath(
                publisher=path_pub, rx=cx, ry=cy, ryaw=cyaw)

        r.sleep()
