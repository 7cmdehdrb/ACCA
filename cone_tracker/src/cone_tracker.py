#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from geometry_msgs.msg import PoseStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from path_planner.msg import stanleyMsg
import cubic_spline_planner


desired_speed = rospy.get_param("/desired_speed", 3.0)  # kph
max_steer = rospy.get_param("/max_steer", 30.0)


k = rospy.get_param("/c_gain", 30.0)  # control gain
L = 1.040  # [m] Wheel base of vehicle


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


class PathDetection(object):
    def __init__(self):
        super(PathDetection, self).__init__()

        self.paths = [[0.0, 0.0]]
        self.x_paths = [0.0]
        self.y_paths = [0.0]

        self.final_path = Path()

    def poseCallback(self, msg):
        data = msg

        x = data.pose.position.x
        y = data.pose.position.y

        new_path_position = [x, y]

        distance = np.hypot(
            abs(self.paths[-1][0] - new_path_position[0]), abs(self.paths[-1][1] - new_path_position[1]))

        if abs(distance) > 0.05:
            self.paths.append(new_path_position)
            self.x_paths.append(new_path_position[0])
            self.y_paths.append(new_path_position[1])

    def publishPath(self, cx, cy, cyaw, publisher):
        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = []

        for i in range(0, len(cx) - 1):
            temp_pose = PoseStamped()

            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.header.frame_id = "map"

            temp_pose.pose.position.x = cx[i]
            temp_pose.pose.position.y = cy[i]
            temp_pose.pose.position.z = 0.0

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, cyaw[i])

            temp_pose.pose.orientation.x = quat[0]
            temp_pose.pose.orientation.y = quat[1]
            temp_pose.pose.orientation.z = quat[2]
            temp_pose.pose.orientation.w = quat[3]

            path.poses.append(temp_pose)

        self.final_path = path
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
        self.v = np.hypot(self.dx, self.dy)

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw
        self.lastTime = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    # Object
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    path = PathDetection()
    cmd_msg = stanleyMsg()

    rospy.Subscriber("/odom", Odometry, state.odometryCallback)
    rospy.Subscriber("/transformed_cone_position",
                     PoseStamped, path.poseCallback)

    path_pub = rospy.Publisher("/cone_path", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/cone_stanley_cmd", stanleyMsg, queue_size=1)

    target_idx = 0

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():

        if len(path.x_paths) > 1:

            cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(
                x=path.x_paths, y=path.y_paths, ds=0.1
            )

            di, target_idx = stanley_control(
                state, cx, cy, cyaw, target_idx
            )

            cmd_msg.speed = desired_speed
            cmd_msg.steer = - \
                np.clip(di, -m.radians(max_steer), m.radians(max_steer))
            cmd_msg.brake = 1

            cmd_pub.publish(cmd_msg)
            path.publishPath(cx=cx, cy=cy, cyaw=cyaw, publisher=path_pub)

        r.sleep()
