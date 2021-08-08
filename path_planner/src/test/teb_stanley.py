#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist


k = 1.0  # control gain
L = 1.040  # [m] Wheel base of vehicle
max_steer = np.radians(25.0)  # [rad] max steering angle


class PathPlanner(object):
    def __init__(self):
        super(PathPlanner, self).__init__()

        self.localPath = Path()

        self.cx = []
        self.cy = []
        self.cyaw = []

    def localPathCallback(self, msg):
        self.localPath = msg

        self.cx = []
        self.cy = []
        self.cyaw = []

        poses = self.localPath.poses

        for pose in poses:

            x = pose.pose.orientation.x
            y = pose.pose.orientation.y
            z = pose.pose.orientation.z
            w = pose.pose.orientation.w

            (_, _, yaw) = tf.transformations.euler_from_quaternion(
                [x, y, z, w])

            self.cx.append(pose.pose.position.x)
            self.cy.append(pose.pose.position.y)
            self.cyaw.append(yaw)


class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()

        self.data = Odometry()

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.last_x = x
        self.last_y = y

        self.dt = 0.1

        self.x = x
        self.y = y
        self.yaw = yaw

        self.v = v
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)

    def odometryCallback(self, msg):
        self.data = msg

        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        self.x = self.data.pose.pose.position.x
        self.y = self.data.pose.pose.position.y

        x = self.data.pose.pose.orientation.x
        y = self.data.pose.pose.orientation.y
        z = self.data.pose.pose.orientation.z
        w = self.data.pose.pose.orientation.w

        quaternion_array = [x, y, z, w]

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion_array)

        self.yaw = yaw

        try:

            self.dx = (self.x - self.last_x) / self.dt
            self.dy = (self.x - self.last_y) / self.dt
            self.v = m.sqrt((self.dx ** 2) + (self.dy ** 2))

        except ZeroDivisionError:
            pass
        except Exception as ex:
            print(ex)

        steer = stanley_control(state=self, path=path)

        rospy.loginfo(m.degrees(steer))

        self.last_x = self.x
        self.last_y = self.y
        self.lastTime = rospy.Time.now()


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


def stanley_control(state, path):

    if len(path.cx) == 0 or len(path.cy) == 0 or len(path.cyaw) == 0:
        return 0.0

    current_target_idx, error_front_axle = calc_error(state, path.cx, path.cy)

    # theta_e corrects the heading error
    theta_e = normalize_angle(path.cyaw[current_target_idx] - state.yaw)

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
    rospy.init_node("teb_stanley")

    state = State(x=0.0, y=0.0, yaw=np.radians(0.0), v=0.0)
    path = PathPlanner()

    rospy.Subscriber("/odom", Odometry, state.odometryCallback)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan",
                     Path, path.localPathCallback)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
