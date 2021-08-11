#!/usr/bin/env python


import rospy
import rospkg
import tf
import math as m
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from path_planner.msg import stanleyMsg
from potential_field_planning import *
import cubic_spline_planner
from stanley import *
from loadPose import LoadPose
from obstacle_detection import Obstacle

grid_size = 0.5  # potential grid size [m]
robot_radius = 2.0  # robot radius [m]
desired_speed = rospy.get_param("/desired_speed", 2.0)  # kph


class PotentialFieldPlanning(object):
    def __init__(self):
        super(PotentialFieldPlanning, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_idx = 0

        self.obstacle_x = [0]
        self.obstacle_y = [0]

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

    def obstacleCallback(self, msg):
        poses = msg.poses

        temp_xs = []
        temp_ys = []

        for pose in poses:
            temp_xs.append(pose.position.x)
            temp_ys.append(pose.position.y)

        if len(temp_xs) == len(temp_ys):
            self.obstacle_x = temp_xs
            self.obstacle_y = temp_ys

            if len(temp_xs) < 1:
                self.obstacle_x = [0]
                self.obstacle_y = [0]


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
    rospy.Subscriber("/obstacles", PoseArray,
                     potential_planning.obstacleCallback)

    path_pub = rospy.Publisher(
        "/potential_field_planning", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/potential_stanley_cmd",
                              stanleyMsg, queue_size=1)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        target_idx = 0

        ox = potential_planning.obstacle_x
        oy = potential_planning.obstacle_y

        sx = state.x
        sy = state.y

        gx, gy = potential_planning.getTempGoal(state=state, gap=50)

        rx, ry = potential_field_planning(
            sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(
            rx, ry, ds=0.1)

        potential_planning.publishPath(
            publisher=path_pub, rx=cx, ry=cy, ryaw=cyaw)

        # while True:
        #     distance = np.hypot(gx - state.x, gy - state.y)

        #     if distance < 3.0:
        #         break

        #     di, target_idx = stanley_control(
        #         state, cx, cy, cyaw, target_idx
        #     )

        #     di = np.clip(di, -m.radians(30.0), m.radians(30.0))

        #     cmd_msg.speed = desired_speed
        #     cmd_msg.steer = -di
        #     cmd_msg.brake = 1

        #     cmd_pub.publish(cmd_msg)

        #     potential_planning.publishPath(
        #         publisher=path_pub, rx=cx, ry=cy, ryaw=cyaw)

        #     r.sleep()

        r.sleep()
