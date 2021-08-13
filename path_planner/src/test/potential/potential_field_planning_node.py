#!/usr/bin/env python


import rospy
import sys
import tf
import math as m
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from path_planner.msg import stanleyMsg
from potential_field_planning import *
from loadPose import LoadPose

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    import cubic_spline_planner
    from pure_pursuit import TargetCourse, pure_pursuit_steer_control
except Exception as ex:
    print(ex)


grid_size = 0.5  # potential grid size [m]
robot_radius = 2.0  # robot radius [m]
WB = 1.040
desired_speed = rospy.get_param("/desired_speed", 2.0)  # kph


class PurePursuitState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(PurePursuitState, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.rear_x = 0.0
        self.rear_y = 0.0

    def updateRear(self):
        self.rear_x = self.x - ((WB / 2) * m.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * m.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return m.hypot(dx, dy)


class PotentialFieldPlanning(object):
    def __init__(self):
        super(PotentialFieldPlanning, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_idx = 0

        self.obstacle_x = [0.0]
        self.obstacle_y = [0.0]

    def calc_target_idx(self, state):
        # Search nearest point index
        dx = [state.x - icx for icx in self.cx]
        dy = [state.y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

    def publishPath(self, publisher, rx, ry):
        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = []

        for i in range(len(rx) - 1):
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            pose.pose.position.x = rx[i]
            pose.pose.position.y = ry[i]
            pose.pose.position.z = 0.0

            yaw = m.atan2((rx[i + 1] - rx[i]), (ry[i + 1] - ry[i]))

            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)

        publisher.publish(path)

    def getTempGoal(self, state, gap=100):
        last_idx = len(self.cx) - 1
        target_idx = self.calc_target_idx(state=state) + gap

        if target_idx > last_idx:
            target_idx = last_idx - 1

        return self.cx[target_idx], self.cy[target_idx]


if __name__ == '__main__':
    rospy.init_node("potential_field_planning")

    potential_planning = PotentialFieldPlanning()
    state = PurePursuitState(x=0.0, y=0.0, yaw=0.0, v=0.0)
    load = LoadPose()

    cmd_msg = stanleyMsg()

    potential_planning.cx = load.cx
    potential_planning.cy = load.cy
    potential_planning.cyaw = load.cyaw

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    path_pub = rospy.Publisher(
        "/potential_field_planning", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/potential_stanley_cmd",
                              stanleyMsg, queue_size=1)

    target_idx = 1

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        ox = potential_planning.obstacle_x
        oy = potential_planning.obstacle_y

        state.updateRear()

        gx, gy = potential_planning.getTempGoal(state=state, gap=200)

        rx, ry = potential_field_planning(
            state.x, state.y, gx, gy, ox, oy, grid_size, robot_radius)

        target_course = TargetCourse(rx, ry)

        di, target_idx = pure_pursuit_steer_control(
            state, target_course, target_idx
        )

        cmd_msg.speed = desired_speed
        cmd_msg.steer = di
        cmd_msg.brake = 1

        cmd_pub.publish(cmd_msg)

        print(cmd_msg)

        potential_planning.publishPath(
            publisher=path_pub, rx=rx, ry=ry)

        r.sleep()
