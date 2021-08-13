#!/usr/bin/env python


import rospy
import sys
import tf
import math as m
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from path_planner.msg import stanleyMsg
from loadPose import LoadPose
from rrt_star import RRTStar

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    import cubic_spline_planner
    from pure_pursuit import pure_pursuit_control
except Exception as ex:
    print(ex)


desired_speed = rospy.get_param("/desired_speed", 2.0)  # kph
WB = 1.040


class RRTStarState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(RRTStarState, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.rear_x = 0.0
        self.rear_y = 0.0

    def updateRear(self):
        self.rear_x = self.x - ((WB / 2) * m.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * m.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return m.hypot(dx, dy)


class RRTStarPath(object):
    def __init__(self, state):
        super(RRTStarPath, self).__init__()

        load = LoadPose()

        self.state = state

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw
        self.target_idx = 0

    def calc_target_idx(self):
        # Search nearest point index
        dx = [self.state.x - icx for icx in self.cx]
        dy = [self.state.y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

    def getTempGoal(self, gap=100):
        last_idx = len(self.cx) - 1
        target_idx = self.calc_target_idx() + gap

        if target_idx > last_idx:
            target_idx = last_idx - 1

        return self.cx[target_idx], self.cy[target_idx]

    def reversePath(self, rrt_path):
        result = []

        for i in range(len(rrt_path) - 1, 0, -1):
            result.append(rrt_path[i])

        return result

    def dividePath(self, rrt_path):
        cx = []
        cy = []

        for i in range(len(rrt_path)):
            cx.append(rrt_path[i][0])
            cy.append(rrt_path[i][1])

        return cx, cy

    def publishPath(self, publisher, rrt_path):
        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = []

        for i in range(len(rrt_path)):
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            pose.pose.position.x = rrt_path[i][0]
            pose.pose.position.y = rrt_path[i][1]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        publisher.publish(path)


if __name__ == '__main__':
    rospy.init_node("rrt_star_planning")

    state = RRTStarState(x=0.0, y=0.0, yaw=0.0, v=0.0)
    rrt_star_path = RRTStarPath(state=state)

    cmd_msg = stanleyMsg()

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    path_pub = rospy.Publisher(
        "/rrt_star_path", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/rrt_star_cmd",
                              stanleyMsg, queue_size=1)

    obstacle_list = [
        (10.0, 10.0, 1.0)
    ]

    target_idx = 1

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        state.updateRear()

        rrt_star = RRTStar(
            start=[state.x, state.y],
            goal=rrt_star_path.getTempGoal(gap=70),
            rand_area=[[state.x - 5, state.x + 5],
                       [state.y - 5, state.y + 5]],
            obstacle_list=obstacle_list,
            expand_dis=3,
            max_iter=300
        )

        path = rrt_star.planning()

        if path is None:
            print("CAN NOT FIND PATH")
        else:
            path = rrt_star_path.reversePath(path)

            # middle = int(len(path) / 2)

            goal = path[-1]

            steer = pure_pursuit_control(state, goal)

            cmd_msg.speed = desired_speed
            cmd_msg.steer = -steer
            cmd_msg.brake = 1

            cmd_pub.publish(cmd_msg)

            print(cmd_msg)

            rrt_star_path.publishPath(publisher=path_pub, rrt_path=path)

        r.sleep()
