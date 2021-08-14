#!/usr/bin/env python


import rospy
import sys
import tf
import math as m
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
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


desired_speed = rospy.get_param("/desired_speed", 1.0)  # kph
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

        self.path = None

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw
        self.target_idx = 0

    def calc_pure_target_idx(self, cx, cy):
        dx = [self.state.x - icx for icx in cx]
        dy = [self.state.y - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return target_idx

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

    def publishMarker(self, obstacles, publisher):
        msg = MarkerArray()

        for obstacle in obstacles:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.lifetime = rospy.Duration(0.2)
            marker.ns = str(obstacle[0])

            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.scale.x = obstacle[2] / 2
            marker.scale.y = obstacle[2] / 2
            marker.scale.z = 0.1

            marker.color.a = 0.5
            marker.color.r = 1.0

            marker.pose.position.x = obstacle[0]
            marker.pose.position.y = obstacle[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            msg.markers.append(marker)

        publisher.publish(msg)

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
    obstacle_pub = rospy.Publisher(
        "obstacles", MarkerArray, queue_size=1)

    obstacle_list = [
        (-13.4, -17.1, 1.5),
        (-21.0, -25.5, 1.5),
        (-38.5, -36.1, 1.5),
    ]

    target_idx = 1

    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():

        state.updateRear()

        rrt_star = RRTStar(
            start=[state.x, state.y],
            goal=rrt_star_path.getTempGoal(gap=100),
            rand_area=[[state.x - 10, state.x + 10],
                       [state.y - 10, state.y + 10]],
            obstacle_list=obstacle_list,
            expand_dis=2.0,
            max_iter=700
        )

        path = rrt_star.planning()

        if path is not None:
            path = rrt_star_path.reversePath(path)
            rrt_star_path.path = path

        else:
            path = rrt_star_path.path

        cx, cy = rrt_star_path.dividePath(path)

        target_idx = rrt_star_path.calc_pure_target_idx(cx, cy)

        goal = path[target_idx + 1]

        steer = pure_pursuit_control(state, goal)

        cmd_msg.speed = desired_speed
        cmd_msg.steer = -steer * 2.0
        cmd_msg.brake = 1

        cmd_pub.publish(cmd_msg)

        rrt_star_path.publishPath(
            publisher=path_pub, rrt_path=rrt_star_path.path)

        rrt_star_path.publishMarker(
            publisher=obstacle_pub, obstacles=obstacle_list)

        r.sleep()
