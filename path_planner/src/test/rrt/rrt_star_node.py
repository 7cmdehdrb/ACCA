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
k_gain = rospy.get_param("/k_gain", 2.0)
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
    def __init__(self, state, cmd_msg, cmd_publisher):
        super(RRTStarPath, self).__init__()

        load = LoadPose()

        self.state = state

        self.obstacle_list = []

        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_publisher

        self.path_pub = rospy.Publisher(
            "/rrt_star_path", Path, queue_size=1)

        self.obstacle_pub = rospy.Publisher(
            "obstacles", MarkerArray, queue_size=1)

        self.path = None

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw

        self.target_idx = 1

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

    def obstacleCallback(self, msg, radius=1.0):
        data = msg

        obstacles = data.poses

        result = []

        for obstacle in obstacles:
            x = obstacle.position.x
            y = obstacle.position.y
            r = radius

            result.append((x, y, r))

        self.obstacle_list = result

    def main(self):
        self.state.updateRear()

        rrt_star = RRTStar(
            start=[self.state.x, self.state.y],
            goal=self.getTempGoal(gap=100),
            rand_area=[[self.state.x - 10, self.state.x + 10],
                       [self.state.y - 10, self.state.y + 10]],
            obstacle_list=self.obstacle_list,
            expand_dis=2.0,
            max_iter=700
        )

        path = rrt_star.planning()

        if path is not None:
            path = self.reversePath(path)
            self.path = path

        else:
            path = self.path

        cx, cy = self.dividePath(path)

        self.target_idx = self.calc_pure_target_idx(cx, cy)

        goal = path[self.target_idx + 1]

        steer = pure_pursuit_control(state, goal)

        self.cmd_msg.speed = desired_speed
        self.cmd_msg.steer = -steer * k_gain
        self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)

        self.publishPath(
            publisher=self.path_pub, rrt_path=self.path)

        self.publishMarker(
            publisher=self.obstacle_pub, obstacles=self.obstacle_list)


if __name__ == '__main__':
    rospy.init_node("rrt_star_planning")

    cmd_msg = stanleyMsg()
    cmd_pub = rospy.Publisher("/rrt_star_cmd",
                              stanleyMsg, queue_size=1)

    state = RRTStarState(x=0.0, y=0.0, yaw=0.0, v=0.0)
    rrt_star_path = RRTStarPath(
        state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub)

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    rospy.Subscriber("/transformed_obstacles", PoseArray,
                     rrt_star_path.obstacleCallback)

    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        rrt_star_path.main()
        r.sleep()
