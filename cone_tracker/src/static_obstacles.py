#!/usr/bin/env python


import sys
import tf
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    from util_class import Obstacle, Map
    from cubic_spline_planner import calc_spline_course
except Exception as ex:
    print("UTIL CLASS IMPORT ERROR")
    print(ex)


LANE_YAW = rospy.get_param("/LANE_YAW", 0.0)
DISTANCE_GAP = rospy.get_param("/DISTANCE_GAP", 1.0)


class StaticObstacles(object):

    """

    Subscribe '/obstacles' and transform into odom frame
    Then, append them to Map

    """

    def __init__(self, map, state, tf_node):
        super(StaticObstacles, self).__init__()

        self.map = map
        self.state = state
        self.tf_node = tf_node

        self.start_point = None
        self.last_goal = None

        self.doPublishing = True

        self.path_pub = rospy.Publisher(
            "static_obstacle_path", Path, queue_size=1)

    def checkObstacle(self, new_ob):
        min_dis = float("inf")

        for obstacle in self.map.obstacles:
            distance = np.hypot(obstacle.x - new_ob.x, obstacle.y - new_ob.y)

            if distance < min_dis:
                min_dis = distance

        if min_dis > 0.1:
            return True

        return False

    def obstaclesCallback(self, msg):
        temp = []

        poses = msg.poses

        try:
            for pose in poses:
                new_obstacle = PoseStamped()

                new_obstacle.header.frame_id = "laser"
                new_obstacle.header.stamp = rospy.Time(0)

                new_obstacle.pose = pose

                new_obstacle = self.tf_node.transformPose("odom", new_obstacle)

                new_obstacle = Obstacle(
                    x=new_obstacle.pose.position.x, y=new_obstacle.pose.position.y)

                if self.checkObstacle(new_obstacle) is True:
                    self.map.obstacles.append(new_obstacle)

        except Exception as ex:
            pass

    def getNearestObstacle(self):
        nearest_obstacle = None
        is_right = None

        min_dis = float("inf")

        path_VEC = np.array([
            m.cos(LANE_YAW), m.sin(LANE_YAW)
        ])

        for obstacle in self.map.obstacles:

            obstacle_VEC = np.array([
                obstacle.x - self.state.x, obstacle.y - self.state.y
            ])

            dot = np.dot(obstacle_VEC, path_VEC)

            # If obstacle is located in FRONT
            if dot > 0.0:
                distance = np.hypot(obstacle.x - self.state.x,
                                    obstacle.y - self.state.y)

                if distance < min_dis:
                    min_dis = distance

                    cross = np.cross(obstacle_VEC, path_VEC)

                    nearest_obstacle = obstacle
                    is_right = True if cross >= 0.0 else False

        return nearest_obstacle, is_right

    def addFrame(self, frame, obstacle):
        temp_ob = PoseStamped()

        temp_ob.header.frame_id = frame
        temp_ob.header.stamp = rospy.Time(0)

        temp_ob.pose.position.x = obstacle.x
        temp_ob.pose.position.y = obstacle.y

        temp_ob.pose.orientation.w = 1.0

        return temp_ob

    def createGoalPoint(self):
        nearest_obstacle, is_right = self.getNearestObstacle()

        if nearest_obstacle is None:
            return None

        odom_obstacle = self.addFrame("odom", nearest_obstacle)

        laser_obstacle = self.tf_node.transformPose("laser", odom_obstacle)

        if is_right is True:
            laser_obstacle.pose.position.x - DISTANCE_GAP
        else:
            laser_obstacle.pose.position.x + DISTANCE_GAP

        return self.tf_node.transformPose("odom", laser_obstacle)

    def createPath(self):
        goal_point = self.createGoalPoint()

        if goal_point is not None:
            xs = [self.state.x, goal_point.pose.position.x]
            ys = [self.state.y, goal_point.pose.position.y]

            cx, cy, cyaw, _, _ = calc_spline_course(x=xs, y=ys, ds=0.1)

            if self.doPublishing is True:
                self.publishPath(cx, cy, cyaw)

            return cx, cy, cyaw

        return None

    def publishPath(self, cx, cy, cyaw):
        msg = Path()

        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()

        msg.poses = []

        for i in range(len(cx)):
            pose = PoseStamped()

            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = cx[i]
            pose.pose.position.y = cy[i]

            quat = tf.transformations.quaternion_from_euler(
                0.0, 0.0, cyaw[i])

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            msg.poses.append(pose)

        self.path_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("static_obstacles")

    map = Map()
    state = State(0.0, 0.0, 0.0, 0.0)
    tf_node = tf.TransformListener()

    static_ob = StaticObstacles(map=map, state=state, tf_node=tf_node)

    test_pub = rospy.Publisher("test_goal", PoseStamped, queue_size=1)

    rospy.Subscriber("/obstacles", PoseArray, static_ob.obstaclesCallback)
    rospy.Subscriber("/odometry/imu", Odometry,
                     callback=state.odometryCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        goal_point = static_ob.createGoalPoint()

        # TEST

        if goal_point is None:
            print("NO GOAL POINT. PLEASE USE GLOBAL PATH")
        else:
            test_pub.publish(goal_point)
            print(goal_point.pose.position.x, goal_point.pose.position.y)

        r.sleep()
