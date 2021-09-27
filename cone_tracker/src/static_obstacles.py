#!/usr/bin/env python


import sys
import tf
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from path_planner.msg import stanleyMsg
from visualization_msgs.msg import Marker


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")


_, _, YAW = tf.transformations.euler_from_quaternion([0.0, 0.0, 0.921844102877, 0.387560898429,
                                                      ])

LANE_YAW = YAW
DISTANCE_GAP = rospy.get_param("/distance_gap", 1.0)

desired_speed = rospy.get_param("/static_obstacles_speed", 5.0)  # KPH
max_steer = rospy.get_param("/max_steer", 30.0)  # DEG

WB = 1.040

try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils")
    from state import State
    from util_class import Obstacle, Map
    from cubic_spline_planner import calc_spline_course
    from stanley import Stanley
except Exception as ex:
    print("UTIL CLASS IMPORT ERROR")
    print(ex)


class Box(object):
    def __init__(self, x, y, yaw, x_len, y_len):
        super(Box, self).__init__()

        self.x = x
        self.y = y
        self.yaw = yaw
        self.x_len = x_len
        self.y_len = y_len

        self.publisher = rospy.Publisher(
            "static_obstacle_area", Marker, queue_size=1)

    def checkIsInArea(self, obstacle):
        dist = np.hypot(self.x - obstacle.x, self.y - obstacle.y)

        if dist == 0.0:
            return True

        area_VEC = np.array([
            m.cos(self.yaw - m.radians(90.0)), m.sin(self.yaw - m.radians(90.0))
        ])

        ob_VEC = np.array([
            obstacle.x - self.x, obstacle.y - self.y
        ])

        theta = m.acos(np.dot(area_VEC, ob_VEC) / dist)

        x_dist = abs(dist * m.sin(theta))
        y_dist = abs(dist * m.cos(theta))

        if x_dist <= self.x_len / 2.0 and y_dist <= self.y_len / 2.0:
            return True

        return False

    def publishBox(self):
        msg = Marker()

        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        msg.lifetime = rospy.Duration(0.2)
        msg.ns = "static_obstacle_area"

        msg.type = msg.CUBE
        msg.action = msg.ADD

        msg.scale.x = self.x_len
        msg.scale.y = self.y_len
        msg.scale.z = 0.1

        msg.color.a = 0.5
        msg.color.r = 1.0

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.publisher.publish(msg)


class StaticObstacles(object):

    """

    Subscribe '/obstacles' and transform into odom frame
    Then, append them to Map

    """

    def __init__(self, state, cmd_msg, cmd_publisher, start_point=[0.0, 0.0]):
        super(StaticObstacles, self).__init__()

        rospy.Subscriber("/obstacles", PoseArray,
                         self.obstaclesCallback)

        """ TEST """

        self.areaFlag = False
        self.areaCnt = 0

        self.box = Box(-5.4544358253479, -103.28044128417969,
                       4.4404087821629386, 3.3172813542081476, 28.436379040076638)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         callback=self.obstacleTestCallback)

        if self.areaFlag is True:
            rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                             callback=self.createAreaCallback)

        while self.areaFlag is True and not rospy.is_shutdown():
            continue

        """ TEST END """

        self.path_pub = rospy.Publisher(
            "static_obstacle_path", Path, queue_size=1)

        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_publisher

        self.stanley = Stanley()

        self.target_idx = 0

        self.map = Map()

        self.map.obstacles = []

        self.state = state
        self.tf_node = tf.TransformListener()

        self.start_point = start_point
        self.new_start_point = self.start_point
        self.last_goal = None

        self.cx = []
        self.cy = []
        self.cyaw = []
        self.last_idx = -1

        self.doPublishing = True

    def obstacleTestCallback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y

        new_obstacle = Obstacle(x=x, y=y)

        print("ADD NEW OBSTACLE")

        self.map.obstacles.append(new_obstacle)

    def createAreaCallback(self, msg):
        # msg = PoseStamped()

        x = msg.pose.position.x
        y = msg.pose.position.y

        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w

        _, _, yaw = tf.transformations.euler_from_quaternion([ox, oy, oz, ow])

        if self.areaCnt == 0:
            self.box.x = x
            self.box.y = y
            self.box.yaw = yaw + m.radians(90.0)

        if self.areaCnt == 1:
            distance = np.hypot(x - self.box.x, y - self.box.y)
            self.box.x_len = distance * 2

        if self.areaCnt == 2:
            distance = np.hypot(x - self.box.x, y - self.box.y)
            self.box.y_len = distance * 2
            self.areaFlag = False

        print(
            self.box.x,
            self.box.y,
            self.box.yaw,
            self.box.x_len,
            self.box.y_len,
        )

        self.areaCnt += 1

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

                new_obstacle = self.tf_node.transformPose("map", new_obstacle)

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

        car_VEC = np.array([
            m.cos(self.state.yaw), m.sin(self.state.yaw)
        ])

        path_VEC = np.array([
            m.cos(LANE_YAW), m.sin(LANE_YAW)
        ])

        for obstacle in self.map.obstacles:

            # If obstacle is in area
            if self.box.checkIsInArea(obstacle) is True:

                rear_y = self.state.y + ((WB / 2) * m.sin(self.state.yaw))
                rear_x = self.state.x + ((WB / 2) * m.cos(self.state.yaw))

                ob_VEC = np.array([
                    obstacle.x - rear_x, obstacle.y - rear_y
                ])

                dot = np.dot(ob_VEC, car_VEC)

                # If obstacle is located in FRONT
                if dot >= 0.0:
                    distance = np.hypot(obstacle.x - self.state.x,
                                        obstacle.y - self.state.y)

                    if distance < min_dis:
                        min_dis = distance

                        cross = np.cross(ob_VEC, path_VEC)

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

        r = 1.0 if is_right is True else -1.0

        map_obstacle = self.addFrame("map", nearest_obstacle)

        map_obstacle.pose.position.x += (m.cos(LANE_YAW +
                                               m.radians(90.0)) * DISTANCE_GAP) * r
        map_obstacle.pose.position.y += (m.sin(LANE_YAW +
                                               m.radians(90.0)) * DISTANCE_GAP) * r

        return map_obstacle

    def createPath(self, goal_point):
        if goal_point is not None:
            xs = [self.new_start_point[0], goal_point.pose.position.x]
            ys = [self.new_start_point[1], goal_point.pose.position.y]

            cx, cy, cyaw, _, _ = calc_spline_course(x=xs, y=ys, ds=0.1)

            return cx, cy, cyaw

        return None, None, None

    def publishPath(self, cx, cy, cyaw):
        msg = Path()

        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        msg.poses = []

        for i in range(len(cx)):
            pose = PoseStamped()

            pose.header.frame_id = "map"
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

    def main(self):
        goal_point = self.createGoalPoint()

        if self.start_point[0] == 0.0 and self.start_point[1] == 0.0:
            self.new_start_point = [self.state.x, self.state.y]

        if self.doPublishing is True:
            self.box.publishBox()

        if goal_point is None:
            # print("NO GOAL POINT")
            self.target_idx = 0
            return False

        if goal_point != self.last_goal:
            print("DETECT NEW GOAL POINT")
            self.target_idx = 0
            self.new_start_point = [self.state.x, self.state.y]

        self.last_goal = goal_point

        cx, cy, cyaw = self.createPath(goal_point)

        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.last_idx = len(cx) - 1

        target_idx = self.target_idx
        di, target_idx = self.stanley.stanley_control(
            self.state, self.cx, self.cy, self.cyaw, target_idx
        )
        self.target_idx = target_idx

        if self.target_idx == self.last_idx:
            self.new_start_point = [self.state.x, self.state.y]
            self.target_idx = 0
            return False

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        self.cmd_msg.speed = desired_speed * 0.8
        self.cmd_msg.steer = -di
        self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)

        if self.doPublishing is True:
            self.publishPath(self.cx, self.cy, self.cyaw)
            self.box.publishBox()

        return True


if __name__ == "__main__":
    rospy.init_node("static_obstacles")

    state = State(0.0, 0.0, 0.0, 0.0)
    cmd_msg = stanleyMsg()

    tf_node = tf.TransformListener()

    rospy.Subscriber("/odometry/imu", Odometry,
                     callback=state.odometryCallback)

    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)

    static_ob = StaticObstacles(
        state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub, tf_node=tf_node)

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
