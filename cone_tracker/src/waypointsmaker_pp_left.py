#!/usr/bin/env python


import sys
import rospy
import tf
from vehicle_msgs.msg import TrackCone, Command, Waypoint, WaypointsArray
from vehicle_msgs.msg import Track
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
import math as m
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import numpy as np
from path_planner.msg import stanleyMsg
from nav_msgs.msg import Odometry, Path
# from state import State

ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")
ODOMETRY_TOPIC = rospy.get_param("/odometry_topic", "/odom")


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src" + "/ACCA/utils")
    # from cubic_spline_planner import calc_spline_course
    from state import State

except Exception as ex:
    print(ex)


class Track2Waypoints(object):
    def __init__(self, state, cmd_msg, cmd_pub, speed):
        super(Track2Waypoints, self).__init__()

        # Get all parameters from launch file
        self.shouldPublishWaypoints = rospy.get_param(
            '~publishWaypoints', True)
        self.shouldPublishPredefined = rospy.get_param(
            '~publishPredefined', False)

        if rospy.has_param('~odom_topic'):
            self.odometry_topic = rospy.get_param('~odom_topic')
        else:
            self.odometry_topic = "/erp42_fusion_odometry"

        if rospy.has_param('~world_frame'):
            self.world_frame = rospy.get_param('~world_frame')
        else:
            self.world_frame = "world_frame"

        waypointsFrequency = rospy.get_param('~desiredWaypointsFrequency', 5)
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        self.state = state

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0
        self.speed = speed
        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_pub

        self.waypoints = []

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.c_x = 0.0  # look ahead x point
        self.c_y = 0.0  # look ahead y point

        self.delta_ref = 0.0
        self.v = 3.0
        self.K = 2.0
        self.L = 1.040

        self.ind = 0
        self.Lr = self.v * self.K * self.L
        self.Ld = 0.0
        self.Map = []

        self.servo_bias = 0.0

    def updates(self):
        Lr = self.v * self.K - self.L

        if Lr > 0:
            self.Lr = Lr
        else:
            self.Lr = 2.0

        # All Subs and pubs
        # rospy.Subscriber(self.odometry_topic, Odometry, self.odometryCallback)
        #rospy.Subscriber("/erp42_encoder", encoderMsg, self.carSensorsCallback)

        # # Create publishers
        # self.waypointsPub = rospy.Publisher("/waypoints", WaypointsArray, queue_size=1)
        # self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

    def mapCallback(self, msg):

        self.Map = msg.cones

    def Tracktfpublish(self, publisher):
        test = PoseArray()
        test.header.frame_id = "odom"
        for cone in self.Map:
            temp = Pose()
            temp.position.x = cone.x
            temp.position.y = cone.y
            temp.position.z = 0.0

            temp.orientation.x = 0.0
            temp.orientation.y = 0.0
            temp.orientation.z = 0.0
            temp.orientation.w = 1.0

            test.poses.append(temp)

        publisher.publish(test)


# self.mergeWaypoints(newWaypoints)
# self.publishWaypoints(newWaypoints)

    def Generation_waypoints(self, maps):
        distance = []
        waypoint = []
        idx = []
        if len(maps) < 2:
            print("Cannot create waypoint")
            return

        elif len(maps) >= 2:
            for i in range(len(maps)):

                dist = np.hypot(maps[i].x-self.state.x, maps[i].y-self.state.y)
                distance.append(dist)
            sorted_distance = sorted(distance)
            min1_index = int(distance.index(sorted_distance[0]))
            min2_index = int(distance.index(sorted_distance[1]))

            x_vec = (maps[min2_index].x - maps[min1_index].x)
            y_vec = (maps[min2_index].y - maps[min1_index].y)
            r = 1.0

            # if x_vec < 0.0:
            #      else -1.0

            a = [x_vec, y_vec, 0.0]   # min2_ind - min1_ind
            b = [0.0, 0.0, 1.0]   # z_vector
            vector = np.cross(a, b)

            # if vector[0] < 0.0:
            #     vector[0] *= -1.0

            # print(vector[0])

            norm = np.linalg.norm(vector)
            unit = vector/norm

            data1 = [maps[min1_index].x + 0.9*unit[0],
                     maps[min1_index].y+0.9*unit[1]]
            data2 = [maps[min2_index].x + 0.9*unit[0],
                     maps[min2_index].y+0.9*unit[1]]

            # print(x_vec)
            # print(vector)
            # print(data1)
            # print()

            waypoint.append(data1)
            waypoint.append(data2)
            self.update(waypoint)

        return waypoint

    def testpublish(self, publisher, waypoints):
        test = PoseArray()
        test.header.frame_id = "odom"

        for i in range(len(waypoints)):
            temp = Pose()
            temp.position.x = waypoints[i][0]
            temp.position.y = waypoints[i][1]
            temp.position.z = 0.0

            temp.orientation.x = 0.0
            temp.orientation.y = 0.0
            temp.orientation.z = 0.0
            temp.orientation.w = 1.0

            test.poses.append(temp)

        publisher.publish(test)

    def update(self, waypoints):

        temp_cx = []
        temp_cy = []

        for i in range(len(waypoints)):

            temp_cx.append(waypoints[i][0])
            temp_cy.append(waypoints[i][1])

        self.cx = temp_cx
        self.cy = temp_cy

    # def closest_path_point(self):
    #     # i = 0
    #     # cpp = 0

    #     dx = self.cx - self.state.x
    #     dy = self.cy - self.state.y
    #     d = np.hypot(dx, dy)

    #     # if (i == 0) or (cpp > d):
    #     #     cpp = d
    #     #     self.ind = i

    #     # i += 1

    #     return

    # def find_goal_point(self):  # 3 find the goal point
    #     j = self.ind  # index
    #     b = False  # boolean True when we find look ahead point or False

    #     while (j < len(self.cx) - 2) and (b is False):
    #         dx1 = self.cx[j] - self.state.x
    #         dy1 = self.cy[j] - self.state.y
    #         d1 = np.hypot(dx1, dy1)

    #         dx2 = self.cx[j + 1] - self.state.x
    #         dy2 = self.cy[j + 1] - self.state.y
    #         d2 = np.hypot(dx2, dy2)

    #         if (d1 < self.Lr) and (d2 < self.Lr):
    #             # in case the current and next points are too close
    #             j += 1
    #             self.c_x = self.cx[j + 1]
    #             self.c_y = self.cy[j + 1]

    #         elif ((d1 < self.Lr) and (d2 > self.Lr)):
    #             # in case the current point close than the required look ahead length and the next point is too far
    #             b = True

    #             # interpolation
    #             self.c_x = (self.cx[j+1] + self.cx[j]) / 2
    #             self.c_y = (self.cy[j+1] + self.cy[j]) / 2

    #             self.ind = j + 1

    #         elif (d1 > self.Lr) and (d2 > self.Lr):
    #             # in case there is no closer new look ahead point
    #             b = True

    #             self.ind = j
    #             self.c_x = self.cx[j]
    #             self.c_y = self.cy[j]

    #         else:
    #             b = True

    #     return self.c_x, self.c_y

    def set_steering(self):
        if self.cx:
            dx, dy = 0, 0
            dist = []
            for i in range(len(self.cx)):
                dx = self.cx[i] - self.state.x
                dy = self.cy[i] - self.state.y
                dist.append(np.hypot(dx, dy))
            sorted_dist = sorted(dist)
            idx = dist.index(sorted_dist[0])
            if dx == 0.0:
                self.delta_ref = 0.0
                return self.delta_ref

            self.Ld = sorted_dist[0]
            if self.Ld > 2.0:
                alpha = m.atan(
                    (self.cy[idx] - self.state.y)/(self.cx[idx] - self.state.x)) - self.state.yaw
                # alpha = m.asin(dy/self.Ld)  # if error -- change dx to dy

                if dx > 0:
                    alpha *= -1.0

                self.delta_ref = m.atan(2 * self.L * m.sin(alpha) / self.Ld)
            else:
                self.Ld = sorted_dist[1]

                alpha = m.atan(
                    (self.cy[idx] - self.state.y)/(self.cx[idx] - self.state.x)) - self.state.yaw
                # alpha = m.asin(dy/self.Ld)  # if error -- change dx to dy

                if dx > 0:
                    alpha *= -1.0

                self.delta_ref = m.atan(2 * self.L * m.sin(alpha) / self.Ld)

        # print(self.delta_ref)
        return self.delta_ref  # *0.6

    def steady_state_bias(self):  # this function insert steady state bias to servo
        self.delta_ref
        self.delta_ref = self.delta_ref + self.servo_bias
        return

    def re_calcalc_point(self):
        new_ind = self.closest_path_point()
        self.ind = new_ind
        return self.ind

    def main(self):
        self.updates()

        # if self.ind == 0:
        #     ind = self.closest_path_point()

        # cx, cy = self.find_goal_point()
        di = self.set_steering()

        self.cmd_msg.speed = self.speed
        self.cmd_msg.steer = di
        self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)


if __name__ == "__main__":
    rospy.init_node("waypoints_maker")

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    cmd_msg = stanleyMsg()

    cmd_pub = rospy.Publisher("Control_msg", stanleyMsg, queue_size=1)
    wp = Track2Waypoints(state=state, cmd_msg=cmd_msg,
                         cmd_pub=cmd_pub, speed=10.0)
    test_pub = rospy.Publisher("waypoints", PoseArray, queue_size=1)
    track_pub = rospy.Publisher("track_re", PoseArray, queue_size=1)

    rospy.Subscriber(ODOMETRY_TOPIC, Odometry, state.odometryCallback)
    rospy.Subscriber("/track", Track, wp.mapCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        # waypoints.publishPath(publisher=waypoints_pub)
        waypoints = wp.Generation_waypoints(maps=wp.Map)
        wp.main()

        if waypoints:
            # print(waypoints)
            wp.testpublish(publisher=test_pub, waypoints=waypoints)
            wp.Tracktfpublish(publisher=track_pub)

        r.sleep()
