#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
from vehicle_msgs.msg import WaypointsArray, Waypoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/ACCA-master/utils")
    import cubic_spline_planner
except ImportError as ie:
    print(ie)
    print("CUBIC PLANNER IMPORT ERROR")



class ConeGlobalPath(object):
    def __init__(self, publisher):
        super(ConeGlobalPath, self).__init__()

        self.pub = publisher

        self.way_xs = []
        self.way_ys = []

        self.xs = []
        self.ys = []
        self.yaws = []


    def waypointsClustering(self, new_waypoints):

        if len(self.way_xs) == 0:
            for new_waypoint in new_waypoints:
                new_position_x = new_waypoint.y
                new_position_y = new_waypoint.id

                self.way_xs.append(new_position_x)
                self.way_ys.append(new_position_y)

        else:
            
            for new_waypoint in new_waypoints:

                min_dis = float("inf")

                for i in range(len(self.way_xs)):
                    new_position_x = new_waypoint.y
                    new_position_y = new_waypoint.id

                    old_position_x = self.way_xs[i]
                    old_position_y = self.way_ys[i]

                    distance = np.hypot(new_position_x - old_position_x, new_position_y - old_position_y)

                    if distance < min_dis:
                        min_dis = distance

                if min_dis > 0.3:
                    self.way_xs.append(new_position_x)
                    self.way_ys.append(new_position_y)


    

    def waypointsCallback(self, msg):
        waypoints = msg.waypoints

        self.waypointsClustering(waypoints)

        print(len(self.way_xs), len(self.way_ys))

        if len(self.way_xs) < 2:

            self.xs = []
            self.ys = []
            self.yaws = []

            return

        try:
            cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(self.way_xs, self.way_ys, ds=0.01)

            self.xs = cx
            self.ys = cy
            self.yaws = cyaw

        except Exception as ex:
            print(ex)

            self.xs = []
            self.ys = []
            self.yaws = []



    def publishPath(self):
        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"
        path.poses = []

        if len(self.xs) == len(self.ys) and len(self.ys) == len(self.yaws):

            for i in range(len(self.xs)):
                pose = PoseStamped()

                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "odom"

                pose.pose.position.x = self.xs[i]
                pose.pose.position.y = self.ys[i]
                pose.pose.position.z = 0.0

                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaws[i])

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                path.poses.append(pose)

            self.pub.publish(path)

        else:
            raise ValueError



if __name__ == "__main__":
    rospy.init_node("cone_global_path_generator")

    path_pub = rospy.Publisher("cone_global_path", Path, queue_size=1)

    cone_path = ConeGlobalPath(publisher=path_pub)

    rospy.Subscriber("/waypoints", WaypointsArray, callback=cone_path.waypointsCallback)
    rospy.Subscriber("/is_loop_close", UInt8,
                         callback=self.loopCloseCallback)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        try:
            cone_path.publishPath()

        except ValueError as ve:
            print(ve)
            print("INVAILID ARRAY LENTH")

        except Exception as ex:
            print(ex)
            print("WHAt THE FUCK")

        r.sleep()