#!/usr/bin/env python


import sys
import rospy
import tf
from vehicle_msgs.msg import TrackCone, Command, Waypoint, WaypointsArray, Track
from vehicle_msgs.msg import Track
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
import math as m
from geometry_msgs.msg import PoseArray,Pose, PoseStamped
import numpy as np


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils")
    from cubic_spline_planner import calc_spline_course
except Exception as ex:
    print(ex)


class Track2Waypoints(object):
    def __init__(self):
        super(Track2Waypoints, self).__init__()

        # Get all parameters from launch file
        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
        self.shouldPublishPredefined = rospy.get_param('~publishPredefined', False)

        if rospy.has_param('~odom_topic'):
            self.odometry_topic = rospy.get_param('~odom_topic')
        else:
            self.odometry_topic = "/odometry"

        if rospy.has_param('~world_frame'):
            self.world_frame = rospy.get_param('~world_frame')
        else:
            self.world_frame = "world_frame"

        waypointsFrequency = rospy.get_param('~desiredWaypointsFrequency', 5)
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.waypoints = []

        self.cx = []
        self.cy = []
        self.cyaw = []

        # All Subs and pubs
        # rospy.Subscriber(self.odometry_topic, Odometry, self.odometryCallback)
        #rospy.Subscriber("/erp42_encoder", encoderMsg, self.carSensorsCallback)
        
        # # Create publishers
        # self.waypointsPub = rospy.Publisher("/waypoints", WaypointsArray, queue_size=1)
        # self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)
        
        self.map = []


    def mapCallback(self, msg):
     
        self.map = msg.cones

    def Tracktfpublish(self, publisher):
        test = PoseArray()
        test.header.frame_id = "odom"
        for TrackCone in self.map :
            temp = Pose()
            temp.position.x = TrackCone.x
            temp.position.y = TrackCone.y
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
        if len(maps) < 2:
            print("Cannot create waypoint")
            return

        elif len(maps) > 2:
            for cone in maps:
                dist = m.sqrt(m.pow(cone.x,2)+m.pow(cone.y,2))
                distance.append(dist)
            sorted_distance = sorted(distance)
            min1_index = distance.index(sorted_distance[0])
            min2_index = distance.index(sorted_distance[1])
            a=[maps[min2_index].x-maps[min1_index].x,maps[min2_index].y-maps[min1_index].y,0]
            b=[0,0,1]
            vector = np.cross(a,b)
            norm = np.linalg.norm(vector)
            unit = vector/norm


            data1 = [maps[min1_index].x + unit[0], maps[min1_index].y+unit[1]]
            data2 = [maps[min2_index].x + unit[0], maps[min2_index].y+unit[1]]
            
            waypoint.append(data1)
            waypoint.append(data2)
            self.update(waypoint)          

        else:
            a=[maps[1].x-maps[0].x,maps[1].y-maps[0].y,0]
            b=[0,0,1]
            vector = np.cross(a,b)
            norm = np.linalg.norm(vector)
            unit = vector/norm

            data1 = [maps[0].x + unit[0], maps[0].y+unit[1]]
            data2 = [maps[1].x + unit[0], maps[1].y+unit[1]]
          
            
            waypoint.append(data1)
            waypoint.append(data2)

            self.update(waypoint)
                    
        return waypoint

    def testpublish(self, publisher, waypoints):
        test = PoseArray()
        test.header.frame_id = "odom"

        for i in range (len(waypoints)) :
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

    def update(self,waypoints):

        temp_cx = []
        temp_cy = []

        for i in range(len(waypoints)):
            
            temp_cx.append(waypoints[i][0])
            temp_cy.append(waypoints[i][1])

        self.cx = temp_cx
        self.cy = temp_cy
   
    def publishPath(self, publisher):
        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        path.poses = []
        try:
            # cx, cy, cyaw, _, _ = calc_spline_course(x=self.cx, y=self.cy, ds=0.1)
            print(self.cy[1])
            for i in range(len(self.cx)):
                pose = PoseStamped()
                
                pose.header.frame_id = "odom"
                pose.header.stamp = rospy.Time.now()

                pose.pose.position.x = self.cx[i] 
                pose.pose.position.y = self.cy[i]
                pose.pose.position.z = 0.0
                
                pose.pose.orientation.w = 1.0

               

                path.poses.append(pose)
            publisher.publish(path)

        except Exception:
            pass



if __name__ == "__main__":
    rospy.init_node("waypoints_maker")

    wp = Track2Waypoints()

    rospy.Subscriber("/track", Track, wp.mapCallback)
    test_pub = rospy.Publisher("waypoints", PoseArray, queue_size=1)
    path_pub = rospy.Publisher("paths", Path, queue_size=1)
    track_pub = rospy.Publisher("track_re",PoseArray, queue_size=1)
    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        # waypoints.publishPath(publisher=waypoints_pub)
        waypoints = wp.Generation_waypoints(wp.map)

        if waypoints:
            # print(waypoints)
            wp.testpublish(publisher=test_pub,waypoints=waypoints)
            wp.Tracktfpublish(publisher=track_pub)
            wp.publishPath(publisher=path_pub)
        
        r.sleep()