#!/usr/bin/env python

import rospy
import math as m
from sklearn import datasets
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from sklearn.cluster import KMeans


class ObstacleDetection(object):
    def __init__(self):
        super(ObstacleDetection, self).__init__()

        self.laserData = LaserScan()

    def laserCallback(self, msg):
        # self.laserData = msg
        self.handelLaser(msg=msg)

    def handelLaser(self, msg):
        new_laser = msg

        ranges = new_laser.ranges
        new_ranges = []

        init_angle = m.radians(-45.0)

        for r in ranges:
            # if 0.0 < m.degrees(init_angle) and m.degrees(init_angle) < 180.0:

            if r < 1.0:
                new_ranges.append(r)
            else:
                new_ranges.append(float("inf"))

            init_angle += m.radians(0.5)

        new_laser.ranges = new_ranges

        self.laserData = new_laser

    def publishCenter(self, centers, publisher):
        poseArray = PoseArray()

        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = "laser"
        poseArray.poses = []

        for center in centers:
            pose = Pose()

            pose.position.x = center[0]
            pose.position.y = center[1]
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            poseArray.poses.append(pose)

        publisher.publish(poseArray)

    def kmean_clustering(self):
        data = self.transform_laser_to_pose()

        if len(data) == 0:
            return []

        km = KMeans(n_jobs=5, max_iter=500, n_clusters=1)
        km.fit(data)

        centers = km.cluster_centers_

        return centers

    def transform_laser_to_pose(self):
        ranges = self.laserData.ranges

        temp_poses = []

        init_yaw = m.radians(-45.0)
        for r in ranges:

            if r != float("inf"):

                x = r * m.sin(init_yaw)
                y = r * m.cos(init_yaw)

                temp_poses.append([y, x])

            init_yaw += m.radians(0.5)

        return temp_poses


if __name__ == "__main__":
    rospy.init_node("obstacle_detection")

    obstacle = ObstacleDetection()

    rospy.Subscriber("/scan_filtered", LaserScan, obstacle.laserCallback)

    obstacle_pub = rospy.Publisher("obstacles", PoseArray, queue_size=1)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        try:
            centers = obstacle.kmean_clustering()

            obstacle.publishCenter(centers=centers, publisher=obstacle_pub)
        except Exception as ex:
            print(ex)

        r.sleep()
