#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
import math as m


detection_range = rospy.get_param("/detection_range", 10.0)


class Obstacle(object):
    def __init__(self):
        super(Obstacle, self).__init__()

        self.laserData = LaserScan()
        self.poseArray = PoseArray()

        obstacle_x = []
        obstacle_y = []

    def laserCallback(self, msg):
        self.laserData = msg

    def transform_laser_to_pose(self):
        ranges = self.laserData.ranges

        temp_poses = []

        init_yaw = m.radians(-45.0)
        for r in ranges:
            if 0.3 < r and r < detection_range:
                pose = Pose()

                pose.position.x = r * m.sin(init_yaw)
                pose.position.y = -r * m.cos(init_yaw)
                pose.position.z = 0.0

                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0

                temp_poses.append(pose)

            init_yaw += m.radians(0.5)

        return temp_poses

    def publishObstacle(self, publisher):
        pose_array = PoseArray()

        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        pose_array.poses = self.transform_laser_to_pose()

        publisher.publish(pose_array)


if __name__ == "__main__":
    rospy.init_node("obstacle_detection")

    obstacle = Obstacle()

    rospy.Subscriber("/scan", LaserScan, obstacle.laserCallback)

    obstacle_pub = rospy.Publisher("/obstacles", PoseArray, queue_size=1)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        obstacle.publishObstacle(publisher=obstacle_pub)
        r.sleep()
