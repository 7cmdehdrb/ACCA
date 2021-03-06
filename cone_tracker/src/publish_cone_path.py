#!/usr/bin/env python


import sys
import rospy
import tf
from std_msgs.msg import UInt8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose,PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from vehicle_msgs.msg import Waypoint, WaypointsArray


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils")
    from cubic_spline_planner import calc_spline_course
except Exception as ex:
    print(ex)


class Waypoint(object):
    def __init__(self, x, y, z):
        super(Waypoint, self).__init__()

        self.x = x
        self.y = y
        self.z = z


class Waypoints(object):
    def __init__(self):
        super(Waypoints, self).__init__()

        self.waypoints = []

        self.cx = []
        self.cy = []
        self.cyaw = []

    def poseArrayCallback(self, msg):
        temp = []
        poses = msg.poses

        try:
            for pose in poses:
                test = Pose()
                test = Waypoint(x=test.position.x, y=test.position.y, z=0.0)
                temp.append(test)

            
            self.waypoints = temp
            print(self.waypoints)
            
        except Exception as ex:
            print(ex)
        self.update()

    def update(self):
        temp = []
        temp_cx = []
        temp_cy = []

        for waypoint in self.waypoints:
            new_waypoint = Waypoint(x=waypoint.x, y=waypoint.y, z=0.0)

            temp.append(new_waypoint)
            temp_cx.append(waypoint.x)
            temp_cy.append(waypoint.y)
            
        self.waypoints = temp
        self.cx = temp_cx
        self.cy = temp_cy


    def publishPath(self, publisher):
        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        path.poses = []

        try:
            cx, cy, cyaw, _, _ = calc_spline_course(
                x=self.cx, y=self.cy, ds=0.1)

            for i in range(len(cx)):
                pose = PoseStamped()

                pose.header.frame_id = "odom"
                pose.header.stamp = rospy.Time.now()

                pose.pose.position.x = cx[i]
                pose.pose.position.y = cy[i]
                pose.pose.position.z = 0.0

                quat = tf.transformations.quaternion_from_euler(
                    0.0, 0.0, cyaw[i])

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                path.poses.append(pose)

            publisher.publish(path)

        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("publish_cone_path")

    waypoints = Waypoints()

    rospy.Subscriber("/waypoints", PoseArray, callback=waypoints.poseArrayCallback)

    path_pub = rospy.Publisher("cone_path", Path, queue_size=1)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        waypoints.publishPath(publisher=path_pub)
        r.sleep()