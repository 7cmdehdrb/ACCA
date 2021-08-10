#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path, Odometry
from path_planner.msg import stanleyMsg
import cubic_spline_planner
import stanley_control


desired_speed = rospy.get_param("/desired_speed", 3.0)  # kph
max_steer = rospy.get_param("/max_steer", 30.0)


class PathDetection(object):
    def __init__(self):
        super(PathDetection, self).__init__()

        self.paths = []
        self.x_paths = []
        self.y_paths = []

        self.final_path = Path()

    def poseCallback(self, msg):
        data = msg()

        x = data.pose.position.x
        y = data.pose.position.y

        new_path_position = [x, y]

        if len(self.paths) == 0:
            self.paths.append(new_path_position)

        for obstacle in self.paths:
            distance = np.hypot(
                obstacle[0] - new_path_position[0], obstacle[1] - new_path_position[1])

            if distance < 0.2:
                self.paths.append(new_path_position)
                self.x_paths.append(new_path_position[0])
                self.x_paths.append(new_path_position[1])

    def publishPath(self, cx, cy, cyaw, publisher):
        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = []

        for i in range(0, len(cx) - 1):
            temp_pose = Pose()

            temp_pose.position.x = cx[i]
            temp_pose.position.y = cy[i]
            temp_pose.position.z = 0.0

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, cyaw[i])

            temp_pose.orientation.x = quat[0]
            temp_pose.orientation.y = quat[1]
            temp_pose.orientation.z = quat[2]
            temp_pose.orientation.w = quat[3]

            path.poses.append(temp_pose)

        self.final_path = path
        publisher.publish(path)


class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()

        self.data = Odometry()

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.dt = 0.1

        self.x = x
        self.y = y
        self.yaw = yaw

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw

        self.v = v
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)

    def odometryCallback(self, msg):
        self.data = msg

        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        if self.dt == 0:
            return

        self.x = self.data.pose.pose.position.x
        self.y = self.data.pose.pose.position.y

        x = self.data.pose.pose.orientation.x
        y = self.data.pose.pose.orientation.y
        z = self.data.pose.pose.orientation.z
        w = self.data.pose.pose.orientation.w

        quaternion_array = [x, y, z, w]

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion_array)

        self.yaw = yaw

        self.dx = (self.x - self.last_x) / self.dt
        self.dy = (self.x - self.last_y) / self.dt
        self.dyaw = (self.yaw - self.last_yaw) / self.dt
        self.v = np.hypot(self.dx, self.dy)

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw
        self.lastTime = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    # Object
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    path = PathDetection()
    cmd_msg = stanleyMsg()

    rospy.Subscriber("/odom", Odometry, state.odometryCallback)
    rospy.Subscriber("/hj", PoseStamped, path.poseCallback)

    path_pub = rospy.Publisher("/cone_path", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/cone_stanley_cmd", stanleyMsg, queue_size=1)

    target_idx = 0

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(
            x=path.x_paths, y=path.y_paths, ds=0.1
        )

        di, target_idx = stanley_control(
            state, cx, cy, yaw, target_idx
        )

        cmd_msg.speed = desired_speed
        cmd_msg.steer = - \
            np.clip(di, -m.radians(max_steer), m.radians(max_steer))
        cmd_msg.brake = 1

        cmd_pub.publish(cmd_msg)
        path.publishPath(cx=cx, cy=cy, cyaw=cyaw, publisher=path_pub)

        r.sleep()
