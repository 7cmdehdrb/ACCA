#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from path_planner.msg import stanleyMsg
from loadPose import LoadPose
from stanley import stanley_control, calc_target_index, normalize_angle


desired_speed = rospy.get_param("/desired_speed", 30.0)  # kph


class PathFinder(object):
    def __init__(self):
        super(PathFinder, self).__init__()
        self.counter = True

        self.path = Path()

        self.cx = []
        self.cy = []
        self.cyaw = []


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

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


def checkGoal(current_speed, last_idx, current_idx):
    global desired_speed

    temp_speed = 0.0
    temp_brake = 1

    if abs(last_idx - current_idx) < 20:
        temp_speed = 0.0
        temp_brake = 80

    else:
        temp_speed = desired_speed
        temp_brake = 1

    return temp_speed, temp_brake


if __name__ == "__main__":
    rospy.init_node("stanley_method")

    # Initial state
    load = LoadPose()
    load.readCSV()

    state = State(x=-0.0, y=0.0, yaw=np.radians(0.0), v=0.0)
    path = PathFinder()

    cmd_msg = stanleyMsg()

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    cmd_pub = rospy.Publisher("/stanley_cmd", stanleyMsg, queue_size=1)
    path_pub = rospy.Publisher("/cublic_global_path", Path, queue_size=1)

    path.cx = load.cx
    path.cy = load.cy
    path.cyaw = load.cyaw

    speed = 0.0
    brake = 0.0

    last_idx = len(path.cx) - 1

    target_idx, _ = calc_target_index(state, path.cx, path.cy)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        di, target_idx = stanley_control(
            state, path.cx[:target_idx+100], path.cy[:target_idx+100], path.cyaw[:target_idx+100], target_idx)

        di = np.clip(di, -m.radians(30.0), m.radians(30.0))

        speed, brake = checkGoal(current_speed=speed,
                                 last_idx=last_idx, current_idx=target_idx)

        cmd_msg.speed = speed
        cmd_msg.steer = -di
        cmd_msg.brake = brake

        cmd_pub.publish(cmd_msg)

        rospy.loginfo((-m.degrees(di), target_idx))

        load.pathPublish(pub=path_pub)

        r.sleep()
