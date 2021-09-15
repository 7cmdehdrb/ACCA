#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from path_planner.msg import stanleyMsg


from state import State
from loadPose import LoadPose


SPEED = rospy.get_param("/desired_speed", 1.0)


class ConeTracker(object):
    def __init__(self, load):
        super(ConeTracker, self).__init__()

        self.path = Path()
        self.isLoopClose = False

        self.load = load

        # rospy.Subscriber("/cone_path", Path, callback=self.pathCallback)
        # rospy.Subscriber("/is_loop_close", UInt8,
        #                  callback=self.loopCloseCallback)

    def pathCallback(self, msg):
        temp_x = []
        temp_y = []
        temp_yaw = []

        poses = msg.poses

        for pose in poses:
            X = pose.pose.position.x
            Y = pose.pose.position.y

            ori_x = pose.pose.orientation.x
            ori_y = pose.pose.orientation.y
            ori_z = pose.pose.orientation.z
            ori_w = pose.pose.orientation.w

            quat = [ori_x, ori_y, ori_z, ori_w]

            _, _, YAW = tf.transformations.euler_from_quaternion(quat)

            temp_x.append(X)
            temp_y.append(Y)
            temp_yaw.append(YAW)

        self.load.cx = temp_x
        self.load.cy = temp_y
        self.load.cyaw = temp_yaw

    def loopCloseCallback(self, msg):
        self.isLoopClose = True if msg.data == 1 else False


class PurePursuit(object):
    def __init__(self, state, cmd_msg, cmd_pub, load, speed=SPEED):
        super(PurePursuit, self).__init__()

        """ Input Var """

        self.state = state
        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_pub
        self.load = load
        self.speed = speed

        self.ind = 0  # global index

        self.delta_ref = 0.0  # steer

        self.c_x = 0.0  # look ahead x point
        self.c_y = 0.0  # look ahead y point

        self.servo_bias = 0.0  # [rad] 0.03

        self.v = 3.0
        self.K = rospy.get_param("k_gain", 1.0)
        self.L = 1.040

        self.Lr = self.v * self.K - self.L
        self.Ld = 0.0

    def update(self):
        Lr = self.v * self.K - self.L

        if Lr > 0:
            self.Lr = Lr
        else:
            self.Lr = 2.0

    def closest_path_point(self):  # 2 finding the closest path point
        i = 0  # index
        cpp = 0  # closest path point [m]

        dx = self.load.cx[i] - self.state.x
        dy = self.load.cy[i] - self.state.y
        d = np.hypot(dx, dy)

        while (i < len(self.load.cx)):  # searching the whole path points
            dx = self.load.cx[i] - self.state.x
            dy = self.load.cy[i] - self.state.y
            d = np.hypot(dx, dy)

            if (i == 0) or (cpp > d):
                cpp = d
                self.ind = i

            i += 1

        return self.ind

    def find_goal_point(self):  # 3 find the goal point
        j = self.ind  # index
        b = False  # boolean True when we find look ahead point or False

        while (j < len(self.load.cx) - 2) and (b is False):
            dx1 = self.load.cx[j] - self.state.x
            dy1 = self.load.cy[j] - self.state.y
            d1 = np.hypot(dx1, dy1)

            dx2 = self.load.cx[j + 1] - self.state.x
            dy2 = self.load.cy[j + 1] - self.state.y
            d2 = np.hypot(dx2, dy2)

            if (d1 < self.Lr) and (d2 < self.Lr):
                # in case the current and next points are too close
                j += 1
                self.c_x = self.load.cx[j + 1]
                self.c_y = self.load.cy[j + 1]

            elif ((d1 < self.Lr) and (d2 > self.Lr)):
                # in case the current point close than the required look ahead length and the next point is too far
                b = True

                # interpolation
                self.c_x = (self.load.cx[j+1] + self.load.cx[j]) / 2
                self.c_y = (self.load.cy[j+1] + self.load.cy[j]) / 2

                self.ind = j + 1

            elif (d1 > self.Lr) and (d2 > self.Lr):
                # in case there is no closer new look ahead point
                b = True

                self.ind = j
                self.c_x = self.load.cx[j]
                self.c_y = self.load.cy[j]

            else:
                b = True

        return self.c_x, self.c_y

    def set_steering(self):  # 5 calculate the new required steering angle
        dx = self.c_x - self.state.x
        dy = self.c_y - self.state.y

        if dx == 0.0:
            self.delta_ref = 0.0
            return self.delta_ref

        self.Ld = np.hypot(dx, dy)
        alpha = math.atan(dy / dx) - self.state.yaw

        if dx > 0:
            alpha *= -1.0

        self.delta_ref = math.atan(2 * self.Ld * math.sin(alpha) / self.Ld)

        return self.delta_ref

    def steady_state_bias(self):  # this function insert steady state bias to servo
        self.delta_ref
        self.delta_ref = self.delta_ref + self.servo_bias
        return

    def re_calcalc_point(self):
        new_ind = self.closest_path_point()
        self.ind = new_ind
        return self.ind

    def main(self):
        self.update()

        if self.ind == 0:
            ind = self.closest_path_point()

        cx, cy = self.find_goal_point()
        di = self.set_steering()

        self.cmd_msg.speed = self.speed
        self.cmd_msg.steer = di
        self.cmd_msg.brake = 1

        self.cmd_pub.publish(self.cmd_msg)


if __name__ == '__main__':
    rospy.init_node("pure_pursuit_control")

    state = State(x=-0.0, y=-0.0, yaw=-0.0, v=0.0)
    cmd_msg = stanleyMsg()
    cmd_pub = rospy.Publisher("Control_msg", stanleyMsg, queue_size=1)
    load = LoadPose(file_name="path.csv")

    cone_path = ConeLoad()
    load = cone_path

    pure_pursuit = PurePursuit(
        state=state, cmd_msg=cmd_msg, cmd_pub=cmd_pub, load=load)

    rospy.Subscriber("/fake_odom", Odometry, state.odometryCallback)

    path_pub = rospy.Publisher("pp_path", Path, queue_size=1)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        pure_pursuit.main()

        load.pathPublish(pub=path_pub)

        r.sleep()
