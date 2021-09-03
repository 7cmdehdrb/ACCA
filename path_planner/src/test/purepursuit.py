#!/usr/bin/env python

import numpy as np
import math
# import
# import csv
# import os
import sys
import rospy
from nav_msgs.msg import Odometry, Path
from path_planner.msg import stanleyMsg

# dt = 0.05  # [s]
# time = 0.0  # current simulation time
# simulation_time = 7200  # [s]
# S_t = 0.2  # Servo delay 200 [msec]
# L = 1.04  # length of platform [meter]
# v = 10.0  # car velocity [m/s]
# K = 0.4  # proportion controller for look ahead [sec]        v*K=40
# Lr = K*v - L  # [m] Ld required gain
# Ld = 0.0  # actual look ahead length
# ind = 0.0  # global index
# max_delta = 0.48  # max wheel angle [rad]______27 degree
# min_delta = -0.48
# delta_ref = 0.0  # required delta [rad]
# last_delta_ref = 0.0
# delay = 0.2  # [s]
# a = 0.0  # for servo dynamic function
# c_x = 0.0  # look ahead x point
# c_y = 0.0  # look ahead y point
# global i  # lidar dataset index
# servo_bias = 0.03  # [rad]


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    from loadPose import LoadPose
except Exception as ex:
    try:
        sys.path.insert(0, "/home/hojin/catkin_ws/src/utils")
        from state import State
        from loadPose import LoadPose
    except Exception as ex:

        print("UTIL IMPORT ERROR")
        print(ex)
        sys.exit()

# try:
#     sys.path.insert(0, "/home/hojin/catkin_ws/src/utils")
#     from  import
#     from state import State
#     from loadPose import LoadPose
# except Exception as ex:
#     print("GLOBAL STANLEY IMPORT ERROR")
#     print(ex)


class PurePursuit(object):
    def __init__(self, load):
        super(PurePursuit, self).__init__()

        self.dt = 0.05  # [s]
        self.time = 0.0  # current simulation time
        self.simulation_time = 7200  # [s]
        self.S_t = 0.2  # Servo delay 200 [msec]
        # self.Ld = 0.0  # actual look ahead length
        self.ind = 0  # global index
        self.max_delta_ref = 0.5  # max wheel angle [rad]
        self.min_delta_ref = -0.5
        self.delta_ref = 0.0  # required delta [rad]
        self.last_delta_ref = 0.0
        self.delay = 0.2  # [s]
        self.a = 0.0  # for servo dynamic function
        self.c_x = 0.0  # look ahead x point
        self.c_y = 0.0  # look ahead y point
        # self.i  # lidar dataset index
        self.servo_bias = 0.0  # [rad] 0.03
        self.v = 3.0
        self.K = 1.7
        self.L = 1.04
        # self.v * self.K - self.L = different for each path
        self.Lr = self.v * self.K - self.L
        self.Ld = 5.0

    def closest_path_point(self, states, load):  # 2 finding the closest path point
        i = 0  # index
        cpp = 0  # closest path point [m]
        dx = load.cx[i] - states.x
        dy = load.cy[i] - states.y
        d = math.sqrt(dx**2 + dy**2)
        # print(i)
        while (i < len(load.cx)):  # searching the whole path points
            # dx = load.cx[i] - states.x
            # dy = load.cy[i] - states.y
            # d = math.sqrt(dx**2 + dy**2)
            if (i == 0) or (cpp > d):
                cpp = d
                self.ind = i
                # print(d)
                # print(i)
                # print("-----------------------")
            i += 1

        return

    def find_goal_point(self, states, load):  # 3 find the goal point
        j = self.ind  # index
        b = False  # boolean True when we find look ahead point or False
        print("idx: " + str(j))
        while (j < len(load.cx)) and (b is False):
            dx1 = load.cx[j] - states.x
            dy1 = load.cy[j] - states.y
            d1 = math.sqrt(dx1**2 + dy1**2)
            dx2 = load.cx[j+1] - states.x
            dy2 = load.cy[j+1] - states.y
            d2 = math.sqrt(dx2**2 + dy2**2)

            if (d1 < self.Lr) and (d2 < self.Lr):  # in case the current and next points are too close
                j += 1
                self.c_x = load.cx[j+1]
                self.c_y = load.cy[j+1]
            # in case the current point close than the required look ahead length and the next point is too far
            elif ((d1 < self.Lr) and (d2 > self.Lr)):
                b = True
                # interpolation
                self.c_x = (load.cx[j+1] + load.cx[j])/2
                # self.c_x = load.cx[j+1]
                # interpolation
                self.c_y = (load.cy[j+1] + load.cy[j])/2
                # self.c_y = load.cy[j+1]
                self.ind = j+1
            # in case there is no closer new look ahead point
            elif (d1 > self.Lr) and (d2 > self.Lr):
                b = True
                self.ind = j
                self.c_x = load.cx[j]
                self.c_y = load.cy[j]
            else:
                b = True
        return self.c_x, self.c_y

    # 4 adding white noise function to heading angle and vehicle position
    def white_noise(self, std):
        mean = 0.0          # 0
        std = std
        num_sample = 100.0
        # random values using normal distribution
        sample = np.random.normal(mean, std, size=num_sample)
        return sample

    def set_steering(self, states):  # 5 calculate the new required steering angle

        dx = self.c_x - states.x
        dy = self.c_y - states.y

        # print(dx)
        # print(dy)

        self.Ld = math.sqrt(dx**2 + dy**2)
        states.alpha = math.atan(dy / dx) - states.yaw

        # self.delta_ref = math.atan(
        #     2 * self.Ld * math.sin(states.alpha) / self.Ld)
        if (dx < 0) and (dy < 0):
            self.delta_ref = math.atan(
                2 * self.Ld * math.sin(states.alpha) / self.Ld)
        elif (dx < 0) and (dy > 0):
            self.delta_ref = math.atan(
                2 * self.Ld * math.sin(states.alpha) / self.Ld)
        elif (dx > 0) and (dy > 0):
            self.delta_ref = math.atan(
                2 * self.Ld * math.sin(-states.alpha) / self.Ld)
        else:
            self.delta_ref = math.atan(
                2 * self.Ld * math.sin(-states.alpha) / self.Ld)

        # print(math.degrees(self.delta_ref))
        return self.delta_ref, self.ind

    def steady_state_bias(self):  # this function insert steady state bias to servo
        self.delta_ref
        self.delta_ref = self.delta_ref + self.servo_bias
        return

    # def update_vehicle_position(states):  # 7 update vehicle states every loop with this function
        # states.x = states.x + dt*states.v * \
        #     math.cos(states.yaw) + \
        #     white_noise(0.1)  # std of 0.1 meter white noise
        # states.y = states.y + dt*states.v * \
        #     math.sin(states.yaw) + \
        #     white_noise(0.1)  # std of 0.1 meter white noise
        # return states

    def main(self):  # 10 run the whole simulation
        self.ind
        self.time
        # states = VehicleStates(st.x = x0, st.y = y0, st.v = v, st.yaw = yaw)
        # states = State(st.x = x0, st.y = y0, st.v = v, st.yaw = yaw)
        # states = rospy.Subscriber()

        # make the iterations with time delta of dt between every loop
        # while (time < simulation_time) and (ind < len(lp.cx.append)):


if __name__ == '__main__':
    rospy.init_node("pure_pursuit_point")
    r = rospy.Rate(30)
    st = State(x=-0.0, y=-0.0, yaw=-0.0, v=0.0)
    lp = LoadPose(file_name="path.csv")
    # print(len(lp.cx))
    pure_pursuit = PurePursuit(load=lp)

    rospy.Subscriber("/fake_odom", Odometry, st.odometryCallback)
    path_pub = rospy.Publisher("hojin_path", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)
    cmd_msg = stanleyMsg()
    if pure_pursuit.ind == 0:
        sta = pure_pursuit.closest_path_point(states=st, load=lp)

    while not rospy.is_shutdown():
        # print(ind)
        pure_pursuit.find_goal_point(states=st, load=lp)
        delta, _ = pure_pursuit.set_steering(states=st)
        cmd_msg.speed = 4.0
        cmd_msg.steer = delta
        cmd_msg.brake = 1
        cmd_pub.publish(cmd_msg)
        lp.pathPublish(pub=path_pub)
        # steady_state_bias()
        r.sleep()
