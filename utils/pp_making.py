#!/usr/bin/env python

import numpy as np
import math
import csv
import os
import sys
import rospy
from nav_msgs.msg import Odometry, Path
from path_planner.msg import stanleyMsg

dt = 0.05  # [s]
time = 0.0  # current simulation time
simulation_time = 7200  # [s]
S_t = 0.2  # Servo delay 200 [msec]
L = 2.020  # length of platform [meter]
v = 10  # car velocity [m/s]
K = 5  # proportion controller for look ahead [sec]
Lr = K*v + L  # [m] Ld required gain
Ld = 2.0  # actual look ahead length
ind = 0.0  # global index
max_delta = 0.4712  # max wheel angle [rad]______27 degree
min_delta = -0.4712
delta_ref = 0.0  # required delta [rad]
last_delta_ref = 0.0
delay = 0.2  # [s]
a = 0.0  # for servo dynamic function
c_x = 0.0  # look ahead x point
c_y = 0.0  # look ahead y point
global i  # lidar dataset index
servo_bias = 0.03  # [rad]


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    from loadPose import LoadPose
except Exception as ex:
    try:
        sys.path.insert(0, "/home/hojin/catkin_ws/src/ACCA/utils")
        from state import State
        from loadPose import LoadPose
    except Exception as ex:

        print("UTIL IMPORT ERROR")
        print(ex)
        sys.exit()


def closest_path_point(states, load):  # 2 finding the closes path point
    global ind
    i = 0  # index
    cpp = 0  # close path point [m]

    while (i < len(load.cx)):  # searching the whole path points
        dx = load.cx[i] - states.x
        dy = load.cy[i] - states.y
        d = math.sqrt(dx**2 + dy**2)
        if (i == 0) or (cpp > d):
            cpp = d
            ind = i
        i += 1
    return states


def find_goal_point(states, load):  # 3 find the goal point
    global ind
    global c_x, c_y
    j = ind  # index
    b = False  # boolean True when we find look ahead point
    while (j < len(load.cx)) and (b is False):
        dx1 = load.cx[j] - states.x
        dy1 = load.cy[j] - states.y
        d1 = math.sqrt(dx1**2 + dy1**2)
        dx2 = load.cx[j+1] - states.x
        dy2 = load.cy[j+1] - states.y
        d2 = math.sqrt(dx2**2 + dy2**2)
        if (d1 < Lr) and (d2 < Lr):  # in case the current and next points are too close
            j += 1
            c_x = load.cx[j+1]
            c_y = load.cy[j+1]
        # in case the current point close than the required look ahead length and the next point is too far
        elif ((d1 < Lr) and (d2 > Lr)):
            b = True
            c_x = (load.cx[j+1] + load.cx[j])/2  # interpolation
            c_y = (load.cy[j+1] + load.cy[j])/2  # interpolation
            ind = j+1
        elif (d1 > Lr) and (d2 > Lr):  # in case there is no closer new look ahead point
            b = True
            ind = j
            c_x = load.cx[j]
            c_y = load.cy[j]
        else:
            b = True
    # print(c_x, c_y)
    return c_x, c_y


def white_noise(std):  # 4 adding white noise function to heading angle and vehicle position
    mean = 0
    std = std
    num_sample = 1
    # random values using normal distribution
    sample = np.random.normal(mean, std, size=num_sample)
    return sample


def set_steering(states):  # 5 calculate the new required steering angle
    global Ld
    global ind
    global delta_ref
    dx = c_x - states.x
    dy = c_y - states.y

    Ld = math.sqrt(dx**2 + dy**2)

    states.alpha = math.atan(dy / dx) - states.yaw

    delta_ref = math.atan(2*L*math.sin(states.alpha) / Ld)

    # print(delta_ref)
    print(math.degrees(delta_ref))
    return delta_ref, ind


def steady_state_bias():  # this function insert steady state bias to servo
    global delta_ref
    delta_ref = delta_ref + servo_bias
    return


# def update_vehicle_position(states):  # 7 update vehicle states every loop with this function
    # states.x = states.x + dt*states.v * \
    #     math.cos(states.yaw) + \
    #     white_noise(0.1)  # std of 0.1 meter white noise
    # states.y = states.y + dt*states.v * \
    #     math.sin(states.yaw) + \
    #     white_noise(0.1)  # std of 0.1 meter white noise
    # return states


def main():  # 10 run the whole simulation
    global ind
    global time
    # states = VehicleStates(st.x = x0, st.y = y0, st.v = v, st.yaw = yaw)
    # states = State(st.x = x0, st.y = y0, st.v = v, st.yaw = yaw)
    states = rospy.Subscriber()

    # make the iterations with time delta of dt between every loop
    # while (time < simulation_time) and (ind < len(lp.cx.append)):


if __name__ == '__main__':
    rospy.init_node("pure_pursuit_point")
    r = rospy.Rate(30)
    st = State(x=-0.0, y=-0.0, yaw=-0.0, v=0.0)
    lp = LoadPose(file_name="path.csv")
    rospy.Subscriber("/fake_odom", Odometry, st.odometryCallback)
    path_pub = rospy.Publisher("hojin_path", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)
    cmd_msg = stanleyMsg()

    if ind == 0:
        st = closest_path_point(states=st, load=lp)

    while not rospy.is_shutdown():
        find_goal_point(states=st, load=lp)
        delta, _ = set_steering(states=st)

        cmd_msg.speed = 10.0
        cmd_msg.steer = delta
        cmd_msg.brake = 1

        cmd_pub.publish(cmd_msg)
        lp.pathPublish(pub=path_pub)

        # steady_state_bias()
        r.sleep()
