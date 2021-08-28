#!/usr/bin/env python

import numpy as np
import math as m

# Parameters
k = 0.1  # look forward gain
Lfc = 0.5  # [m] look-ahead distance
WB = 1.04  # [m] wheel base of vehicle


def pure_pursuit_control(state, goal):
    Lf = np.hypot(goal[0] - state.x, goal[1] - state.y)

    if Lf == 0.0:
        print("ERROR!!")
        return 0.0

    tx = goal[0]
    ty = goal[1]

    alpha = m.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    delta = m.atan2(2.0 * WB * m.sin(alpha) / Lf, 1.0)

    return delta


# def pure_pursuit_control(state, goal):
#     Ld = np.hypot(state.x - goal[0], state.y - goal[1])

#     G_Vec = np.array([goal[0] - state.x, goal[1] - state.y])
#     F_Vec = np.array([m.cos(state.yaw), m.sin(state.yaw)])

#     abs_GVec = np.hypot(G_Vec[0], G_Vec[1])
#     abs_FVec = np.hypot(F_Vec[0], F_Vec[1])

#     dot_product = np.dot(G_Vec, F_Vec)

#     alpha = m.acos(dot_product / (abs_GVec * abs_FVec))

#     e = Ld * (m.sin(alpha))

#     theta = m.atan((2 * WB * e) / (m.pow(Ld, 2)))

#     return theta
