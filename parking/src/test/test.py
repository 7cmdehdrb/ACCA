#!/usr/bin/env python

import sys
import rospy

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()


if __name__ == "__main__":
    rospy.init_node("test_node")

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    state.my_var = 10

    print(state.my_var)
