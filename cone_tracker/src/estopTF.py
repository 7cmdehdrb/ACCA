#!/usr/bin/env python

import sys
import rospy
from path_planner.msg import WTF

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
except Exception as ex:
    print(ex)


class Dynamic(object):
    def __init__(self, state):
        super(Dynamic, self).__init__()
        self.state = state

        self.obs = WTF()

    def dynamicCallback(self, msg):
        self.obs = msg

    def main(self):
        if (self.obs.front_left == 1) or (self.obs.front_right == 1):
            self.state.EStop = True
        else:
            self.state.EStop = False


if __name__ == "__main__":
    rospy.init_node("estop")

    dobs = Dynamic()
    state = State()

    rospy.Subscriber("/ob_TF", WTF, dobs.dynamicCallback)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        dobs.main()
        r.sleep()
