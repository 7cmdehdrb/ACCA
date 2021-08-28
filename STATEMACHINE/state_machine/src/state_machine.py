#!/usr/bin/env python

import sys
import rospy
import math as m
import time
from std_msgs.msg import Float32, Int32
from path_planner.msg import stanleyMsg, obTF
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/path_planner/src")
    from global_stanley import GlobalStanley
except ImportError as ie:
    print("PATH PLANNER IMPORT ERROR")
    print(ie)
    sys.exit()

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/cone_tracker/src")
    from estopTF import Dynamic
except ImportError as ie:
    print("CONE TRACKER IMPORT ERROR")
    print(ie)
    sys.exit()


WB = 1.040


class Machine():
    def __init__(self):
        self.cmd_pub = rospy.Publisher(
            "/Control_msg", stanleyMsg, queue_size=1)
        self.cmd_msg = stanleyMsg()

        self.state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
        self.global_stanley = GlobalStanley(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)
        self.estop_node = Dynamic(state=self.state)

        self.Mode = -1

    def stateCallback(self, msg):
        self.Mode = msg.data
        # pass

    def doEStop(self):
        self.cmd_msg.speed = 0.0
        self.cmd_msg.brake = 200

        self.cmd_pub.publish(self.cmd_msg)


#################################Main Station#########################################
if __name__ == '__main__':
    rospy.init_node('state_machine')

    machine = Machine()

    rospy.Subscriber("/hdl_state", Int32, machine.stateCallback)
    rospy.Subscriber("/odom", Odometry, machine.state.odometryCallback)
    rospy.Subscriber("/ob_TF", obTF, machine.estop_node.dynamicCallback)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        print(machine.Mode)

        if machine.Mode == -1:
            machine.global_stanley.main()

        # Dynamic Obstacle Mode
        if machine.Mode == 3:
            machine.estop_node.main()

            if machine.state.EStop is True:
                machine.doEStop()
            else:
                machine.global_stanley.main()

        rate.sleep()
