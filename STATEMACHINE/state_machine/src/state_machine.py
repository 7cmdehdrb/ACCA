#!/usr/bin/env python

import sys
import rospy
import tf
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
    from static_obstacles import StaticObstacles
except ImportError as ie:
    print("CONE TRACKER IMPORT ERROR")
    print(ie)
    sys.exit()


try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/parking/src")
    from parking import Parking
except ImportError as ie:
    print("PARKING IMPORT ERROR")
    print(ie)
    sys.exit()


WB = 1.040
PARKING_WAIT_TIME = 10


class Machine():
    def __init__(self):
        self.cmd_pub = rospy.Publisher(
            "/Control_msg", stanleyMsg, queue_size=1)
        self.cmd_msg = stanleyMsg()

        self.state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

        self.tf_node = tf.TransformListener()

        self.state.parkingFlag = True
        self.state.backwardFlag = True

        self.global_stanley_node = GlobalStanley(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub, main_path_file="static_path.csv")
        self.estop_node = Dynamic(state=self.state)
        self.static_ob_node = StaticObstacles(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub, start_point=[-14.0730895996, -16.6290416718])
        self.parking_node = Parking(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)

        self.Mode = -1

        self.parkingCnt = 0

    def stateCallback(self, msg):
        self.Mode = msg.data
        # pass

    def doEStop(self):
        self.cmd_msg.speed = 0.0
        self.cmd_msg.brake = createGoalPoint200

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

        if machine.state.x == 0.0 and machine.state.y == 0.0 and machine.state.yaw == 0.0 and machine.state.v == 0.0:
            pass
        else:
            break

        rate.sleep()

    while not rospy.is_shutdown():
        # print(machine.Mode)

        if machine.Mode == -1:
            machine.global_stanley_node.main()

        # Static Obstacle Mode
        if machine.Mode == 2:
            machine.static_ob_node.start_point = [
                machine.state.x, machine.state.y]

            while not rospy.is_shutdown():

                find_goal = machine.static_ob_node.main()

                if find_goal is not True:
                    # print("NO PATH!")
                    machine.global_stanley_node.main()

                # PLEASE ADD PP CONTROL

                if machine.Mode != 2:
                    break

                rate.sleep()

        # Dynamic Obstacle Mode
        if machine.Mode == 3:
            machine.estop_node.main()

            if machine.state.EStop is True:
                machine.doEStop()
            else:
                machine.global_stanley_node.main()

        # Parking Mode
        if machine.Mode == 4:
            if machine.state.parkingFlag is True:
                machine.parking_node.main()

            else:

                while machine.parkingCnt < 30.0 * PARKING_WAIT_TIME:
                    machine.parking_node.main()
                    machine.parkingCnt += 1
                    rate.sleep()

                if machine.state.backwardFlag is True:
                    print("BACK BACK BACK")
                    machine.parking_node.goBack(speed=0.5)

                machine.global_stanley_node.main()

        rate.sleep()
