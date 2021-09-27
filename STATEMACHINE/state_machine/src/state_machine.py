#!/usr/bin/env python

import sys
import rospy
import tf
import math as m
import time
from std_msgs.msg import Float32, Int32, Int32MultiArray
from path_planner.msg import stanleyMsg, obTF
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")
ODOMETRY_TOPIC = rospy.get_param("/odometry_topic", "/odom")
GLOBAL_PATH_FILE = rospy.get_param("/global_path_file", "path.csv")

GLOBAL_SPEED = rospy.get_param("/desired_speed", 3.0)
DYNAMIC_SPEED = rospy.get_param("/dynamic_speed", 1.0)
BACKWARD_SPEED = rospy.get_param("/backward_speed", 1.0)

WB = 1.040
PARKING_WAIT_TIME = 5
DELIVERY_WAIT_TIME = 5


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/path_planner/src")
    from global_stanley import GlobalStanley
    from selectTraffic import isTrafficLeft2, isTrafficStraight2
    from path_switcher import PathSwitcher
except ImportError as ie:
    print("PATH PLANNER IMPORT ERROR")
    print(ie)
    sys.exit()

try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils/")
    from state import State
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()

try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/cone_tracker/src")
    from estopTF import Dynamic
    from static_obstacles import StaticObstacles
except ImportError as ie:
    print("CONE TRACKER IMPORT ERROR")
    print(ie)
    sys.exit()


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/parking/src")
    from parking import Parking
except ImportError as ie:
    print("PARKING IMPORT ERROR")
    print(ie)
    sys.exit()


try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/delivery/src")
    from deliveryPy import Delivery
except ImportError as ie:
    print("DELIVERY IMPORT ERROR")
    print(ie)
    sys.exit()


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
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub, main_path_file=GLOBAL_PATH_FILE)
        self.estop_node = Dynamic(state=self.state)
        self.static_ob_node = StaticObstacles(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)
        self.parking_node = Parking(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)
        self.path_switcher_node = PathSwitcher(
            state=self.state, cmd_pub=self.cmd_pub, cmd_msg=self.cmd_msg, file_name="kcity_staticpath")

        self.delivery_A_node = Delivery(
            cmd_pub=self.cmd_pub, cmd_msg=self.cmd_msg, state=self.state, file_name="deliveryA3.csv")
        self.delivery_B_node = Delivery(
            cmd_pub=self.cmd_pub, cmd_msg=self.cmd_msg, state=self.state, file_name="deliveryB_K.csv")

        self.Mode = 0
        self.trafficLight = 0
        # self.trafficLine = TrafficStopLine(state=self.state)

        self.parkingCnt = 0
        self.deliveryCnt = 0

    def stateCallback(self, msg):
        data = msg.data

        if data != -1:

            ind = self.global_stanley_node.target_idx

            if 18000 < ind and ind < 22000:
                pass
            else:
                self.Mode = data

    def trafficLightCallback(self, msg):
        data = msg.data

        # 0: STOP 1: STRAIGHT 2: TURN LEFT

        self.trafficLight = data

    def resetDelivery(self):
        self.delivery_A_node.reset()
        self.delivery_B_node.reset()

        self.deliveryCnt = 0
        self.Mode = 1

        # print("DELIVERY RESET")

    def doEStop(self):
        self.cmd_msg.speed = 0.0
        self.cmd_msg.brake = 200

        # print(self.cmd_msg)

        self.cmd_pub.publish(self.cmd_msg)

    def doBrake(self, value):
        self.cmd_msg.speed = 0.0
        self.cmd_msg.brake = int(value)

        # print(self.cmd_msg)

        self.cmd_pub.publish(self.cmd_msg)


#################################Main Station#########################################
if __name__ == '__main__':
    rospy.init_node('state_machine')

    machine = Machine()

    t_pub = rospy.Publisher("TEST2", Path, queue_size=1)

    rospy.Subscriber(ODOMETRY_TOPIC, Odometry, machine.state.odometryCallback)
    rospy.Subscriber("/hdl_state", Int32, machine.stateCallback)
    rospy.Subscriber("/traffic_light", Int32,
                     machine.trafficLightCallback)
    rospy.Subscriber("/ob_TF", obTF, machine.estop_node.dynamicCallback)

    rate = rospy.Rate(50.0)

    # while not rospy.is_shutdown():

    #     if machine.state.x == 0.0 and machine.state.y == 0.0 and machine.state.yaw == 0.0 and machine.state.v == 0.0:
    #         pass
    #     else:
    #         break

    #     rate.sleep()

    while not rospy.is_shutdown():

        machine.delivery_B_node.load.pathPublish(t_pub)

        # print(machine.Mode)
        machine.global_stanley_node.main()
        machine.global_stanley_node.doPublishingMsg = False

        if machine.Mode == 0:
            # WILL USE LANE KEEPING
            # machine.global_stanley_node.doPublishingMsg = True
            pass

        if machine.Mode == 1:
            machine.global_stanley_node.doPublishingMsg = True

        # Static Obstacle Mode
        # elif machine.Mode == 2:
        #     machine.static_ob_node.start_point = [
        #         machine.state.x, machine.state.y]

        #     while not rospy.is_shutdown():

        #         find_goal = machine.static_ob_node.main()

        #         if find_goal is not True:
        #             # print("NO PATH!")
        #             machine.global_stanley_node.main()

        #         if machine.Mode != 2:
        #             break

        #         rate.sleep()

        # Dynamic Obstacle Mode
        # elif machine.Mode == 3:
        #     machine.global_stanley_node.setDesiredSpeed(DYNAMIC_SPEED)
        #     machine.estop_node.main()

        #     if machine.state.EStop is True:
        #         machine.doEStop()
        #     else:
        #         machine.global_stanley_node.main()

        #     machine.global_stanley_node.setDesiredSpeed(GLOBAL_SPEED)

        # Parking Mode
        if machine.Mode == 4:
            if machine.state.parkingFlag is True:
                machine.parking_node.main()

            else:

                while machine.parkingCnt < 50.0 * PARKING_WAIT_TIME and not rospy.is_shutdown():
                    machine.doBrake(120)
                    machine.parkingCnt += 1
                    rate.sleep()

                if machine.state.backwardFlag is True:
                    print("GO BACK")
                    machine.parking_node.goBack(speed=BACKWARD_SPEED)

                    while machine.parkingCnt < 50.0 * (PARKING_WAIT_TIME + 3) and not rospy.is_shutdown():
                        machine.doBrake(100)
                        machine.parkingCnt += 1
                        rate.sleep()

                machine.Mode = 1

        # Straight Traffic Mode
        if machine.Mode == 5:
            if isTrafficStraight2(machine.trafficLight) is False:
                machine.doBrake(80)
            else:
                machine.global_stanley_node.doPublishingMsg = True
                # machine.global_stanley_node.main()

        # Left Traffic Mode
        if machine.Mode == 6:
            if isTrafficLeft2(machine.trafficLight) is False:
                machine.doBrake(80)

            else:
                machine.global_stanley_node.doPublishingMsg = True
                # machine.global_stanley_node.main()

        if machine.Mode == 7:
            machine.delivery_A_node.main()
            # print(machine.delivery_A_node.target_idx)

            if machine.delivery_A_node.isEnd is True:
                while machine.deliveryCnt < 30.0 * DELIVERY_WAIT_TIME and not rospy.is_shutdown():
                    machine.doBrake(90)
                    machine.deliveryCnt += 1
                    rate.sleep()

                machine.resetDelivery()

        if machine.Mode == 8:
            machine.delivery_B_node.main()

            if machine.delivery_B_node.isEnd is True:
                while machine.deliveryCnt < 30.0 * DELIVERY_WAIT_TIME and not rospy.is_shutdown():
                    machine.doBrake(90)
                    machine.deliveryCnt += 1
                    rate.sleep()

                machine.resetDelivery()

        # Static - main
        if machine.Mode == 2:
            machine.path_switcher_node.main()

        print(machine.cmd_msg)
        print("idx: " + str(machine.global_stanley_node.target_idx))
        print("mode: " + str(machine.Mode))

        rate.sleep()
