#!/usr/bin/env python

import sys
import rospy
import math as m
import time
from std_msgs.msg import Float32, Int32
from path_planner.msg import stanleyMsg, WTF
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/path_planner/src")
    from global_stanley import GlobalStanley
    from rrt_star_node import RRTStarPath
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


# import cv2


# try:

#     # from Lane_Detection import LaneDetection

# except Exception as ex:
#     print(ex)

#################################Library##############################################

WB = 1.040


class RRTStarState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(RRTStarState, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.EStop = False

        self.rear_x = 0.0
        self.rear_y = 0.0

    def updateRear(self):
        self.rear_x = self.x - ((WB / 2) * m.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * m.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return m.hypot(dx, dy)


class Machine():
    def __init__(self):
        self.cmd_pub = rospy.Publisher(
            "/Control_msg", stanleyMsg, queue_size=1)
        self.cmd_msg = stanleyMsg()

        self.state = RRTStarState(x=-7.487, y=-5.565, yaw=-2.182, v=0.0)
        self.global_stanley = GlobalStanley(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)
        self.rrt_star_node = RRTStarPath(
            state=self.state, cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)
        self.estop_node = Dynamic(state=self.state)

        # self.Lane_Detection = LaneDetection(
        #     cmd_msg=self.cmd_msg, cmd_publisher=self.cmd_pub)

        self.Mode = -1

    def stateCallback(self, msg):
        # self.Mode = msg.data
        pass

    def doEStop(self):
        self.cmd_msg.speed = 0.0
        self.cmd_msg.brake = 200

        self.cmd_pub.publish(self.cmd_msg)


#################################Main Station#########################################
if __name__ == '__main__':
    rospy.init_node('state_machine')

    machine = Machine()

    # rospy.Subscriber("/usb_cam/image_raw/", Image,
    #                  machine.Lane_Detection.img_callback)

    rospy.Subscriber("/hdl_state", Int32, machine.stateCallback)
    rospy.Subscriber("/odom", Odometry, machine.state.odometryCallback)
    rospy.Subscriber("/ob_TF", WTF, machine.estop_node.dynamicCallback)

    # time.sleep(1)
    # error_pub = rospy.Publisher("/Error", lane_error, queue_size=1)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        # rospy.loginfo(machine.Mode)

        # while machine.Mode == -1:
        #     machine.Lane_Detection.TurnOn(
        #         9.0, image_raw=machine.Lane_Detection.image_raw)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        #     rate.sleep()
        # # machine.Lane_Detection.cap.release()
        # cv2.destroyAllWindows()

        # Global Path Mode

        print(machine.cmd_msg)
        # print(machine.Mode)

        if machine.Mode == -1:
            machine.global_stanley.main()

        # Local Path Mode
        if machine.Mode == 2:
            machine.rrt_star_node.main()

        # Dynamic Obstacle Mode
        if machine.Mode == 3:
            machine.estop_node.main()

            if machine.state.EStop is True:
                machine.doEStop()
            else:
                machine.global_stanley.main()

        rate.sleep()
