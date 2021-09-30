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
DELIVERY_A_PATH = rospy.get_param("/delivery_A", "deliveryA3.csv")
DELIVERY_B_PATH = rospy.get_param("/delivery_B", "deliveryB_K.csv")
SWITCHER_PATH = rospy.get_param("/switcher_path", "kcity_staticpath")

GLOBAL_SPEED = rospy.get_param("/desired_speed", 5.0)
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
    from loadPose import LoadPose
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


if __name__ == "__main__":
    rospy.init_node("test")

    l0 = LoadPose("ssu_switcher20.csv")
    l1 = LoadPose("ssu_switcher21.csv")
    l2 = LoadPose("ssu_parking22.csv")

    pub0 = rospy.Publisher("static0", Path, queue_size=1)
    pub1 = rospy.Publisher("static1", Path, queue_size=1)
    pub2 = rospy.Publisher("static2", Path, queue_size=1)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        l0.pathPublish(pub0)
        l1.pathPublish(pub1)
        # l2.pathPublish(pub2)
        r.sleep()
