#!/usr/bin/env python

import rospy


VALUE = rospy.get_param("/test", [0, 0, 0, 0])


if __name__ == "__main__":
    rospy.init_node("test_node")

    r = rospy.Rate(5.0)
    while not rospy.is_shutdown():

        print(type(VALUE))

        r.sleep()
