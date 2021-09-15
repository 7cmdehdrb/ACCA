#!/usr/bin/env python


import rospy
from std_msgs.msg import Int32


if __name__ == "__main__":
    rospy.init_node("test")

    pub = rospy.Publisher("test", Int32, queue_size=1)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        value = rospy.get_param("desired_speed", 1.0)

        msg = Int32()
        msg.data = value

        pub.publish(msg)

        r.sleep()
