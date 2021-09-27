#! /usr/bin/env python

import rospy
from MaRRTPathPlanNode import MaRRTPathPlanNode

def main():
    rospy.init_node('MaRRTPathPlanNode2')
    maRRTPathPlanNode = MaRRTPathPlanNode()

    rate = rospy.Rate(100.0) # big amount on purpose

    while not rospy.is_shutdown():
        maRRTPathPlanNode.sampleTree()
        rate.sleep()

    # Spin until ctrl + c
    # rospy.spin()

if __name__ == '__main__':
    main()
