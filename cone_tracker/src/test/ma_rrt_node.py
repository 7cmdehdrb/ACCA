#!/usr/bin/env python

import sys
import os

import rospy

import ma_rrt

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

# For odometry message
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Matrix/Array library
import numpy as np
import math

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
except Exception as ex:
    print(ex)


class MaRRTPathPlanNode:
    # All variables, placed here are static
    def __init__(self, map):
        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.poses = []

        self.map = map
        self.savedWaypoints = []
        self.preliminaryLoopClosure = False
        self.loopClosure = False

        self.rrt = None

        self.filteredBestBranch = []
        self.discardAmount = 0

    # def __del__(self):
    #     print('MaRRTPathPlanNode: Destructor called.')

    def sampleTree(self):
        if not self.map:
            return

        frontConesDist = 20
        frontCones = self.getFrontConeObstacles(self.map, frontConesDist)

        coneObstacleSize = 1.0
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.5
        for cone in frontCones:
            coneObstacleList.append((cone.x, cone.y, coneObstacleSize))

            coneDist = self.dist(self.carPosX, self.carPosY, cone.x, cone.y)

            if coneDist > frontConesDist * coneTargetsDistRatio:
                rrtConeTargets.append((cone.x, cone.y, coneObstacleSize))

        # Set Initial parameters
        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000
        planDistance = 10
        expandDistance = 1.0
        expandAngle = 20

        # rrt planning
        rrt = ma_rrt.RRT(start, planDistance, obstacleList=coneObstacleList, expandDis=expandDistance,
                         turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets=rrtConeTargets)
        nodeList, leafNodes = rrt.Planning()

        frontConesBiggerDist = 15
        largerGroupFrontCones = self.getFrontConeObstacles(
            self.map, frontConesBiggerDist)

        # BestBranch
        bestBranch = self.findBestBranch(
            leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance)

        if bestBranch:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)
        else:
            print("CAN NOT FIND BEST BRANCH")

        return bestBranch

    def dist(self, x1, y1, x2, y2, shouldSqrt=True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        # Return true if line segments a1a2 and b1b2 intersect
        # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        return self.ccw(a1, b1, b2) != self.ccw(a2, b1, b2) and self.ccw(a1, a2, b1) != self.ccw(a1, a2, b2)

    def ccw(self, A, B, C):
        # if three points are listed in a counterclockwise order.
        # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        everyPointDistChangeLimit = 2.0
        newPointFilter = 0.2
        maxDiscardAmountForReset = 2

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) **
                                 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit:  # changed too much, skip this branch
                    shouldDiscard = True
                    self.discardAmount += 1

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0
                        self.filteredBestBranch = list(bestBranch)
                    break

                changeRate += (everyPointDistChangeLimit - dist)

            if not shouldDiscard:
                for i in range(len(bestBranch)):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (
                        1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (
                        1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0

        return list(self.filteredBestBranch)  # return copy

    def findBestBranch(self, leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance):
        if not leafNodes:
            print("NO LEAFNODES")
            return

        coneDistLimit = 4.0
        coneDistanceLimitSq = coneDistLimit * coneDistLimit

        bothSidesImproveFactor = 3
        minAcceptableBranchRating = 50  # fits good fsg18

        leafRatings = []
        for leaf in leafNodes:
            branchRating = 0
            node = leaf
            while node.parent is not None:
                nodeRating = 0

                leftCones = []
                rightCones = []

                for cone in largerGroupFrontCones:
                    coneDistSq = ((cone.x - node.x) ** 2 +
                                  (cone.y - node.y) ** 2)

                    if coneDistSq < coneDistanceLimitSq:
                        actualDist = math.sqrt(coneDistSq)

                        if actualDist < coneObstacleSize:
                            continue

                        nodeRating += (coneDistLimit - actualDist)

                        if self.isLeftCone(node, nodeList[node.parent], cone):
                            leftCones.append(cone)
                        else:
                            rightCones.append(cone)

                if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                    nodeRating /= bothSidesImproveFactor

                if (len(leftCones) > 0 and len(rightCones) > 0):
                    nodeRating *= bothSidesImproveFactor

                nodeFactor = (node.cost - expandDistance) / \
                    (planDistance - expandDistance) + 1

                branchRating += nodeRating * nodeFactor

                node = nodeList[node.parent]

            leafRatings.append(branchRating)

        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]

        if maxRating < minAcceptableBranchRating:
            print("RATING ERROR")
            return

        reverseBranch = []
        reverseBranch.append(node)
        while node.parent is not None:
            node = nodeList[node.parent]
            reverseBranch.append(node)

        directBranch = []
        for n in reversed(reverseBranch):
            directBranch.append(n)

        return directBranch

    def isLeftCone(self, node, parentNode, cone):
        # //((b.X - a.X)*(cone.Y - a.Y) - (b.Y - a.Y)*(cone.X - a.X)) > 0;
        return ((node.x - parentNode.x) * (cone.y - parentNode.y) - (node.y - parentNode.y) * (cone.x - parentNode.x)) > 0

    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()

        headingVectorOrt = [-headingVector[1], headingVector[0]]

        behindDist = 0.5
        carPosBehindPoint = [self.carPosX - behindDist *
                             headingVector[0], self.carPosY - behindDist * headingVector[1]]

        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map.cones:
            if (headingVectorOrt[0] * (cone.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.x - carPosBehindPoint[0])) < 0:
                if ((cone.x - self.carPosX) ** 2 + (cone.y - self.carPosY) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [
                             math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, map, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in map.cones:
            if ((cone.x - x) ** 2 + (cone.y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList


class Edge():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt(
            (self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt(
            (self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
                or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1, 2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2, 2)) + ")"

    def __repr__(self):
        return str(self)


class Map(object):
    def __init__(self):
        super(Map, self).__init__()

        self.cones = []
        self.obstacles = PoseArray()

        self.distance_threshhold = 0.5

    def obstacleCallback(self, msg):
        self.obstacles = msg
        self.updateMap()

    def updateMap(self):
        obstacles = self.obstacles.poses
        new_cones = []

        # real-time obstacles
        for obstacle in obstacles:
            x = obstacle.position.x
            y = obstacle.position.y

            new_cone = Cone(x=x, y=y, r=1.0)

            new_cones.append(new_cone)

        for new_cone in new_cones:

            min_distance = float("inf")

            for cone in self.cones:
                distance = np.hypot(new_cone.x - cone.x, new_cone.y - cone.y)

                if distance < min_distance:
                    min_distance = distance

            if min_distance < self.distance_threshhold:
                self.cones.append(new_cone)


class Cone(object):
    def __init__(self, x=0.0, y=0.0, r=0.0):
        super(Cone, self).__init__()

        self.x = x
        self.y = y
        self.r = r


if __name__ == '__main__':
    rospy.init_node("ma_rrt_node")

    map = Map()

    map.cones = [
        Cone(x=1.0, y=-3.0),
        Cone(x=1.0, y=3.0),
        Cone(x=3.0, y=-3.0),
        Cone(x=3.0, y=3.0),
        Cone(x=6.0, y=-3.0),
        Cone(x=6.0, y=3.0),
        Cone(x=9.0, y=-3.0),
        Cone(x=9.0, y=3.0),
        Cone(x=12.0, y=-3.0),
        Cone(x=12.0, y=3.0),
        # Cone(x=16.0, y=-3.0),
        # Cone(x=16.0, y=3.0),
    ]

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    maNode = MaRRTPathPlanNode(map=map)

    rospy.Subscriber("transformed_obstacles", PoseArray,
                     callback=map.obstacleCallback)

    rospy.Subscriber("/odom", Odometry, callback=state.odometryCallback)

    path_pub = rospy.Publisher("/rrt_star_path", Path, queue_size=1)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        best_branch = maNode.sampleTree()

        # maNode.carPosX = state.x
        # maNode.carPosY = state.y
        # maNode.carPosYaw = state.yaw

        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"
        path.poses = []

        if not best_branch:
            path.poses = maNode.poses
        else:

            """ result """

            poses = []

            for node in best_branch:
                print(node.x, node.y, node.yaw)

                p = PoseStamped()

                p.header.stamp = rospy.Time.now()
                p.header.frame_id = "odom"

                p.pose.position.x = node.x
                p.pose.position.y = node.y
                p.pose.position.z = 0.0

                quat = quaternion_from_euler(0.0, 0.0, node.yaw)

                p.pose.orientation.x = quat[0]
                p.pose.orientation.y = quat[1]
                p.pose.orientation.z = quat[2]
                p.pose.orientation.w = quat[3]

                poses.append(p)

            path.poses = poses
            maNode.poses = poses

        path_pub.publish(path)

        r.sleep()
