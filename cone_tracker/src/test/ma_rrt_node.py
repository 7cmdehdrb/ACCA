#!/usr/bin/env python

import sys
import os

import rospy

import ma_rrt

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from path_planner.msg import stanleyMsg

# For odometry message
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Matrix/Array library
import numpy as np
import math

try:
    sys.path.insert(0, "/home/acca/catkin_ws/src/utils")
    from state import State
    from cubic_spline_planner import calc_spline_course
    from stanley import Stanley
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

        frontConesDist = 10
        frontCones = self.getFrontConeObstacles(self.map, frontConesDist)

        coneObstacleSize = 0.3
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.0
        for cone in frontCones:
            coneObstacleList.append((cone.x, cone.y, coneObstacleSize))

            coneDist = self.dist(self.carPosX, self.carPosY, cone.x, cone.y)

            if coneDist > frontConesDist * coneTargetsDistRatio:
                rrtConeTargets.append((cone.x, cone.y, coneObstacleSize))

        # Set Initial parameters
        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000
        planDistance = 5
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
            pass
            # print("CAN NOT FIND BEST BRANCH")

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
            # print("NO LEAFNODES")
            return

        coneDistLimit = 3.0
        coneDistanceLimitSq = coneDistLimit * coneDistLimit

        bothSidesImproveFactor = 3
        minAcceptableBranchRating = 60  # fits good fsg18

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
            # print("RATING ERROR")
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


# class Edge():
#     def __init__(self, x1, y1, x2, y2):
#         self.x1 = x1
#         self.y1 = y1
#         self.x2 = x2
#         self.y2 = y2
#         self.intersection = None

#     def getMiddlePoint(self):
#         return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

#     def length(self):
#         return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

#     def getPartsLengthRatio(self):
#         import math

#         part1Length = math.sqrt(
#             (self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
#         part2Length = math.sqrt(
#             (self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

#         return max(part1Length, part2Length) / min(part1Length, part2Length)

#     def __eq__(self, other):
#         return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
#                 or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

#     def __str__(self):
#         return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1, 2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2, 2)) + ")"

#     def __repr__(self):
#         return str(self)


class Map(object):
    def __init__(self):
        super(Map, self).__init__()

        self.cones = []
        self.obstacles = PoseArray()

        self.distance_threshhold = 1.0

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

        if len(self.cones) == 0:
            self.cones = new_cones
            return

        for new_cone in new_cones:

            min_distance = float("inf")

            for cone in self.cones:
                distance = np.hypot(new_cone.x - cone.x, new_cone.y - cone.y)

                if abs(distance) < min_distance:
                    min_distance = distance

            if min_distance > self.distance_threshhold:
                # print(min_distance)
                self.cones.append(new_cone)

        # print("DETECTED CONES: " + str(len(self.cones)))

    def printCones(self):
        for cone in self.cones:
            print(cone.x, cone.y)

        Pose().position

        print()

    def publishCones(self, publisher):
        msg = PoseArray()

        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()

        msg.poses = []

        for cone in self.cones:
            pose = Pose()

            pose.position.x = cone.x
            pose.position.y = cone.y

            pose.orientation.w = 1.0

            msg.poses.append(pose)

        publisher.publish(msg)


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
        Cone(-0.5556434796123261, 2.6459947271752977, 1.0),
        Cone(0.9315238877976841, 4.865960020518196, 1.0),
        Cone(2.5344793606150904, 3.345932790396864, 1.0),
        Cone(1.6022718096583017, 1.942950835400901, 1.0),
        Cone(4.284745579175762, 3.6319982573884415, 1.0),
        Cone(3.1635045656961225, 5.437523601213728, 1.0),
        Cone(0.7095897238910186, 3.876773792145854, 1.0),
        Cone(5.682834422802511, 3.2049009570756066, 1.0),
        Cone(3.756053224213485, 2.7780282660368165, 1.0),
        Cone(5.631384097194796, 5.543669593677949, 1.0),
        Cone(6.899484660380155, 2.9194508775352697, 1.0),
        Cone(8.05043234438493, 5.37893277028548, 1.0),
        Cone(7.907191042338056, 2.6272800016742197, 1.0),
        Cone(9.298409903256445, 4.7597758804455985, 1.0),
        Cone(9.037612676495812, 1.7194122203585072, 1.0),
        Cone(10.446639945834566, 3.6969661475435185, 1.0),
        Cone(9.282152791001609, 0.37640548642868055, 1.0),
        Cone(11.421169502532656, 1.7634841883468155, 1.0),
        Cone(12.159878397843787, -0.4145257628522456, 1.0),
        Cone(9.10516454255917, -1.3647771295166593, 1.0),
        Cone(11.536128649968916, -2.6151349853758368, 1.0),
        Cone(8.440632133651743, -2.6574885919428013, 1.0),
        Cone(12.436836270255109, 0.6496168037859197, 1.0),
        Cone(10.812534887765665, -4.293516814806486, 1.0),
        Cone(7.503455546210312, -3.5133029527457187, 1.0),
        Cone(12.226984941148698, -1.690940156081655, 1.0),
        Cone(9.270858671193107, -5.740230217506533, 1.0),
        Cone(6.570009300894069, -4.399096336692841, 1.0),
        Cone(7.100445464335067, -7.126738628758638, 1.0),
        Cone(5.40112282229279, -4.828865132628525, 1.0),
        Cone(5.37794501991534, -7.367919686457235, 1.0),
        Cone(3.740134529923825, -4.834221310594402, 1.0),
        Cone(3.296518584789344, -7.301831609653052, 1.0),
        Cone(2.608201353482719, -4.129272332002811, 1.0),
        Cone(1.5231289498091778, -6.290717210939585, 1.0),
        Cone(2.380163996153594, -2.731360093200478, 1.0),
        Cone(0.39070964364215577, -4.828919391157544, 1.0),
        Cone(-0.20848148709559178, -2.5111458922569634, 1.0),
        Cone(1.9069278201249449, -0.645274460840894, 1.0),
        Cone(-0.2320139949847695, -1.1088634198348575, 1.0),
        Cone(-0.17599811488810402, -3.5438405402963404, 1.0),
        Cone(-0.3615930387800479, 0.6451772327527041, 1.0),
        # Cone(1.963514421901607, 0.3763897884859948, 1.0),
        # Cone(1.589976037260068, -1.7459873015786007, 1.0),
    ]

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    stanley = Stanley()
    maNode = MaRRTPathPlanNode(map=map)

    rospy.Subscriber("/transform_obstacles", PoseArray,
                     callback=map.obstacleCallback)

    rospy.Subscriber("/fake_odom", Odometry,
                     callback=state.odometryCallback)

    path_pub = rospy.Publisher("/rrt_star_path", Path, queue_size=1)
    test_pub = rospy.Publisher("test_pub", PoseArray, queue_size=1)
    cmd_pub = rospy.Publisher("/Control_msg", stanleyMsg, queue_size=1)
    cmd_msg = stanleyMsg()

    odom_x = [0.0]
    odom_y = [0.0]
    last_position = [0.0, 0.0]

    cx = []
    cy = []
    target_idx = 0
    is_loop_close = False
    is_start = False
    loop_flag = False

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        """ STATE """

        dist = np.hypot(state.x, state.y)
        d_dist = np.hypot(
            state.x - last_position[0], state.y - last_position[1])

        if is_loop_close is not True:
            if is_start is False:
                if dist > 3.0:
                    is_start = True
            else:
                if dist < 3.0:
                    is_loop_close = True

            if d_dist > 0.5:
                odom_x.append(state.x)
                odom_y.append(state.y)
                last_position = [state.x, state.y]
                print("ADD!")

            maNode.carPosX = state.x
            maNode.carPosY = state.y
            maNode.carPosYaw = state.yaw

            best_branch = maNode.sampleTree()

            # print(len(map.cones))
            # map.printCones()
            map.publishCones(test_pub)
            # print(best_branch)

            if not best_branch:
                pass
                # path.poses = maNode.poses

            else:
                temp_cx = []
                temp_cy = []

                for node in best_branch:
                    temp_cx.append(node.x)
                    temp_cy.append(node.y)

                cx = temp_cx
                cy = temp_cy

            if len(cx) < 2:
                continue

            xs, ys, yaws, _, _ = calc_spline_course(cx, cy, ds=0.1)

            # target_idx = stanley.calc_target_index(state, xs, ys)

            try:

                di, target_idx = stanley.stanley_control(
                    state=state, cx=xs, cy=ys, cyaw=yaws, last_target_idx=2)

                cmd_msg.speed = 1.0
                cmd_msg.steer = -di
                cmd_msg.brake = 1

                cmd_pub.publish(cmd_msg)

            except Exception as ex:
                print(ex)

            """ Publish Path """

            path = Path()

            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "odom"

            path.poses = []

            for i in range(len(xs)):
                # print(node.x, node.y, node.yaw)

                p = PoseStamped()

                p.header.stamp = rospy.Time.now()
                p.header.frame_id = "odom"

                p.pose.position.x = xs[i]
                p.pose.position.y = ys[i]
                p.pose.position.z = 0.0

                quat = quaternion_from_euler(0.0, 0.0, yaws[i])

                p.pose.orientation.x = quat[0]
                p.pose.orientation.y = quat[1]
                p.pose.orientation.z = quat[2]
                p.pose.orientation.w = quat[3]

                path.poses.append(p)

            path_pub.publish(path)

            """ Publish Path End """

        else:
            if loop_flag is not True:
                print(len(odom_x), len(odom_y))
                new_xs, new_ys, new_yaws, _, _ = calc_spline_course(
                    odom_x, odom_y, 0.1)
                # target_idx, _ = stanley.calc_target_index(
                #     state, xs[:100], ys[:100])
                target_idx = 0

                loop_flag = True
                print("LOOP FLAG TRUE")

            else:

                try:
                    # stanley.setCGain(1000.0)
                    print(target_idx, len(new_xs))

                    if target_idx == len(new_xs) - 1:
                        target_idx = 0

                    di, target_idx = stanley.stanley_control(
                        state=state, cx=new_xs[:target_idx+15], cy=new_ys[:target_idx+15], cyaw=new_yaws[:target_idx+15], last_target_idx=target_idx)

                    cmd_msg.speed = 3.0
                    cmd_msg.steer = -di
                    cmd_msg.brake = 1

                    cmd_pub.publish(cmd_msg)

                except Exception as ex:
                    print(ex)

        r.sleep()
