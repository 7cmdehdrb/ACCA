#!/usr/bin/env python

import rospy
import math
import numpy as np
# import tf
# import enum
from geometry_msgs.msg import PoseArray, Pose  # PointStamped, PoseStamped
# from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
# from operator import pos


class DBSCAN(object):
    def __init__(self, epsilon, minpts):
        self.epsilon = epsilon
        self.minpts = minpts
        self.C = 0
        self.scan_data = []
        self.x = []
        self.data = []
        self.centerpts = []
        self.degree = 0

    def laserCallback(self, msg):
        self.scan_data = msg.ranges

    def cvtRange(self):
        if len(self.scan_data) == 0:
            pass

        else:
            self.x = []
            for j in range(0, 120+self.degree):

                angle = j / 2.0

                x = self.scan_data[j] * math.cos(math.radians(angle))
                y = self.scan_data[j] * math.sin(math.radians(angle))
                self.x.append([x, y])

            self.data = np.array(self.x)

    def run(self):
        if not len(self.data) == 0:
            self.n = len(self.data)
            # Euclidean distance
            p, q = np.meshgrid(np.arange(self.n), np.arange(self.n))
            self.dist = np.sqrt(np.sum(((self.data[p] - self.data[q])**2), 2))
            # label as visited points and noise
            self.visited = np.full((self.n), False)
            self.noise = np.full((self.n), False)
            self.idx = np.full((self.n), 0)
            self.input = self.data
            # Clustering
            for i, vector in enumerate(self.data):
                if self.visited[i] is False:
                    self.visited[i] = True
                    self.neighbors = self.regionQuery(i)
                    if len(self.neighbors) > self.minpts:
                        self.C += 1
                        self.expandCluster(i)
                    else:
                        self.noise[i] = True

            return self.idx, self.noise

    def regionQuery(self, i):
        g = self.dist[i, :] < self.epsilon
        Neighbors = np.where(g is True)[0].tolist()

        return Neighbors

    def expandCluster(self, i):
        self.idx[i] = self.C
        k = 0

        while True:
            try:
                j = self.neighbors[k]
            except Exception:
                pass
            if self.visited[j] is not True:
                self.visited[j] = True

                self.neighbors2 = self.regionQuery(j)

                if len(self.neighbors2) > self.minpts:
                    self.neighbors = self.neighbors+self.neighbors2

            if self.idx[j] == 0:
                self.idx[j] = self.C

            k += 1
            if len(self.neighbors) < k:
                return

    def sort(self):
        self.cluster = []
        self.noise = []
        if not len(self.data) == 0:
            cnum = np.max(self.idx)

            for i in range(cnum):

                k = np.where(self.idx == (i+1))[0].tolist()

                if not len(k) == 0:
                    self.cluster.append([self.input[k, :]])

            self.noise = self.input[np.where(self.idx == 0)[0].tolist(), :]

        return self.cluster, self.noise

    def find_far_pt(self, cluster):
        self.centerpts = []
        for idx, group in enumerate(cluster):
            pt = np.mean(cluster[idx][0], axis=0).tolist()
            if math.sqrt(math.pow(pt[1], 2) + math.pow(pt[0], 2)) < 12.0:
    
                # print((np.sqrt(pow((group[0][:,0]-(np.mean(cluster[idx][0],axis=0)[0])),2)+pow((group[0][:,1]-(np.mean(cluster[idx][0],axis=0)[1])),2))))
                self.centerpts.append(
                    np.mean(cluster[idx][0], axis=0).tolist())

        return self.centerpts


if __name__ == "__main__":
    rospy.init_node("dbscan")

    dbscan = DBSCAN(0.3, 2)

    rospy.Subscriber("/scan_filtered", LaserScan, dbscan.laserCallback)
    pub = rospy.Publisher("/cone_position", PoseArray, queue_size=1)

    position = []

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        # print("??")

        dbscan.cvtRange()

        dbscan.run()
        cluster, _ = dbscan.sort()

        if not len(cluster) == 0:

            position = dbscan.find_far_pt(cluster)
            if not len(position) < 3:
                print(position)
                obstacle = PoseArray()

                obstacle.header.frame_id = "laser"
                obstacle.header.stamp = rospy.Time.now()

                del obstacle.poses[:]

                for i in range(len(position)):

                    if position[i][0] != 0 and position[i][1] != 0:
                        pose = Pose()

                        pose.position.x = position[i][1]
                        pose.position.y = position[i][0]
                        pose.position.z = 0

                        pose.orientation.x = 0.0
                        pose.orientation.y = 0.0
                        pose.orientation.z = 0.0
                        pose.orientation.w = 1.0

                        obstacle.poses.append(pose)

                pub.publish(obstacle)
                dbscan.degree = 0
            else:
                dbscan.degree += 40
        else:
            print("NO OB")

        r.sleep()
