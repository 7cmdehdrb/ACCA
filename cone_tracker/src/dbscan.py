#!/usr/bin/env python

import enum
from operator import pos
from sensor_msgs.msg import LaserScan
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped
import tf
from nav_msgs.msg import Path


class DBSCAN(object):
    def __init__(self, epsilon, minpts):
        self.epsilon = epsilon
        self.minpts = minpts
        self.C = 0
        self.scan_data = []
        self.x = []
        self.data = []
        self.centerpts = []

    def laserCallback(self, msg):
        self.scan_data = msg.ranges

    def cvtRange(self):
        DEG = 30.0

        if len(self.scan_data) == 0:
            pass

        else:
            self.x = []
            for j in range(len(self.scan_data)):

                center = len(self.scan_data) / 2.0
                right = center - (DEG / 0.5)
                left = center + (DEG / 0.5)

                if right <= j and j <= left:
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
                if self.visited[i] == False:
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
        Neighbors = np.where(g == True)[0].tolist()

        return Neighbors

    def expandCluster(self, i):
        self.idx[i] = self.C
        k = 0

        while True:
            try:
                j = self.neighbors[k]
            except:
                pass
            if self.visited[j] != True:
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

            # print((np.sqrt(pow((group[0][:,0]-(np.mean(cluster[idx][0],axis=0)[0])),2)+pow((group[0][:,1]-(np.mean(cluster[idx][0],axis=0)[1])),2))))
            self.centerpts.append(np.mean(cluster[idx][0], axis=0).tolist())
        return self.centerpts

    def plot(self, cluster):
        self.sort()

        fig, ax = plt.subplots()
        for idx, group in enumerate(cluster):

            ax.plot(group[0][:, 0],
                    group[0][:, 1],
                    marker='o',
                    linestyle='',
                    label='Cluster {}'.format(idx))

            if not idx == 0:
                ax.plot(np.mean(cluster[idx][0], axis=0)[0],
                        np.mean(cluster[idx][0], axis=0)[1],
                        marker='x')

        ax.legend(fontsize=10, loc='lower left')
        plt.title('Scatter Plot of Clustering results', fontsize=15)
        plt.xlabel('X', fontsize=14)
        plt.ylabel('Y', fontsize=14)
        plt.show()


if __name__ == "__main__":
    rospy.init_node("dbscan")

    dbscan = DBSCAN(0.3, 2)

    rospy.Subscriber("/scan_filtered", LaserScan, dbscan.laserCallback)
    pub = rospy.Publisher("/obstacles", PoseArray, queue_size=1)

    position = []

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        # print("??")

        dbscan.cvtRange()

        dbscan.run()
        cluster, _ = dbscan.sort()

        if not len(cluster) == 0:
            position = dbscan.find_far_pt(cluster)

            obstacle = PoseArray()

            obstacle.header.frame_id = "laser"
            obstacle.header.stamp = rospy.Time.now()

            del obstacle.poses[:]

            for i in range(len(position)):
                if position[i][0] != 0 and position[i][1] != 0:
                    pose = Pose()

                    pose.position.x = position[i][1]
                    pose.position.y = position[i][0] * -1
                    pose.position.z = 0

                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0

                    obstacle.poses.append(pose)

            pub.publish(obstacle)

        else:
            print("NO OB")

        r.sleep()
