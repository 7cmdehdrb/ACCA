import rospy 
import ma_rrt
import csv
import math, time

from scipy.spatial import Delaunay
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry, Path

from scipy.spatial import Delaunay

class DelaunayEdges:

    self.carPosX = 0.0
    self.carPosY = 0.0
    self.carPosYaw = 0.0

    def odometryCallback(self, odometry):
        
        orientation_q = odometry.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)

        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y
        self.carPosYaw = yaw

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector
    
    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()
        # print("headingVector:", headingVector)

        headingVectorOrt = [-headingVector[1], headingVector[0]]
        # print("headingVectorOrt:", headingVectorOrt)

        behindDist = 0.5
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]

        # print "carPos:", [self.carPosX, self.carPosY]
        # print "carPosBehindPoint:", carPosBehindPoint

        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.x - carPosBehindPoint[0])) < 0:
                if ((cone.x - self.carPosX) ** 2 + (cone.y - self.carPosY) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return

        conePoints = np.zeros((len(frontCones), 2))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.x, cone.y])

        # print conePoints
        tri = Delaunay(conePoints)
        # print "len(tri.simplices):", len(tri.simplices)

        delaunayEdges = []
        for simp in tri.simplices:
            # print simp

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)

        return delaunayEdges

    def publishDelaunayEdgesVisual(self, edges):
        if not edges:
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1

        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.b = 1.0

        for edge in edges:
            # print edge

            p1 = Point(edge.x1, edge.y1, 0)
            p2 = Point(edge.x2, edge.y2, 0)

            marker.points.append(p1)
            marker.points.append(p2)

        self.delaunayLinesVisualPub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("lms_test")

    laser = Laser()

    cone_pub = rospy.Publisher("cone_position", PoseArray, queue_size=1)
    rospy.Subscriber("/scan_filtered", LaserScan, laser.laserCallback)
    delaunayLinesVisualPub = rospy.Publisher("/visual/delaunay_lines", Marker, queue_size=1)


    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        laser.pubResults(publisher=cone_pub)
        r.sleep()
