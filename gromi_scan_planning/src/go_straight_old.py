#!/usr/bin/env python

import tf
import math
import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int8, Int16
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),]

class Node:
    def __init__(self, i, j, endI, endJ, theta):
      
        self.i = i
        self.j = j
        self.theta = theta
        self.costToCome = 0.0
        self.costToGo = 2.5 * (math.sqrt((i - endI) ** 2 + (j - endJ) ** 2))
        self.cost = None
        self.neighbours = {}
        self.valid_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class Graph:
    def __init__(self, start, end, RPM1, RPM2, RADIUS, CLEARANCE):
        self.visited = {}
        self.endI = end.i
        self.endJ = end.j
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.RADIUS = RADIUS
        self.CLEARANCE = CLEARANCE

    def getNeighbours(self, currentNode):
       
        i, j, theta = currentNode.i, currentNode.j, currentNode.theta
        neighbours = {}
        valid_actions = {}
        actions = [[0, self.RPM1], [self.RPM1, 0], [self.RPM1, self.RPM1], [0, self.RPM2], [self.RPM2, 0],
                   [self.RPM2, self.RPM2], [self.RPM1, self.RPM2], [self.RPM2, self.RPM1]]
        for UL, UR in actions:
            x, y, newTheta, distance, lin_vel, ang_vel = self.getNewCoordiantes(i, j, theta, UL, UR)
            if (not self.isOutsideArena(x, y)) and (not self.isAnObstacle(x, y)):
                newNode = Node(x, y, self.endI, self.endJ, newTheta)
                neighbours[newNode] = distance
                valid_actions[newNode] = [lin_vel, ang_vel]
        return neighbours, valid_actions

    def getNewCoordiantes(self, i, j, theta, UL, UR):
        t = 0
        r = 0.105
        L = 0.16
        dt = 0.1

        UL = 3.14 * (UL / 30)
        UR = 3.14 * (UR / 30)
        ang_vel = (r / L) * (UR - UL)
        lin_vel = 0.5 * r * (UL + UR)
        newI = i
        newJ = j
        newTheta = 3.14 * theta / 180
        D = 0

        while t < 1:
            t = t + dt
            Delta_Xn = 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            Delta_Yn = 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            newI += Delta_Xn
            newJ += Delta_Yn
            newTheta += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        newTheta = 180 * newTheta / 3.14

        if newTheta > 0:
            newTheta = newTheta % 360
        elif newTheta < 0:
            newTheta = (newTheta + 360) % 360

        newI = self.getRoundedNumber(newI)
        newJ = self.getRoundedNumber(newJ)
        # newI = int(newI)
        # newJ = int(newJ)

        return newI, newJ, newTheta, D, lin_vel, ang_vel


    def getRoundedNumber(self, i):
        i = 50 * i
        i = int(i)
        i = float(i) / 50.0
        return i

    def performAStar(self, start, end):
       

        # Checking is start and end are in obstancle.
        if self.isAnObstacle(start.i, start.j) and self.isAnObstacle(end.i, end.j):
            rospy.loginfo("Starting and ending point are inside the obstacle! Check clearances!")
            return

        if self.isAnObstacle(start.i, start.j):
            rospy.loginfo("Starting point is inside the obstacle! Check clearances!")
            return

        if self.isAnObstacle(end.i, end.j):
            rospy.loginfo("Ending point is inside the obstacle! Check clearances!")
            return

        if self.isOutsideArena(start.i, start.j):
            rospy.loginfo("Starting point is outside the arena! Check clearances!")
            return

        if self.isOutsideArena(end.i, end.j):
            rospy.loginfo("Ending point is outside the arena! Check clearances!")
            return

        rospy.loginfo("Finding path...")
        priorityQueue = []
        visited_list = {}
        heapq.heappush(priorityQueue, (start.cost, start))
        while len(priorityQueue):
            currentNode = heapq.heappop(priorityQueue)
            currentNode = currentNode[1]
            if self.isInTargetArea(currentNode.i, currentNode.j):
                self.backTrack(currentNode)
                print("Found a path!")
                print("Distance Required to reach from start to end is:", currentNode.costToCome)
                return True

            if tuple([currentNode.i, currentNode.j]) in visited_list:
                continue
            visited_list[tuple([currentNode.i, currentNode.j])] = True

            currentDistance = currentNode.costToCome
            neighbours, valid_actions = self.getNeighbours(currentNode)
            currentNode.neighbours = neighbours
            currentNode.valid_actions = valid_actions
            for neighbourNode, newDistance in neighbours.items():
                neighbourNode.costToCome = currentDistance + newDistance
                neighbourNode.cost = neighbourNode.costToCome + neighbourNode.costToGo
                neighbourNode.parent = currentNode
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))
        print("Cannot find a path :(")
        return False

    def isInTargetArea(self, i, j):
       
        if (i - self.endI) ** 2 + (j - self.endJ) ** 2 - 0.01 <= 0:
            return True
        else:
            return False

    def isAnObstacle(self, x, y):
       
        # Boundary condition
        if (x < 0) or (x > 10) or (y < 0) or (y > 10): 
            return True
        
        # Obstacle 1 (Circle Up)
        elif (x-2)**2 + (y-8)**2 - (1+self.CLEARANCE)**2 <= 0:   
            return True
        
        # Obstacle 2 (Square) 
        elif x >= 0.25-self.CLEARANCE and x <= 1.75+self.CLEARANCE and y >= 4.25-self.CLEARANCE and y <= 5.75+self.CLEARANCE: 
            return True
        
        # Obstacle 3 (Rectangle Up)
        elif x >= 3.75-self.CLEARANCE and x <= 6.25+self.CLEARANCE and y >= 4.25-self.CLEARANCE and y <= 5.75+self.CLEARANCE:      
            return True
        
          # Obstacle 4 (Circle Down)
        elif (x-2)**2 + (y-2)**2 - (1+self.CLEARANCE)**2 <= 0:                
            return True
        
        # Obstacle 3 (Rectangle Down)
        elif x >= 7.25-self.CLEARANCE and x <= 8.75+self.CLEARANCE and y >= 2-self.CLEARANCE and y <= 4+self.CLEARANCE:      
            return True
        
        # Node in Freespace
        else:
            return False 
    def backTrack(self, child):
       
        while child != None:
            path.append(child)
            child = child.parent
        return True


    def isOutsideArena(self, x, y):
        

        return True if x < self.CLEARANCE or y < self.CLEARANCE or x > 10 - self.CLEARANCE or y > 10 - self.CLEARANCE else False

    def cost(self, costTogo , costTooCome):
        summ = int(costTogo) + int(costTooCome)
        return summ


class gromi_frontier:
	def __init__(self):
		self.grid = OccupancyGrid()
		self.num_unknown = 0
		self.map_res = 0
		self.map_width = 0
		self.map_height = 0
		self.map_origX = 0
		self.map_origY = 0
		self.mean_dist = 0
		self.current_heading = 0
		self.travel_dist = 0
		self.scan_interval = 0
		self.current_pose = [0, 0, 0]
		self.voxelPts = []
		self.pre_scan_pose = [0, 0]
		self.scanPositions = []
		self.velopt_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.VeldynePoints)
		self.slam_sub = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.slamCb)
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.marker_pub = rospy.Publisher('/visualization_marker',Marker, queue_size = 10)

	def VeldynePoints(self,data):
#		dist = np.array([0, 0, 0])
#		count = np.array([0, 0, 0])
		dist = 0
		count = 0
		for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
			r = math.sqrt( p[0]**2 + p[1]**2 + p[2]**2 )
			theta = math.atan2(p[1],p[0]) * 180 / math.pi
			phi = math.atan2(math.sqrt( p[0]**2 + p[1]**2),p[2])
			dist_tmp = r*math.sin(phi)
#			print theta
#			if theta > -5 and theta < 5:
#				count[0] = count[0] + 1
#				dist[0] = dist[0] + dist_tmp
#			if theta > -95 and theta < -85:
#				count[1] = count[1] + 1
#				dist[1] = dist[1] + dist_tmp
#			if theta > 85 and theta < 95:
#				count[2] = count[2] + 1
#				dist[2] = dist[2] + dist_tmp
			if theta > -5 and theta < 5:
				count = count + 1
				dist = dist + dist_tmp
		print dist, count, dist/count

	def slamCb(self,data):
		self.slam = data.pose.pose
		y = self.slam.position.x
		z = self.slam.position.y
		x = self.slam.position.z
		self.current_pose = [x, y, z]
		qy = self.slam.orientation.x
		qz = self.slam.orientation.y
		qx = self.slam.orientation.z
		qw = self.slam.orientation.w
		orientation_list = [qx, qy, qz, qw]	
		euler = tf.transformations.euler_from_quaternion(orientation_list)
		heading_tmp = euler[2]
		if heading_tmp > math.pi:        
			self.current_heading = heading_tmp - 2*math.pi
		elif heading_tmp < -math.pi:
			self.current_heading = heading_tmp + 2*math.pi
	    	self.travel_dist = math.sqrt( self.current_pose[0]**2 + self.current_pose[1]**2 )
#		print self.travel_dist


	def sendGoal(self,nextpnt):
		q = quaternion_from_euler(0,0,0,'sxyz')
		newPose = PoseStamped()
		newPose.header.stamp = rospy.Time.now()
		newPose.header.frame_id = "map"
		newPose.pose.position.x = nextpnt[0]
		newPose.pose.position.y = nextpnt[1]
		newPose.pose.orientation.x = q[0]
		newPose.pose.orientation.y = q[1]
		newPose.pose.orientation.z = q[2]
		newPose.pose.orientation.w = q[3]
		self.goal_pub.publish(newPose)
# 		print('Next point is ' + str(nextpnt[0]) + ', ' + str(nextpnt[1]))
		

 	def publishMarker(self,pnt):
		newMarker = Marker()
		newMarker.header.frame_id = "/map"
		newMarker.header.stamp = rospy.Time.now()
		newMarker.ns = "points"
		newMarker.id = 0
		newMarker.type = newMarker.POINTS #SPHERE #LINE_STRIP
		newMarker.action = newMarker.ADD
		newMarker.pose.orientation.w = 1
		newMarker.scale.x = 0.2 #0.05
		newMarker.scale.y = 0.2
		newMarker.scale.z = 0.2
		newMarker.color.r = 0.0
		newMarker.color.g = 0.0
		newMarker.color.b = 1.0
		newMarker.color.a = 1.0
		for j in range(0,len(pnt)):
			newMarker.points.append(Point(pnt[j][0], pnt[j][1], 0))
		self.marker_pub.publish(newMarker)
#		print(str(len(pnt)) + ' frontier points marker published!!')
			


if __name__ == "__main__":
    rospy.init_node('gromi_frontier_node') # make node
    rospy.sleep(1)
    gf = gromi_frontier()
    rospy.sleep(1)
    gf.sendGoal([0.0, 0.0])
#    gf.publishScanPositions([0.0, 0.0, 0.0])
    rospy.sleep(0.5)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

