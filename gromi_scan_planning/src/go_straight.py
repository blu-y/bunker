#!/usr/bin/env python3

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
		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb, queue_size=1)
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
#			print(theta)
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
#		print(dist, count, dist/count)

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
#		print(self.travel_dist)


	def mapCb(self,data):
		self.grid = data
		self.map_res = self.grid.info.resolution
		self.map_width = self.grid.info.width
		self.map_height = self.grid.info.height 
		self.map_origX = self.grid.info.origin.position.x
		self.map_origY = self.grid.info.origin.position.y
		map_data = data.data			
		self.num_unknown = map_data.count(-1)
#		rows = len(str(map_data))
#		cols = len(str(map_data[0]))
		arr = np.array(map_data)
		cell_val = arr.reshape(self.map_height,self.map_width)
		
		offset = 2
		frontier = []
		frontier_cost = []
		for i in range(offset, cell_val.shape[0]-offset, offset):	
			for j in range(offset, cell_val.shape[1]-offset, offset):
				if ( cell_val[i,j] == 0 ):
					if ( cell_val[i-offset,j] == -1 or cell_val[i,j-offset] == -1 or cell_val[i+offset,j] == -1 or cell_val[i,j+offset] == -1 ):
						x = j * self.map_res + self.map_origX			
						y = i * self.map_res + self.map_origY						
						x_diff = x - self.current_pose[0]
						y_diff = y - self.current_pose[1]
						dist = math.sqrt( x_diff**2 + y_diff**2 )
						ang = math.atan2(y_diff, x_diff)
						ang_diff_tmp = self.current_heading - ang
						if ang_diff_tmp > math.pi:        
						    ang_diff = ang_diff_tmp - 2*math.pi
						elif ang_diff_tmp < -math.pi:
						    ang_diff = ang_diff_tmp + 2*math.pi
						else:
						    ang_diff = ang_diff_tmp						
						# modify values						
						if dist > 5 and dist < 30 and ang_diff < 3*math.pi/4 and ang_diff > -3*math.pi/4:
							frontier.append([x, y])	
							frontier_cost.append(1.0*abs(ang_diff) + 0.00*dist)
		if len(frontier) > 0:
			self.publishMarker(frontier)
			sorted_index = np.argsort(frontier_cost)
			best_frontier = frontier[sorted_index[0]]

			x1 = self.current_pose[0]
			y1 = self.current_pose[1]
			thetaStart = self.current_heading
			x2 = best_frontier[0]
			y2 = best_frontier[1]
			RPM1 = 15
			RPM2 = 10
			RADIUS = 0.0
			CLEARANCE = 0.0
			print([x1, y1, x2, y2])

			end = Node(x2, y2, x2, y2, 0)
			start = Node(x1, y1, x2, y2, thetaStart)
			start.costToCome = 0
			robot = Graph(start, end, RPM1, RPM2, RADIUS, CLEARANCE)
			path = []

			if robot.performAStar(start, end):
			    path.reverse()
			    self.sendGoal(best_frontier)

		else:
			self.sendGoal([0.0,0.0])
			print("Complete Scan!!!!")
			

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
		print('Next point is ' + str(nextpnt[0]) + ', ' + str(nextpnt[1]))
		

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

