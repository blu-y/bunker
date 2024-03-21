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

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),]

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
		self.pre_frontier = [0, 0]
		self.scanPositions = []
		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb, queue_size=1)
#		self.slam_sub = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.slamCb)
		self.slam_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.hectorslamCb)
		self.less_sharp_sub = rospy.Subscriber('/laser_cloud_less_sharp', PointCloud2, self.LaserCloudLessSharp)
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.marker_pub = rospy.Publisher('/visualization_marker',Marker, queue_size = 10)
		self.scan_pose_pub = rospy.Publisher("/scan_positions", PointCloud2, queue_size=2)


	def hectorslamCb(self,data):
		self.slam = data.pose
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
		heading_tmp = euler[0]
		if heading_tmp > math.pi:        
			self.current_heading = heading_tmp - 2*math.pi
		elif heading_tmp < -math.pi:
			self.current_heading = heading_tmp + 2*math.pi
		else:
			self.current_heading = heading_tmp	
		self.travel_dist = math.sqrt( self.current_pose[0]**2 + self.current_pose[1]**2 )
#		print(self.travel_dist)
		print('Current pose is ' + str(self.current_pose[0]) + ', ' + str(self.current_pose[1]))
		print('Current heading is ' + str(self.current_heading) )
		
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
		print('Current pose is ' + str(self.current_pose[0]) + ', ' + str(self.current_pose[1]))
		print('Current heading is ' + str(self.current_heading) )
		
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
		print("\n")

	def publishMarker(self,pnt):
		newMarker = Marker()
		newMarker.header.frame_id = "map"
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


	def LaserCloudLessSharp(self,data):
		dist = 0
		for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
			x_diff = p[0] - self.current_pose[0]
			y_diff = p[1] - self.current_pose[1]
			z_diff = p[2] - self.current_pose[2]
			dist_tmp = math.sqrt( x_diff**2 + y_diff**2 + z_diff**2 )
			# modify values
			if dist_tmp < 30:
			    dist = dist + dist_tmp
		self.mean_dist = (self.mean_dist + dist) / 2
		self.scan_interval = math.sqrt( (self.pre_scan_pose[0]-self.current_pose[0])**2 + (self.pre_scan_pose[1]-self.current_pose[1])**2 )
		if ( (self.scan_interval > 2) and (dist > self.mean_dist) ):
			self.scanPositions.append([self.current_pose[0], self.current_pose[1], 0.0])
			self.publishScanPositions(self.scanPositions)	
			self.pre_scan_pose = [self.current_pose[0], self.current_pose[1]]


	def publishScanPositions(self, positions):
		header = Header()
		header.frame_id = "map"
		pc2 = point_cloud2.create_cloud(header, fields, self.scanPositions)
		pc2.header.stamp = rospy.Time.now()
		self.scan_pose_pub.publish(pc2)
			

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
							frontier_cost.append(0.0*abs(ang_diff) + 1.0*dist)
		if len(frontier) > 0:
			self.publishMarker(frontier)
			sorted_index = np.argsort(frontier_cost)
			best_frontier = frontier[sorted_index[0]]
#			best_frontier_tmp = frontier[sorted_index[0]]
#			diff = math.sqrt( (best_frontier_tmp[0]-self.current_pose[0])**2 + (best_frontier_tmp[1]-self.current_pose[1])**2)
#			ang = math.atan2((best_frontier_tmp[1]-self.current_pose[1]), (best_frontier_tmp[0]-self.current_pose[0]))
#			ang_diff_tmp = self.current_heading - ang
#			if ang_diff_tmp > math.pi:        
#			    ang_diff = ang_diff_tmp - 2*math.pi
#			elif ang_diff_tmp < -math.pi:
#			    ang_diff = ang_diff_tmp + 2*math.pi
#			else:
#			    ang_diff = ang_diff_tmp						
#			best_frontier = [3*math.cos(ang_diff), 3*math.sin(ang_diff)]
#			if diff < 3:
			self.sendGoal(best_frontier)
			self.pre_frontier = best_frontier
		else:
			self.sendGoal([0.0,0.0])
			print("Complete Scan!!!!")



if __name__ == "__main__":
    rospy.init_node('gromi_frontier_node') # make node
    rospy.sleep(1)
    gf = gromi_frontier()
    rospy.sleep(1)
    gf.sendGoal([0.0, 0.0])
    gf.publishScanPositions([0.0, 0.0, 0.0])
    rospy.sleep(0.5)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

