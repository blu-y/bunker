#!/usr/bin/env python3

import cv2
import rospy
import struct
import numpy as np
import math, tf, time
import matplotlib.path as mpltPath

from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
#from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

cost_update = OccupancyGridUpdate()
result = MoveBaseActionResult()
newPose = PoseStamped()
q = Quaternion()

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          # PointField('rgba', 12, PointField.UINT32, 1),
          ]


class gromi_explore:
	def __init__(self):
		self.minScanDist = 5
		self.min_bndX = -10
		self.max_bndX = 10
		self.min_bndY = -10
		self.max_bndY = 10
		self.costmap = OccupancyGrid()
		self.flagm = 0
		self.flagg = 0
		self.init = 0
		self.newcost = []
		self.grid = OccupancyGrid()
		self.feedback = MoveBaseActionFeedback()
		self.prevpnt = -1
		self.nextpnt = np.asarray([0, 0])
		self.sample = []
		self.stuckpnt = []
		self.isstuck = False
		self.offsetcnt = 0
		self.firstgoal = 0
		self.checkpnt = 0
		self.pntlist = []
		self.send = True
		self.currentpos = [0, 0, 0]
		self.heading = 0
		self.area = 0
		self.num_unknown = 0
		self.scaledprevpnt = np.asarray([-10, -10])
		self.map_res = 0
		self.map_width = 0
		self.map_height = 0
		self.map_origX = 0
		self.map_origY = 0
		self.min_bnd_indX = 0
		self.max_bnd_indX = 0
		self.min_bnd_indY = 0
		self.max_bnd_indY = 0
		self.scanPts = []
		self.poseX = 0
		self.poseY = 0
		self.voxelPts = []
		self.scanPositions = []
		self.newGoal = np.asarray([0, 0])
		self.finished_scan = 0
		self.mean_dist = 0
		self.pre_mean_dist = 0
		self.pre_scan_pose = [0, 0]
		self.slam = Odometry()
		self.cnt = 0
		self.init = 1
		self.rate_of_unknown = 0
		self.pre_rate_of_unknown = 0

		self.scan_pose_pub = rospy.Publisher("/scan_positions", PointCloud2, queue_size=2)
		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb,queue_size=1)
#		self.slam_sub = rospy.Subscriber('odometry/filtered', Odometry, self.slamCb)
		self.slam_sub = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.slamCb)
		self.heading_pub = rospy.Publisher("/current_heading", Float32, queue_size=1)
#		self.pub = rospy.Publisher('/move_base/goal', PoseStamped, queue_size=1)
		self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
#		self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.sendNavCb)
		self.feedback_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.feedbackCb)
		self.octomap_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.VoxelCenter)
		self.less_sharp_sub = rospy.Subscriber('/laser_cloud_less_sharp', PointCloud2, self.LaserCloudLessSharp)
		

	def VoxelCenter(self,data):
		for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
			self.voxelPts.append([p[0], p[1], p[2]])


	def LaserCloudLessSharp(self,data):
		min_dist = 1000
		dist = 0
		diff = 0
		for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
			x_diff = p[0] - self.currentpos[0]
			y_diff = p[1] - self.currentpos[1]
			z_diff = p[2] - self.currentpos[2]
			dist_tmp = math.sqrt( x_diff**2 + y_diff**2 + z_diff**2 )
			# modify values
#			if dist_tmp < 6:	# 30
			dist = dist + dist_tmp
			if min_dist > dist_tmp:
				min_dist = dist_tmp
#		if self.init == 1:
#			self.pre_mean_dist = dist			
#			self.init = 0
#		else:
#			diff = dist - self.pre_mean_dist
#			self.pre_mean_dist = dist
#		self.mean_dist = (self.mean_dist + dist) / 2
		minScanDist = 1000
#		for i in self.scanPositions:
#			print(self.scanPositions[i][0], self.scanPositions[i][1], self.scanPositions[i][2])
#			minScanDist_tmp = math.sqrt( (self.scanPositions[0]-self.currentpos[0])**2 + (self.pre_scan_pose[1]-self.currentpos[1])**2 )

		self.scan_interval = math.sqrt( (self.pre_scan_pose[0]-self.currentpos[0])**2 + (self.pre_scan_pose[1]-self.currentpos[1])**2 )
		print('Scan_interval, min_dist, rate_of_unknown is ' + str(self.scan_interval) + ', ' + str(min_dist) + ', ' + str(self.rate_of_unknown))
#		if ( (self.scan_interval > 3) and (dist > self.mean_dist) ):
		if ( (self.scan_interval > 20) and (min_dist > 3) and (self.pre_rate_of_unknown - self.rate_of_unknown > 0.05) ):
			self.scanPositions.append([self.currentpos[0], self.currentpos[1], 0.0])
			self.publishScanPositions(self.scanPositions)	
			self.pre_scan_pose = [self.currentpos[0], self.currentpos[1]]
			self.pre_rate_of_unknown = self.rate_of_unknown
			print('Scan position is ' + str(self.currentpos[0]) + ', ' + str(self.currentpos[1]))
			print(self.scanPositions)


	def publishScanPositions(self, positions):
		header = Header()
		header.frame_id = "/map"
		pc2 = point_cloud2.create_cloud(header, fields, self.scanPositions)
		pc2.header.stamp = rospy.Time.now()
		self.scan_pose_pub.publish(pc2)


	def mapCb(self,data):
		self.grid = data
		cell_val = data.data
		self.num_unknown = cell_val.count(-1)
		self.map_res = self.grid.info.resolution
		self.map_width = self.grid.info.width
		self.map_height = self.grid.info.height 
		self.map_origX = self.grid.info.origin.position.x
		self.map_origY = self.grid.info.origin.position.y
		map_size = self.map_width*self.map_height
		self.cnt = self.cnt + 1
		self.rate_of_unknown = float(self.num_unknown) / float(map_size)
		if self.cnt == 1:
			self.pre_rate_of_unknown = self.rate_of_unknown
			self.scanPositions.append([0.0, 0.0, 0.0])
			self.publishScanPositions(self.scanPositions)
#		if self.cnt > 50:
#			print( self.rate_of_unknown )
#			self.cnt = 0
#		print(self.num_unknown, self.map_width, self.map_height)

	
	def slamCb(self,data):
		self.slam = data
		x = self.slam.pose.pose.position.x
		y = self.slam.pose.pose.position.y
		z = self.slam.pose.pose.position.z
		w = self.slam.pose.pose.orientation.w
		self.currentpos = [x,y,z]
		orientation_list = [self.slam.pose.pose.orientation.x, self.slam.pose.pose.orientation.y, self.slam.pose.pose.orientation.z, self.slam.pose.pose.orientation.w]	
		euler = tf.transformations.euler_from_quaternion(orientation_list)
		heading_tmp = euler[2]
		if heading_tmp > math.pi:        
			self.heading = heading_tmp - 2*math.pi
		elif heading_tmp < -math.pi:
			self.heading = heading_tmp + 2*math.pi
		else:
			self.heading = heading_tmp
		self.heading_pub.publish(self.heading)
#		print(self.heading)

	
	def feedbackCb(self,data):
		self.feedback = data
		x = self.feedback.feedback.base_position.pose.position.x
		y = self.feedback.feedback.base_position.pose.position.y
		z = self.feedback.feedback.base_position.pose.position.z
		w = self.feedback.feedback.base_position.pose.orientation.w
		self.sample.append([x,y,w])
		self.currentpos = [x,y,z]
		if len(self.sample) > 20 : #20: 15:
			sa = np.array(self.sample)
			mx = np.average(sa[:,0])
			my = np.average(sa[:,1])
			mw = np.average(sa[:,2]) - w
			if calc_distance([x,y],[mx,my]) < 0.1 and abs(mw) < 0.05:
				rospy.loginfo('Stuck. Resetting Goal...')
#				self.isstuck = True
				print('Current point is ' + str(self.heading))
				print('Current heading is ' + str(self.currentpos[0]) + ', ' + str(self.currentpos[1]))
				self.sendGoal([x+0.5*math.cos(self.heading),y+0.5*math.cos(self.heading)])
			self.sample = []


	def sendGoal(self,nextpnt):
		print('Next point is ' + str(nextpnt[0]) + ', ' + str(nextpnt[1]))
		q = quaternion_from_euler(0,0,0,'sxyz')
		newPose.header.stamp = rospy.Time.now()
		newPose.header.frame_id = "map"
		newPose.pose.position.x = nextpnt[0]
		newPose.pose.position.y = nextpnt[1]
		newPose.pose.orientation.x = q[0]
		newPose.pose.orientation.y = q[1]
		newPose.pose.orientation.z = q[2]
		newPose.pose.orientation.w = q[3]
		self.pub.publish(newPose)
	#	print('Area is ' + str(self.area))
	#	print('num of unknown cell is ' + str(self.num_unknown))


def calc_distance(pnt1,pnt2):
    x1,y1 = pnt1
    x2,y2 = pnt2
    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist


def find_nearest_pts(node,nodes):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    dist = np.sqrt(np.sum((nodes - node)**2, axis=1))
    return np.argmin(dist)


def find_closest_frontier(node,nodes,heading):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    diff = nodes - node
    dist_2 = np.sum(diff**2, axis=1)
    distance = np.sqrt(dist_2)
    ang = np.arctan2(diff[:,1],diff[:,0])
    ang_diff = abs(ang-heading)
    return np.argmin(ang_diff)
#    return np.argmin(dist_2*ang_diff)
#    return np.argmin(distance*ang_diff)
#    return np.argmin(dist_2)

if __name__ == "__main__":
    rospy.init_node('explore') #make node
    rospy.sleep(1)
    gc = gromi_explore()
    rospy.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
