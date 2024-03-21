#!/usr/bin/env python

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
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from scipy.interpolate import interp1d
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
          PointField('rgba', 12, PointField.UINT32, 1),
          ]


class gromi_explore:
	def __init__(self):
		self.minScanDist = 5
		self.min_dist = 5
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
		self.currentpos =[]
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
		self.slam = Odometry()

		self.slam_sub = rospy.Subscriber('odometry/filtered', Odometry, self.slamCb)
		self.heading_pub = rospy.Publisher("/current_heading", Int16, queue_size=1)
#		self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.sendNavCb)
		self.feedback_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.feedbackCb)
#		self.octomap_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.VoxelCenter)
#		self.pub_scan_pose = rospy.Publisher("/scan_positions", PointCloud2, queue_size=2)
		

	def slamCb(self,data):
		self.slam = data
		x = self.slam.pose.pose.position.x
		y = self.slam.pose.pose.position.y
		w = self.slam.pose.pose.orientation.w
		orientation_list = [self.slam.pose.pose.orientation.x, self.slam.pose.pose.orientation.y, self.slam.pose.pose.orientation.z, self.slam.pose.pose.orientation.w]	
		euler = tf.transformations.euler_from_quaternion(orientation_list)
		heading_tmp = euler[2]
		if heading_tmp > math.pi:        
			self.heading = heading_tmp - 2*math.pi
		elif heading_tmp < -math.pi:
			self.heading = heading_tmp + 2*math.pi
		else:
			self.heading = heading_tmp
#		print(self.heading)

	
	def feedbackCb(self,data):
		self.feedback = data
		x = self.feedback.feedback.base_position.pose.position.x
		y = self.feedback.feedback.base_position.pose.position.y
		w = self.feedback.feedback.base_position.pose.orientation.w
		self.sample.append([x,y,w])
		self.currentpos = [x,y]
		if len(self.sample) > 20 : #20: 15:
			sa = np.array(self.sample)
			mx = np.average(sa[:,0])
			my = np.average(sa[:,1])
			mw = np.average(sa[:,2]) - w
			if calc_distance([x,y],[mx,my]) < 0.1 and abs(mw) < 0.01:
				rospy.loginfo('Stuck. Resetting Goal...')
#				self.isstuck = True
				self.sendGoal([x+0.5,y])

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
