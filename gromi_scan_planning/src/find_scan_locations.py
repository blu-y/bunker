#!/usr/bin/env python

import math
import numpy
import rospy
import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class gromi_explore:
	def __init__(self):
		self.pts = []
		self.current_pose = []
		self.pre_pose = [0, 0, 0]
		self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
			       PointField('y', 4, PointField.FLOAT32, 1),
			       PointField('z', 8, PointField.FLOAT32, 1),
			       ]
		self.cnt = 0
		self.count = 0
		self.mean_dist = 0
		self.scanPositions = []
		self.pre_scan_pose = [0, 0, 0]
		self.rospy.Subscriber('/frontier', PointCloud2, MakeGroundFrontier)
		self.rospy.Subscriber('/laser_cloud_less_sharp', PointCloud2, LaserCloudLessSharp)
		self.rospy.Subscriber('/aft_mapped_to_init', Odometry, CurrentPose)
pub_ground_frontier = rospy.Publisher('/ground_frontier', PointCloud2, queue_size=1)
pub_best_frontier = rospy.Publisher('/best_frontier',PoseStamped, queue_size = 1)
pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
pub_current_pose = rospy.Publisher('/current_pose',PoseStamped, queue_size = 1)
pub_scan_pose = rospy.Publisher("/scan_positions", PointCloud2, queue_size=2)

def MakeGroundFrontier(data):
    global pts, current_pose, pre_pose, fields

    cnt = 0
    pts = []
    ang = []
    dist = []
    for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
	x_diff = p[0]-current_pose.x
	y_diff = p[1]-current_pose.y
	z_diff = p[2]-current_pose.z
	current_height = current_pose.y
	dist_tmp = math.sqrt( x_diff**2 + z_diff**2 )
	ang_tmp = math.atan2(x_diff, z_diff) * (180/math.pi)
	heading = math.atan2(current_pose.x-pre_pose[0], current_pose.z-pre_pose[2]) * (180/math.pi)
	ang_diff = abs(heading - ang_tmp)
        if ( (ang_diff < 60) and (dist_tmp < 30)):#and (y_diff > 0) ):	
    	    #pts.append([p[0], current_height, p[2]])
	    pts.append([p[0], p[1], p[2]])
	    dist.append(dist_tmp)
	    ang.append(ang_diff)
    	    cnt = cnt + 1

    cost_fn = [dist[i] + ang[i] for i in range(len(dist))]
    #cost_fn = [ang[i] for i in range(len(dist))]
    sorted_index = numpy.argsort(cost_fn)
    best_frontier = pts[sorted_index[0]]
    angle = math.atan2(best_frontier[0]-current_pose.x, best_frontier[2]-current_pose.z) * (180/math.pi)
#    print pts
#    print sorted(cost_fn)
#    print sorted(dist)
#    print current_pose
	    
    header = Header()
    header.frame_id = "/camera_init"
    pc2_ground_frontier = point_cloud2.create_cloud(header, fields, pts)
    pc2_ground_frontier.header.stamp = rospy.Time.now()
    pub_ground_frontier.publish(pc2_ground_frontier)
    
    BestGoal = PoseStamped()
    BestGoal.header.frame_id = "/camera_init"
    BestGoal.header.stamp = rospy.Time.now()
    BestGoal.pose.orientation.w = 1
    BestGoal.pose.position.x = best_frontier[0]
    BestGoal.pose.position.y = best_frontier[1]	
    BestGoal.pose.position.z = best_frontier[2]

    pub_goal.publish(BestGoal)
    rospy.sleep(10)
    print(str(len(best_frontier)) + ' frontier points PoseStamped published!!')


def CurrentPose(data):
    global current_pose, pre_pose, cnt
    
    current_pose = data.pose.pose.position
    CurrentPose = PoseStamped()
    CurrentPose.header.frame_id = "/camera_init"
    CurrentPose.header.stamp = rospy.Time.now()
    CurrentPose.pose.orientation.w = 1
    CurrentPose.pose.position.x = current_pose.x
    CurrentPose.pose.position.y = current_pose.y	
    CurrentPose.pose.position.z = current_pose.z
    pub_current_pose.publish(CurrentPose)
    cnt = cnt + 1
    if cnt > 10:
	pre_pose[0] = current_pose.x
    	pre_pose[1] = current_pose.y
    	pre_pose[2] = current_pose.z
	cnt = 0


def LaserCloudLessSharp(data):
    global current_pose, count, mean_dist, scanPositions, fields, pre_scan_pose
    dist = 0
    for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
	x_diff = p[0]-current_pose.x
	y_diff = p[1]-current_pose.y
	z_diff = p[2]-current_pose.z
	current_height = current_pose.y
	dist_tmp = math.sqrt( x_diff**2 + y_diff**2 + z_diff**2 )
	if dist_tmp < 50:
	    dist = dist + dist_tmp
    mean_dist = (mean_dist + dist) / 2
#    print dist, mean_dist 
    scan_interval = math.sqrt( (pre_scan_pose[0]-current_pose.x)**2 + (pre_scan_pose[1]-current_pose.y)**2 + (pre_scan_pose[2]-current_pose.z)**2 )
    if ( (scan_interval > 20) and (dist > mean_dist) ):
        count = count + 1
        scanPositions.append([current_pose.x, current_pose.y, current_pose.z])
        header = Header()
        header.frame_id = "/camera_init"
        pc2 = point_cloud2.create_cloud(header, fields, scanPositions)
        pc2.header.stamp = rospy.Time.now()
        pub_scan_pose.publish(pc2)
	pre_scan_pose = [current_pose.x, current_pose.y, current_pose.z]
	print scan_interval


print("start find the best frontier point")
rospy.init_node("start_find_best_frontier_point")



fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          ]


r = rospy.Rate(50) 
while not rospy.is_shutdown():
	try:
		r.sleep()
		rospy.spin()
	except KeyboardInterrupt:
		print("end program")
		break

