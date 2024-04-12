#!/usr/bin/env python

import tf
import math
import numpy
import rospy
import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

def MakeGroundFrontier(data):
    global pts, current_pose, pre_pose, fields, pre_heading, heading

    cnt = 0
    pts = []
    ang = []
    dist = []
    print "MakeGroundFrontier"
    for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
	x_diff = p[0]-current_pose.z
	y_diff = p[1]-current_pose.x
	z_diff = p[2]-current_pose.y
	current_height = current_pose.y
	dist_tmp = math.sqrt( x_diff**2 + y_diff**2 )
	ang_tmp = math.atan2(x_diff, y_diff) * (180/math.pi)
	ang_diff_tmp = abs(heading*(180/math.pi) - ang_tmp)
        if ang_diff_tmp > math.pi:        
	    ang_diff = ang_diff_tmp - 2*math.pi
        elif ang_diff_tmp < -math.pi:
	    ang_diff = ang_diff_tmp + 2*math.pi
#        if ( (ang_diff < 180) and (dist_tmp < 30)):#and (y_diff > 0) ):	
#    	if (ang_diff < 180):# and (dist_tmp < 30)):#and (y_diff > 0) ):	
  	    #pts.append([p[0], current_height, p[2]])
        pts.append([p[0], p[1], p[2]])
	dist.append(dist_tmp)
	ang.append(ang_diff)
    	cnt = cnt + 1

    #cost_fn = [dist[i] + ang[i] for i in range(len(dist))]
    cost_fn = [ang[i] for i in range(len(dist))]
    sorted_index = numpy.argsort(cost_fn)
    if len(pts) > 0:
        best_frontier = pts[sorted_index[0]]
#        angle = math.atan2(best_frontier[0]-current_pose.x, best_frontier[2]-current_pose.y) * (180/math.pi)
	angle = math.atan2(best_frontier[0], best_frontier[2])
        print pts
#    print sorted(cost_fn)
#    print sorted(dist)
#    print current_pose
	    
#        header = Header()
#        header.frame_id = "/map"
#        pc2_ground_frontier = point_cloud2.create_cloud(header, fields, pts)
#        pc2_ground_frontier.header.stamp = rospy.Time.now()
#        pub_ground_frontier.publish(pc2_ground_frontier)
    
        BestGoal = PoseStamped()
        BestGoal.header.frame_id = "map"
        BestGoal.header.stamp = rospy.Time.now()
        BestGoal.pose.orientation.w = 1
        BestGoal.pose.position.x = 3*math.cos(angle) #best_frontier[2]
        BestGoal.pose.position.y = 3*math.sin(angle) #best_frontier[0]	
        BestGoal.pose.position.z = current_height #best_frontier[1]
        pub_goal.publish(BestGoal)
        rospy.sleep(10)
        print(str(len(best_frontier)) + ' frontier points PoseStamped published!!')
	print BestGoal


def CurrentPose(data):
    global current_pose, pre_pose, cnt, heading
    
    print "CurrentPose"
    current_pose = data.pose.pose.position
    current_heading = data.pose.pose.orientation
    qy = current_heading.x
    qz = current_heading.y
    qx = current_heading.z
    qw = current_heading.w

    orientation_list = [qx, qy, qz, qw]	
    euler = tf.transformations.euler_from_quaternion(orientation_list)
    heading_tmp = euler[2]
    if heading_tmp > math.pi:        
	heading = heading_tmp - 2*math.pi
    elif heading_tmp < -math.pi:
	heading = heading_tmp + 2*math.pi

    CurrentPose = PoseStamped()
    CurrentPose.header.frame_id = "map"
    CurrentPose.header.stamp = rospy.Time.now()
    CurrentPose.pose.orientation.w = 1
    CurrentPose.pose.position.x = current_pose.z
    CurrentPose.pose.position.y = current_pose.x	
    CurrentPose.pose.position.z = current_pose.y
    pub_current_pose.publish(CurrentPose)
    cnt = cnt + 1
    if cnt > 10:
	pre_pose[0] = current_pose.x
    	pre_pose[1] = current_pose.y
    	pre_pose[2] = current_pose.z
	cnt = 0


def LaserCloudLessSharp(data):
    global current_pose, count, mean_dist, scanPositions, fields, pre_scan_pose

    print "LaserCloudLessSharp"
    dist = 0
    for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
	x_diff = p[0]-current_pose.z
	y_diff = p[1]-current_pose.x
	z_diff = p[2]-current_pose.y
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
        header.frame_id = "/map"
        pc2 = point_cloud2.create_cloud(header, fields, scanPositions)
        pc2.header.stamp = rospy.Time.now()
        pub_scan_pose.publish(pc2)
	pre_scan_pose = [current_pose.x, current_pose.y, current_pose.z]
	print scan_interval


print("start find the best frontier point")
rospy.init_node("start_find_best_frontier_point")
rospy.Subscriber('/frontier', PointCloud2, MakeGroundFrontier)
rospy.Subscriber('/laser_cloud_less_sharp', PointCloud2, LaserCloudLessSharp)
rospy.Subscriber('/aft_mapped_to_init', Odometry, CurrentPose)
pub_ground_frontier = rospy.Publisher('/ground_frontier', PointCloud2, queue_size=1)
pub_best_frontier = rospy.Publisher('/best_frontier',PoseStamped, queue_size = 1)
pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
pub_current_pose = rospy.Publisher('/current_pose',PoseStamped, queue_size = 1)
pub_scan_pose = rospy.Publisher("/scan_positions", PointCloud2, queue_size=2)
pre_pose = [0, 0, 0]
mean_dist = 0
cnt = 0
count = 0
scanPositions = []
pre_scan_pose = [0, 0, 0]
pre_heading = 0
heading = 0

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

