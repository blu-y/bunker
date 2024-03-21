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


class jackal_explore:
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

		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb,queue_size=1)
		self.global_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costCb,queue_size=1)
		self.globalup_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costupCb,queue_size=1)
		self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.sendNavCb)
		self.slam_sub = rospy.Subscriber('aft_mapped_to_init', Odometry, self.slamCb)
#		self.slam_sub = rospy.Subscriber('odometry/filtered', Odometry, self.slamCb)
		self.feedback_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.feedbackCb)
		#self.laserPt_sub = rospy.Subscriber('/scan_cloud', PointCloud2, self.ScanCloud)
		self.octomap_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.VoxelCenter)
		self.less_sharp_sub = rospy.Subscriber('/laser_cloud_less_sharp', PointCloud2, self.LaserCloudLessSharp)
		self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.line_pub = rospy.Publisher('/visualization_marker',Marker, queue_size = 10)
		self.pub_scan_pose = rospy.Publisher("/scan_positions", PointCloud2, queue_size=2)
		self.pub_start_scan = rospy.Publisher("/start_scan", Int16, queue_size=2)
		self.finish_scan_sub = rospy.Subscriber('/finish_scan', Int16, self.finish_scan)
		

	def finish_scan(self,data):
		if data.data == 1:
			self.finished_scan = data.data
			print "finish static scan"
	
	def costupCb(self,data):
		self.flagg = 1
		cost_update = data

		if self.init == 1 and self.flagm == 0:
			gpnts = cost_update.data
			gwidth = cost_update.width
			gheight = cost_update.height
			gx = cost_update.x
			gy = cost_update.y
			cnt = 0
			for x in range(gheight):
				for y in range(gwidth):
					g = gpnts[cnt]
					self.newcost[x+gy][y+gx] = g
					cnt += 1

	def costCb(self,data):
		rospy.loginfo('cost map received')

		self.newcost = []
		self.flagm = 1
		self.costmap = data

		pnts = self.costmap.data
		cnt = 0

		for x in range(self.costmap.info.height):
			newcost_x = []
			for y in range(self.costmap.info.width):
				newcost_x.append(pnts[cnt])
				cnt+=1
			self.newcost.append(newcost_x)

	def mapCb(self,data):
		self.grid = data
		cell_val = data.data
		self.num_unknown = cell_val.count(-1)
		rows = len(str(cell_val))
		cols = len(str(cell_val[0]))
		
		self.map_res = self.grid.info.resolution
		self.map_width = self.grid.info.width
		self.map_height = self.grid.info.height 
		self.map_origX = self.grid.info.origin.position.x
		self.map_origY = self.grid.info.origin.position.y

		self.min_bnd_indX = round((self.min_bndX - self.map_origX) / self.map_res) 
		self.max_bnd_indX = round((self.max_bndX - self.map_origX) / self.map_res)
		self.min_bnd_indY = round((self.min_bndY - self.map_origY) / self.map_res)
		self.max_bnd_indY = round((self.max_bndY - self.map_origY) / self.map_res) 

		if self.flagm == 1:
			self.init = 1
			rospy.loginfo('map received')
			pnts = self.grid.data
			gpnts = self.costmap.data
			cnt = 0
			pic_mat = []

			for x in range(self.map_height):
				pic_x = []
				for y in range(self.map_width):
					p = pnts[cnt]
					g = gpnts[cnt]

					if (g>0):
						p = 0
					else:
						if (p ==-1):
							p = 50
						elif (p == 0):
							p = 100
						else:
							p = 0
					pic_x.append(p)
					cnt += 1
				pic_mat.append(pic_x)
			self.flagm = 0
	    
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

        def LaserCloudLessSharp(data):
		dist = 0
	        for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
		    x_diff = p[0] - self.slam.pose.pose.position.x
		    y_diff = p[1] - self.slam.pose.pose.position.y
		    z_diff = p[2] - self.slam.pose.pose.position.z
		current_height = self.slam.pose.pose.position.y
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

	def feedbackCb(self,data):
		offsetx = [1,-1,1,-1]
		offsety = [1,-1,-1,1]

		self.feedback = data
		x = self.feedback.feedback.base_position.pose.position.x
		y = self.feedback.feedback.base_position.pose.position.y
		w = self.feedback.feedback.base_position.pose.orientation.w
#		orientation_list = [self.feedback.feedback.base_position.pose.orientation.x, self.feedback.feedback.base_position.pose.orientation.y, self.feedback.feedback.base_position.pose.orientation.z, self.feedback.feedback.base_position.pose.orientation.w]	
#		euler = tf.transformations.euler_from_quaternion(orientation_list)
#		heading_tmp = euler[2]
#		if heading_tmp > math.pi:        
#			self.heading = heading_tmp - 2*math.pi
#		elif heading_tmp < -math.pi:
#			self.heading = heading_tmp + 2*math.pi
		self.sample.append([x,y,w])
		self.currentpos = [x,y]
		
		if self.firstgoal == 0 :
		    	self.sendGoal([x,y]) #([x+0.5,y+0.5])
		    	self.firstgoal = 1
		    
		if len(self.sample) > 20 : #20: 15:
			sa = np.array(self.sample)
			mx = np.average(sa[:,0])
			my = np.average(sa[:,1])
			mw = np.average(sa[:,2]) - w
		    
			if calc_distance([x,y],[mx,my]) < 0.015 and abs(mw) < 0.01:
				rospy.loginfo('Stuck. Resetting Goal...')
				self.isstuck = True
				xp, yp = x,y

				if len(self.stuckpnt) > 0:
					sx = self.stuckpnt[0]
					sy = self.stuckpnt[1]
					if calc_distance([x,y],[sx,sy]) < 0.05: #0.1:
						if self.checkpnt < 4 and self.prevpnt != 0:
							self.prevpnt = 0
							rospy.loginfo("Changing Point")
							x = x + 1*offsetx[self.offsetcnt]
							y = y + 1*offsety[self.offsetcnt]
							self.offsetcnt += 1
							if self.offsetcnt > 3:
								self.offsetcnt = 0
					elif self.checkpnt > 0 and self.prevpnt != 0:
			  			self.prevpnt = 1

					if self.checkpnt == 4:
						rospy.loginfo('Adding taboo point')
						self.pntlist.append(self.nextpnt)
						self.checkpnt = 0
				self.stuckpnt = [xp,yp]
				self.sendGoal([x,y])
				self.newGoal = [x, y]
				rospy.loginfo('modified goal is published due to get stuck')
			self.sample = []	


	def VoxelCenter(self,data):
		for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
			self.voxelPts.append([p[0], p[1], p[2]])


	def ScanCloud(self,data):
		for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
			self.scanPts.append([self.poseX + p[0], self.poseY + p[1]])
    	

	def sendNavCb(self,data):
		self.sample = []
		result = data
		frontier_mat = []
		if result.status.text == "Goal reached.":
			if self.flagg == 1 and self.init == 1:
				self.flagg = 0
				rospy.loginfo('Goal reached')
				if calc_distance(self.currentpos,self.nextpnt) < 1 and ( calc_distance(self.scaledprevpnt,self.nextpnt) > self.minScanDist or calc_distance(self.scaledprevpnt,self.newGoal) > self.minScanDist ):
					rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
					self.scanPositions.append([self.currentpos[0], self.currentpos[1], 0, rgb])
					header = Header()
					header.frame_id = "/map"
					pc2 = point_cloud2.create_cloud(header, fields, self.scanPositions)
					pc2.header.stamp = rospy.Time.now()
					self.pub_scan_pose.publish(pc2)
					self.scaledprevpnt = self.currentpos

#					self.pub_start_scan.publish(1)
#					# wait until a static scan is finished
#					while self.finished_scan == 0:
#						pass

					self.finished_scan = 0

				pnts = self.grid.data
				gpnts = np.array(self.newcost).flatten().tolist()
				cnt = 0
				pic_mat = []

				for x in range(self.map_height):
					pic_x = []
					for y in range(self.map_width):
						p = pnts[cnt]
						g = gpnts[cnt]
						if (g>0):
							p = 0
						else:
							if (p ==-1):
								p = 50
							elif (p == 0):
		 						p = 100
							else:
								p = 0
						pic_x.append(p)
						cnt += 1
					pic_mat.append(pic_x)

				ox = self.grid.info.origin.position.x
				oy = self.grid.info.origin.position.y
				mc = interp1d([ox,ox+self.map_width*self.map_res],[0,self.map_width],bounds_error=False)
				mr = interp1d([oy,oy+self.map_height*self.map_res],[0,self.map_height],bounds_error=False)

				r = mr(self.feedback.feedback.base_position.pose.position.y)
				c = mc(self.feedback.feedback.base_position.pose.position.x)

				mw = interp1d([0,self.map_width],[ox,ox+self.map_width*self.map_res],bounds_error=False)
				mh = interp1d([0,self.map_height],[oy,oy+self.map_height*self.map_res],bounds_error=False)

				frontier_pnts = findFrontier(pic_mat)
				xy = []
				for i in range(len(frontier_pnts)):
					xy.append([mw(frontier_pnts[i][0]),mw(frontier_pnts[i][1])])
				self.publishMarker(xy)

				if len(frontier_pnts) > 0:
					idx = find_closest_frontier([int(c),int(r)],frontier_pnts,self.heading)
					nextpnt = frontier_pnts[idx]
					pnt1 = [mw(nextpnt[0]),mh(nextpnt[1])]
					if self.pntlist:
						for i in range(len(self.pntlist)):
							if calc_distance(pnt1,self.pntlist[i]) < self.min_dist:
								frontier_pnts.remove(nextpnt)
								if len(frontier_pnts) > 0:
									idx = find_closest_frontier([int(c),int(r)],frontier_pnts,self.heading)
									nextpnt = frontier_pnts[idx]
									pnt1 = [mw(nextpnt[0]),mh(nextpnt[1])]
									rospy.loginfo('sending number '+str(i+2) + ' in list')
								else:
									rospy.loginfo('Mapping Done')
									self.send = False

					if self.isstuck == False and self.prevpnt != -1:
						if calc_distance(nextpnt,self.prevpnt) < self.min_dist:
							rospy.loginfo('repeated point')
							frontier_pnts.remove(nextpnt)
							if len(frontier_pnts) > 0:
								idx = find_closest_frontier([int(c),int(r)],frontier_pnts,self.heading)
								nextpnt = frontier_pnts[idx]
								rospy.loginfo('Sent second closest')
							else:
								self.send = False
								rospy.loginfo('Mapping Done')

					if self.send == True:
						if self.prevpnt == 0:
							self.checkpnt+=1
							rospy.loginfo('checkpoint increased')
						else:
							self.checkpnt = 0

						self.prevpnt = nextpnt
						self.nextpnt = [mw(nextpnt[0]),mh(nextpnt[1])]
						self.isstuck = False
				
					area = []
					for k in range(0,9) :
						if k == 0 :
							x = self.nextpnt[0]
							y = self.nextpnt[1]
						else :
							ang = 2*math.pi * k / 8
							x = round(self.nextpnt[0] + 1*math.cos(ang))
							y = round(self.nextpnt[1] + 1*math.sin(ang))
						dist = np.sum(np.sqrt(np.sum((np.asarray(self.voxelPts) - np.asarray([x, y, 1.0]))**2, axis=1)))
						area.append(dist)
					max_idx = np.argmax(area)
					if max_idx is not 0:
						ang = 2*math.pi * max_idx / 8
						self.nextpnt[0] = round(self.nextpnt[0] + 1*math.cos(ang))
						self.nextpnt[1] = round(self.nextpnt[1] + 1*math.sin(ang))
					if self.nextpnt[0] > self.min_bndX and self.nextpnt[0] < self.max_bndX and self.nextpnt[1] > self.min_bndY and self.nextpnt[1] < self.max_bndY:
						self.sendGoal(self.nextpnt)
					else:
						self.sendGoal(self.currentpos)
				else:
					rospy.loginfo('No prontier points. Mapping Done!!')
					self.sendGoal([0,0])
					print("Go back to the starting position!!")


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
	

	def publishMarker(self,pnt):
		newMarker = Marker()
		newMarker.header.frame_id = "/map"
		newMarker.header.stamp = rospy.Time.now()
		newMarker.ns = "points"
		newMarker.id = 0
		newMarker.type = newMarker.POINTS #SPHERE #LINE_STRIP
		newMarker.action = newMarker.ADD
		newMarker.pose.orientation.w = 1
		newMarker.scale.x = 0.3 #0.05
		newMarker.scale.y = 0.3
		newMarker.scale.z = 0.3
		newMarker.color.r = 0.0
		newMarker.color.g = 0.0
		newMarker.color.b = 1.0
		newMarker.color.a = 1.0
		for j in range(0,len(pnt)):
			newMarker.points.append(Point(pnt[j][0], pnt[j][1], 0))

		self.line_pub.publish(newMarker)
		print(str(len(pnt)) + ' frontier points marker published!!')


def findFrontier(mat):
    frontier_pnts = []
    # frontier_mat = np.zeros(np.shape(mat),dtype=np.uint8).tolist()
    dx = [0,-1,-1,-1,0,1,1,1]
    dy = [1,1,0,-1,-1,-1,0,1]

    frontier_mat = np.array(mat).astype(np.uint8)
    frontier_mat = cv2.Canny(frontier_mat,100,200)
    # plt.imshow(frontier_mat,cmap='gray',origin='lower')
    # plt.draw()
    # plt.pause(0.001)

    free_pnts = np.asarray(np.where(frontier_mat==255)).T.tolist()
    frontier_mat = np.zeros(np.shape(mat),dtype=np.uint8).tolist()
    row, col = np.shape(mat)
    for j in range(len(free_pnts)):
        r,c = free_pnts[j]
        if mat[r][c] == 100:
            for i in range(8):
                r1 = r + dx[i]
                c1 = c + dy[i]

                if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                    if mat[r1][c1] == 50:
                        frontier_mat[r][c] = 255
                        break
        elif mat[r][c] == 50:
            for i in range(8):
                r1 = r + dx[i]
                c1 = c + dy[i]

                if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                    if mat[r1][c1] == 100:
                        frontier_mat[r][c] = 255
                        break

    frontmat = np.array(frontier_mat).astype(np.uint8)
    kernel = np.ones((5,5), np.uint8)*255
    frontmat = cv2.dilate(frontmat,kernel,iterations=3)
    contour = cv2.findContours(frontmat.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(contour) > 0:
        res = np.array(mat).astype(np.uint8)
        for i in range(len(contour)):
            c = contour[i]
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if radius > 15: #10: #15: #25:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                frontier_pnts.append(center)
                res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),75,2)

    return frontier_pnts


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
    return np.argmin(dist_2*ang_diff)
#    return np.argmin(distance*ang_diff)
#    return np.argmin(dist_2)

if __name__ == "__main__":
    rospy.init_node('explore') #make node
    rospy.sleep(1)
    gc = jackal_explore()
    rospy.sleep(1)
    gc.sendGoal([0.0,0.0])
    rospy.sleep(0.5)
    # plt.figure()
    # plt.show()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
