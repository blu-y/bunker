#!/usr/bin/env python3

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def GroundRemoving(data):
    global pts, free, resolution, arrived, orientation_list

    pts = []
    for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        if p[2] > -0.3:	
    	    pts.append([p[0], p[1], p[2]])
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
	      PointField('y', 4, PointField.FLOAT32, 1),
	      PointField('z', 8, PointField.FLOAT32, 1),
	      ]
    
    header = Header()
    header.frame_id = "base_link"
    pc2 = point_cloud2.create_cloud(header, fields, pts)
 
    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)


print("start point cloud generation")
rospy.init_node("start_point_cloud_generation")
rospy.Subscriber('/velodyne_points', PointCloud2, GroundRemoving)
pub = rospy.Publisher('/vertical_cloud', PointCloud2, queue_size=1)

r = rospy.Rate(50) 
while not rospy.is_shutdown():
	try:
		r.sleep()
		rospy.spin()
	except KeyboardInterrupt:
		print("end program")
		break

