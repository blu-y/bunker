#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

def mapCb(data):
    pub.publish(data)


rospy.init_node("change_topic_name")
map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, mapCb,queue_size=1)
pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

r = rospy.Rate(50) 
while not rospy.is_shutdown():
	try:
		r.sleep()
		rospy.spin()
	except KeyboardInterrupt:
		print("end program")
		break

