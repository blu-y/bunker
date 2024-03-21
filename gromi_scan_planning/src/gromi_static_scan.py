#!/usr/bin/env python

from syslog import LOG_INFO
from std_msgs.msg import Bool, Int16
import time, rospy, os, cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
#from lidar_mapping.srv import PCD

class gromi_scan:
    def __init__(self):
	self.trigger = 0
	self.cnt = 0
	self.image = None
	self.rotation_time = 2.0	# 100.0
	self.num_of_pic = 4		# 8
	self.time_between_pics = self.rotation_time / self.num_of_pic
	self.curr_dir = os.path.dirname(os.path.realpath(__file__))
	self.bridge = CvBridge()

	self.sub = rospy.Subscriber("/start_scan", Int16, self.start_motor)
    	self.publisher = rospy.Publisher("/run_scan_motor", Bool, queue_size=1)
    	self.pub_finish_scan = rospy.Publisher("/finish_scan", Int16, queue_size=1)

    def get_image(self,cnt):
	self.cnt += 1
	print("took a picture {}".format(self.cnt))
#        image_msg = rospy.wait_for_message("/usb_cam/image_raw/compressed", CompressedImage)
#        try:
#            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
#            path = os.path.join(curr_dir, "result", "image_{}.jpg".format(self.cnt))
#            cv2.imwrite(path, cv_image)
#        except CvBridgeError as e:
#            print(e)

#    def self.send_a_request_pcd_save_client(input):
#        print("waiting for the service.")
#        rospy.wait_for_service('/save_pcd_data')
#        print("pcd_server starts")
#        try:
#            pcd_server = rospy.ServiceProxy('/save_pcd_data', PCD)
#            output = pcd_server(input)
#            return output
#        except rospy.ServiceException as e:
#            print("Service call failed: %s"%e)

    def start_motor(self,data):
        self.trigger = data.data
        if self.trigger == 1:
	    print "start to take pictures"
            self.get_image(self.cnt)
#            output_server = self.send_a_request_pcd_save_client(0)

            # send a command to rotate motor
	    print "start to rotate scan motor"
            self.publisher.publish(True)

            time.sleep(self.time_between_pics)
            for i in range(0, self.num_of_pic - 1):
                self.get_image(self.cnt)
                time.sleep(self.time_between_pics)
    
            # send a command to stop motor
            self.publisher.publish(False)
#            os.system("rosnode kill tf_laser")		# commented by pileun
    
#            output_server = self.send_a_request_pcd_save_client(1)
            self.trigger = 0
	    self.cnt = 0
	    self.pub_finish_scan.publish(1)
	    print "finish to rotate scan motor"

def main():
    rospy.init_node("gromi_static_scan")
    gs = gromi_scan()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

    print("Shutting down gromi_static_scan node!")

if __name__ == "__main__":
    main()
