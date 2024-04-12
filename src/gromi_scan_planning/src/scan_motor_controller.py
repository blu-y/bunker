#!/usr/bin/python

import rospy, sys, serial
from std_msgs.msg import Bool

class ScanMotorController:
    def __init__(self):
        # motor command pub frequency
        self.pub_frequency = 20

        # set up serial device
#        self.serial = serial.Serial(
#            "/dev/top_mc", baudrate=9600, parity=serial.PARITY_EVEN, bytesize=serial.EIGHTBITS
#        )

        # define start and stop commands for the motor
        self.start_command = "0106007D40002812"
        self.stop_command = "0106007D0020180A"

        # define a subscriber to listen for motor inputs
        self.cmd_sub = rospy.Subscriber('/run_scan_motor', Bool, self.set_run_motor, queue_size=1)

        self.run_motor = False
        
    def set_run_motor(self, motor_cmd):
        self.run_motor = bool(motor_cmd.data)
    
    def run_update(self):
        if (self.run_motor):
            # send a command to rotate motor
#            self.serial.write(self.start_command.decode("hex"))
	    pass
        else:
            # send a command to stop motor
#            self.serial.write(self.stop_command.decode("hex"))
	    pass

def main(agrs):
    rospy.init_node("scan_motor_controller")
    mc = ScanMotorController()
    rate = rospy.Rate(mc.pub_frequency)
    while not rospy.is_shutdown():
        mc.run_update()
        rate.sleep()
            
    mc.serial.close()
    print("Shutting down top motor_controller node!")

if __name__ == '__main__':
    main(sys.argv) 
