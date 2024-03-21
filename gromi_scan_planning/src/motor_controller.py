#!/usr/bin/python

import rospy, sys, serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class MotorController:
    def __init__(self):
        # Conversion factor for converting m/s to a data value for the motor controller.
        # Setting this to a higher value will lead to the robot moving faster for the same 
        # speed input.
        self.mps_2_data_coeff = 212.6
        # Conversion factor for converting rad/s to a data value offset for the motor 
        # controller. Setting this to a higher value will lead to the robot rotating faster 
        # for the same angular speed input. 
        self.radps_2_data_coeff = 233.9
        # The smoothing coefficient controls how much smoothing is applied. Must be between [0, 1). 
        # Setting to 0.0 would apply zero smoothing while setting to 1.0 would make the system 
        # unresponsive. Keep in mind that higher publishing frequency requires a higher smoothing 
        # coefficient for the same result
        self.smoothing_coeff = 0.8
        # The controller's running frequency in Hz
        self.pub_frequency = 20.0

        # Initialize command velocity subscriber
# revised by pileun (2022.11.1)
# 	 self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.set_goal_vel, queue_size=1)
	self.cmd_sub = rospy.Subscriber('/cmd_vel/nav', Twist, self.set_goal_vel, queue_size=1)

        # Initialize debug wheel velocity publishers
        self.l_wheel_vel = rospy.Publisher('/left_wheel_vel', Int8, queue_size=1)
        self.r_wheel_vel = rospy.Publisher('/right_wheel_vel', Int8, queue_size=1)

        # The current linear velocity in m/s
        self.forward_vel = 0.0
        # The current angular velocity in rad/s
        self.angular_vel = 0.0
        # The current goal linear velocity in m/s
        self.goal_forward_vel = 0.0
        # The current goal angular velocity in m/s
        self.goal_angular_vel = 0.0

        # Initialize serial communication with the two bottom motor controllers
#        serial_port = '/dev/bottom_mc'
#        print('Connecting to bottom motor controllers on port: ', serial_port)
#        self.serial = serial.Serial(
#            port = serial_port,
#            baudrate = 115200,
#            parity = serial.PARITY_NONE,
#            stopbits = serial.STOPBITS_ONE,
#            bytesize = serial.EIGHTBITS
#        )

    
    def set_goal_vel(self, cmd_vel):
        self.goal_forward_vel = -cmd_vel.linear.x
        self.goal_angular_vel = cmd_vel.angular.z
    
    def run_update(self):
        self.forward_vel = self.smoothing_coeff*self.forward_vel + (1.0-self.smoothing_coeff)*self.goal_forward_vel
        self.angular_vel = self.smoothing_coeff*self.angular_vel + (1.0-self.smoothing_coeff)*self.goal_angular_vel

        # calculate the left and right wheel velocities using coefficients
        left_wheel_vel = self.forward_vel*self.mps_2_data_coeff - self.angular_vel*self.radps_2_data_coeff
        right_wheel_vel = self.forward_vel*self.mps_2_data_coeff + self.angular_vel*self.radps_2_data_coeff

        # if velocity too high, scale down the wheel velocities to fit between 0 and 127 while 
        # preserving linear to angular velocity ratio
        max_vel = max(abs(left_wheel_vel), abs(right_wheel_vel))
        if max_vel > 127.0:
            scale_factor = 127.0/max_vel
            left_wheel_vel *= scale_factor
            right_wheel_vel *= scale_factor
        
        # send left motor command
        l_wheel_abs_speed = int(abs(left_wheel_vel))
#        if left_wheel_vel <= 0:
#            self.serial.write(chr(128)) #communicate on channel 128
#            self.serial.write(chr(0)) #drive motor 1 forward dir (which drive robot backward)
#            self.serial.write(chr(l_wheel_abs_speed)) #set motor speed
#            self.serial.write(chr((128+0+l_wheel_abs_speed)&0x7f)) #send checksum
#        else:
#            self.serial.write(chr(128)) #communicate on channel 128
#            self.serial.write(chr(1)) #drive motor 1 backward dir (which drive robot forward)
#            self.serial.write(chr(l_wheel_abs_speed)) #set motor speed
#            self.serial.write(chr((128+1+l_wheel_abs_speed)&0x7f)) #send checksum

        # send right motor command
        r_wheel_abs_speed = int(abs(right_wheel_vel))
#        if right_wheel_vel <= 0:
#            self.serial.write(chr(128)) #communicate on channel 128
#            self.serial.write(chr(4)) #drive motor 2 forward dir (which drive robot backward)
#            self.serial.write(chr(r_wheel_abs_speed)) #set motor speed
#            self.serial.write(chr((128+4+r_wheel_abs_speed)&0x7f)) #send checksum
#        else:
#            self.serial.write(chr(128)) #communicate on channel 128
#            self.serial.write(chr(5)) #drive motor 2 backward dir (which drive robot forward)
#            self.serial.write(chr(r_wheel_abs_speed)) #set motor speed
#            self.serial.write(chr((128+5+r_wheel_abs_speed)&0x7f)) #send checksum

        # publish wheel velocities for debug purposes
        self.l_wheel_vel.publish(Int8(int(left_wheel_vel)))
        self.r_wheel_vel.publish(Int8(int(right_wheel_vel)))



def main(agrs):
    rospy.init_node("motor_controller")
    mc = MotorController()
    rate = rospy.Rate(mc.pub_frequency)
    while not rospy.is_shutdown():
        mc.run_update()
        rate.sleep()    
    
    print("Shutting down bottom motor_controller node!")

if __name__ == '__main__':
    main(sys.argv) 
