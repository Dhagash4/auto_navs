#!/usr/bin/env python
'''
Created on Ocotober 27,2018

@author Dhagash Desai, Final Year IIT Jodhpur

Some portion borrowed from book ROS-Robotics-Projects
by Lentin Joseph
'''

import rospy
import sys
import time
import math
from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Int16, Int32, Int64, Float32, String, UInt64
from geometry_msgs.msg import Twist 


class arduino_launch(object):

	def __init__(self):

		self.counter = 0

		self.linear = 0.0
		self.angular = 0.0
		self.linear_in = 0.0
		self.angular_in = 0.0
		self.left_encoder_value = 0
		self.right_encoder_value = 0
		self.left_RPM = 0
		self.right_RPM = 0
		self.left_set_RPM = 0
		self.right_set_RPM = 0
		self.right_out_PWM = 0
		self.left_out_PWM = 0
		self.left_time = 0
		self.current_time = 0



		self.lwheel_speed = 0
		self.rwheel_speed = 0

		self.lastupdate_microsec = 0
		self.seconds_since_lastupdate = 0

		port = rospy.get_param("~port","/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 9600))

		#rospy.loginfo("Starting serial port with port",port,"and baudRate",baudRate)
		self.SerialDataGateway = SerialDataGateway(port, baudRate, self.HandleReceivedLine)
		rospy.loginfo("Starting serial communication")

		self.left_encoder = rospy.Publisher('lencoder', Int64, queue_size = 10)
		self.right_encoder = rospy.Publisher('rencoder', Int64, queue_size = 10)
		self.leftPWM = rospy.Publisher('lPWM', Int64, queue_size = 10)

		self.SerialPublisher = rospy.Publisher('serial', String,queue_size=10)

		self.lmotor_speed = rospy.Subscriber('cmd_vel',Twist,self.Update_Twist)
		#self.rmotor_speed = rospy.Subscriber('rwheel_speed',Float32,self.Update_Right_Speed)

	def Update_Twist(self,msg):


		self.linear =  100 * msg.linear.x
		#self.linear = msg.linear.x
		self.angular = 100 * msg.angular.z
		

		#print self.angular
		twist_message = 's %d %d\r' %(int(self.linear),int(self.angular))

		self.WriteSerial(twist_message)

	
	def HandleReceivedLine(self,  line):
		self.counter = self.counter + 1
		#self.SerialPublisher.publish(String(str(self.counter) + ", in:  " + line))


		if(len(line) > 0):

			lineParts = line.split('\t')

			try:

				if(lineParts[0] == 'e'):

					self.left_encoder_value = long(lineParts[1])
					self.right_encoder_value = long(lineParts[2])
					self.left_encoder.publish(self.left_encoder_value)
					self.right_encoder.publish(-(self.right_encoder_value))
                                        # Minus sign as it is taking CCW as CW

				if(lineParts[0] == 'r'):

					self.left_RPM = long(lineParts[1])
					self.right_RPM = long(lineParts[2])
					
				if(lineParts[0] == 'a'):

					self.left_set_RPM = long(lineParts[1])
					self.right_set_RPM = long(lineParts[2])
					
				if(lineParts[0] == 'p'):

					self.left_out_PWM  = long(lineParts[1])
					self.right_out_PWM  = long(lineParts[2])
					self.leftPWM.publish(self.left_out_PWM)

				if(lineParts[0] == 'i'):

					self.linear_in  = long(lineParts[1])
					self.angular_in  = long(lineParts[2])

				
					
			except:
				rospy.logwarn("Error in Sensor values")
				rospy.logwarn(lineParts)
				pass

	def WriteSerial(self,message):

		#rospy.loginfo('i %d %d : a %d : e %d : r %d : p %d ' %(int(self.linear_in),int(self.angular_in),int(self.left_set_RPM), int(self.left_encoder_value), int(self.left_RPM),int(self.left_out_PWM)))
		
		rospy.loginfo('i %d %d : a %d : e %d %d : r %d : p %d ' %(int(self.linear_in),int(self.angular_in),int(self.right_set_RPM), int(-(self.right_encoder_value)), int(self.left_encoder_value),int(self.right_RPM),int(self.right_out_PWM)))
		
		self.SerialDataGateway.Write(message)

	def Start(self):

		rospy.logdebug("Starting")
		self.SerialDataGateway.Start()

	def Stop(self):

		rospy.logdebug("Stoping")
		self.SerialDataGateway.Stop()

	def Reset_Arduino(self):
		print "Reset"
		reset = "r\r"
		self.WriteSerial(reset)
		time.sleep(1)
		self.WriteSerial(reset)
		time.sleep(2)

	
if __name__ == '__main__':
		
		rospy.init_node('arduino_ros', anonymous = True)
		arduino = arduino_launch()

		try:

			arduino.Start()
			
			rospy.spin()

		except rospy.ROSInterruptException:

			ros.logwarn("Error Starting the node")

		arduino.Reset_Arduino()
		arduino.Stop()

		

