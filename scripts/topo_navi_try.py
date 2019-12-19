#!/usr/bin/env  python

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
import numpy as np
import math
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion,quaternion_matrix

ranges = []
distance = 0.0
PI = 3.1415926535
location = None




def navigation(loc,theta):

	print loc

	vel_msg = Twist()

	if loc == "racks":

		vel_msg.linear.x = 0.3
		vel_msg.angular.z = 0.0

		pub.publish(vel_msg)

	elif loc == "corridor" and theta < 1.3:


		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.3

		pub.publish(vel_msg)

		print "rotate"

	elif loc == "corridor" and theta > 1.3:

		vel_msg.linear.x = 0.3
		vel_msg.angular.z = 0.0 

		pub.publish(vel_msg)

		print "forward"

	elif loc == "final_rack" and theta > 0.2:

		vel_msg.linear.x = 0.0
		vel_msg.angular.z = -0.3 

		pub.publish(vel_msg)

		print "turn"

	elif loc == "final_rack" and theta < 0.2:

		vel_msg.linear.x = 0.3
		vel_msg.angular.z = 0.0 

		pub.publish(vel_msg)

		print "forward"
	
	elif loc == "Reached":

		vel_msg.linear.x = 0.3
		vel_msg.angular.z = 0.0 

		pub.publish(vel_msg)

		print "forward"

		if loc =="stop":

			vel_msg.linear.x = 0.0
			vel_msg.angular.z = 0.0 

			pub.publish(vel_msg)

			print "stop"





def callback_laser(data):


	ranges = data.ranges
	range_shape = (math.ceil(math.degrees(data.angle_max)) - math.floor(math.degrees(data.angle_min)))/math.ceil(math.degrees(data.angle_increment))
	#print range_shape

	theta = 90
	mid = 134

	x1 = ranges[mid + theta] *  math.cos(theta)
	y1 = ranges[mid + theta] *  math.sin(theta)

	y2 = ranges[mid - theta] *  math.sin(-theta)
	x2 = ranges[mid - theta] *  math.cos(-theta)

	distance = math.sqrt((x2-x1)**2+(y2-y1)**2)

	
	xmo = trans.transform.translation.x
	ymo = trans.transform.translation.y
	orientation_mo = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
	rot_mo = quaternion_matrix(orientation_mo)
	rot_mo[0][3] = xmo
	rot_mo[1][3] = ymo

	xob = trans1.transform.translation.x
	yob = trans1.transform.translation.y
	orientation_ob = [trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z, trans1.transform.rotation.w]
	[roll,pitch,yaw] = euler_from_quaternion(orientation_ob)
	rot_ob = quaternion_matrix(orientation_ob)
	rot_ob[0][3] = xob
	rot_ob[1][3] = yob

	rot_mb = np.dot(rot_mo,rot_ob)

	x = rot_mb[0][3]
	y = rot_mb[1][3]

	
	#print x,y,yaw
	
	if x > 7.2 and x <13 and y > -15 and y < 13:

		

		if y > 8 and y < 9 or x > 9:

			location = "final_rack"
		else:
			
			location = "corridor"
		

	elif distance < 25 and x < 7.2:

		location = "racks"
		
	elif x>13 and x < 20:

		location = "Reached"

		if x >15:

			location = "stop"



	navigation(location,yaw)


if __name__ == '__main__':

	rospy.init_node('topo_navigation')
	
	#Try to use message filter for data to be proceesed from a single callback odom and laserscan data to determine the location of the robot

	laser_sub=rospy.Subscriber("/laserscan",LaserScan,callback_laser)

	#rospy.Subscriber("/odom",Odometry,topology)
	
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform("map", "odom", rospy.Time())
			trans1 = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass
		
		rate.sleep()
	#rospy.spin()