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
from geometry_msgs.msg import PoseWithCovarianceStamped	



v = 0.0 
w = 0.0 

v_max = 0.5
w_max = 0.3

x = 0.0
y = 0.0
yaw = 0.0

n = True
# 9.73 = x and y = 7.36
i = 0

def waypoint(dis):

	global n,x,y,yaw,v,w,v_max,w_max,i

	vel_msg = Twist()

	#print yaw
	
	waypoints = [[9.73,7.36],[16.89,8.14]]

	if dis < 26 and n == True:

		vel_msg.linear.x = 0.3
		vel_msg.angular.z = 0.0
		print "Racks"
		pub.publish(vel_msg)
	
	else:

		print "Block executed"
		n = False
		
		#print len(waypoints)
		
		while i < len(waypoints):

			print "Moving and aligning towards waypoint ", i+1

			x_final = waypoints[i][0]
			y_final = waypoints[i][1]

			distance = math.sqrt((x_final-x)**2 + (y_final - y)**2)

			while distance >= 0.5:
			
				theta_cal = math.atan2(y_final-y,x_final-x)
				error_theta = theta_cal - yaw
				#print "theta ",error_theta

				if math.fabs(error_theta) > 0.2:
					#print "theta loop"

					if error_theta>0:

						v = 0
						w = w_max * math.fabs(error_theta)
					else:
						v = 0
						w = -w_max * math.fabs(error_theta)


				else:

					w = 0

					if v < v_max:

						v = v + 0.01 * 1
					else:

						if v == v_max:

							v= v_max

			
				vel_msg.linear.x = v
				vel_msg.angular.z = w

				pub.publish(vel_msg)

				#print "x_final %f y_final %f" %(x_final,y_final)

				distance = math.sqrt((x_final-x)**2 + (y_final - y)**2)

				#print "distance ", distance

			i += 1



		
			

			
			#print x_final,y_final

		print "out of for loop"
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0

		pub.publish(vel_msg)



def callback_amcl(amcl):


	global x,y,yaw
	x = amcl.pose.pose.position.x
	y = amcl.pose.pose.position.y
	orientation = [amcl.pose.pose.orientation.x, amcl.pose.pose.orientation.y, amcl.pose.pose.orientation.z, amcl.pose.pose.orientation.w]
	[roll,pitch,yaw] = euler_from_quaternion(orientation)



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

	waypoint(distance)




if __name__ == '__main__':
	
	rospy.init_node('topo_navigation')
	
	#Try to use message filter for data to be proceesed from a single callback odom and laserscan data to determine the location of the robot

	laser_sub=rospy.Subscriber("/laserscan",LaserScan,callback_laser)
	amcl_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,callback_amcl)

	#rospy.Subscriber("/odom",Odometry,topology)
	
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10)

	rospy.spin()