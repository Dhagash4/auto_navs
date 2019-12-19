#!/usr/bin/env  python

import rospy
import roslib
from nav_msgs.msg import Odometry

def callback(data):

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z

	x_1 = data.pose.pose.orientation.x
	y_1 = data.pose.pose.orientation.y
	z_1 = data.pose.pose.orientation.z
	w_1 = data.pose.pose.orientation.w


	f.write("%f %f %f %f %f %f\n" % (x, y, z, x_1, y_1, z_1))

	print "Data Written"


if __name__ == '__main__':

	f= open("poses_trajectory_1.txt","w+")

	rospy.init_node('extract_poses')
	laser_sub=rospy.Subscriber("/odom",Odometry,callback)
	rospy.spin()