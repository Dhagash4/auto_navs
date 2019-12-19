#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion,quaternion_matrix
import numpy as np
import math

def map_to_base(rot_mo,rot_ob):

	rot_mb = np.dot(rot_mo,rot_ob)

	x = rot_mb[0][3]
	y = rot_mb[1][3]

	f.write("%f %f \n" % (x, y))

def callback_amcl(amcl):

	x1 = amcl.pose.pose.position.x
	y1 = amcl.pose.pose.position.y

	xmo = trans.transform.translation.x
	ymo = trans.transform.translation.y
	orientation_mo = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
	rot_mo = quaternion_matrix(orientation_mo)
	rot_mo[0][3] = xmo
	rot_mo[1][3] = ymo

	xob = trans1.transform.translation.x
	yob = trans1.transform.translation.y
	orientation_ob = [trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z, trans1.transform.rotation.w]
	rot_ob = quaternion_matrix(orientation_ob)
	rot_ob[0][3] = xob
	rot_ob[1][3] = yob

	map_to_base(rot_mo,rot_ob)	
	#print("y: %f"%trans1.transform.translation.x)
	f1.write("%f %f \n" % (x1, y1))

if __name__ == '__main__':
	f= open("odom.txt","w+")
	f1 = open("amcl.txt","w+")
	rospy.init_node('saver', anonymous=True)
	rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,callback_amcl)
	#rospy.Subscriber('/odom',Odometry,callback_odom)
	#rospy.Subscriber('/tf',TFMessage,callback_odom)
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
