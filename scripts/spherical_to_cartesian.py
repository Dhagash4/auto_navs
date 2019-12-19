#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
import numpy as np
import math
from sensor_msgs.msg import LaserScan

def spherical_to_cartesian(ranges,angle_max,angle_min,angle_increment):

	i = 0
	j = angle_min
	array= []
	
	while i <= np.size(ranges) and j <= angle_max:

		x = ranges[i] * math.cos(j)
		y = ranges[i] * math.sin(j)
		array.append([x,y])

		i = i+1
		j = j + angle_increment


def callback_laser(data):


	ranges = data.ranges
	range_shape = math.degrees(data.angle_max) - math.degrees(data.angle_min)/math.degrees(data.angle_increment)

	spherical_to_cartesian(ranges,math.degrees(data.angle_max),math.degrees(data.angle_min),math.degrees(data.angle_increment))

	


if __name__ == '__main__':

	rospy.init_node('topo_navigation')
	laser_sub=rospy.Subscriber("/scan",LaserScan,callback_laser)
	rospy.spin()