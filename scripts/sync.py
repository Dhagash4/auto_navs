#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber
#from msg._SubmapList import SubmapList
from tf2_ros import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

count = 0
def callback(occMsg, imageMsg):
	global count
	#print("Recived messages.")
	#print("Resloution: %f, Width: %f, Height: %f, Origin: %f" % occMsg.info.resloution, occMsg.info.width, occMsg.info.height, occMsg.info.origin)
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(imageMsg, "rgb8")
	grid = np.asarray(occMsg.data, dtype=np.int8).reshape(occMsg.info.height, occMsg.info.width)


	cv2.imwrite(params["dir"]+"image"+str(count)+".jpg",cv_image)
	np.save(params["dir"]+"file"+str(cnt2), grid)
	print grid

	count += 1
	print count

def main():

	params = {"dir" : "/media/tushar/data1/rapyuta/frontier_data/Image","dir_2" : "/media/tushar/data1/rapyuta/frontier_data/Frontiers"}
	rospy.init_node("main", anonymous=True)
	occSub = Subscriber("/localMap2", OccupancyGrid)
	imageSub = Subscriber("/camera/color/image_raw", Image)
	# tfSub = Subscriber("/tf", TFMessage)

	ats = ApproximateTimeSynchronizer([occSub, imageSub], queue_size=5, slop=0.1)
	ats.registerCallback(callback)
	rospy.spin()

if __name__ == '__main__':
	main()