#!/usr/bin/env python

"""

Implementation for the following task
Demand 1: Write a FLIR BFS Series GIGE version of the ROS camera node (ROS version is kinetic) 

"""

import PySpin
import os
import rospy 
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

"""
Functions for configuring the exposure and white balance provided input by user

"""

def configure_exposure(cam):

    print "*** CONFIGURING EXPOSURE ***\n"

    try:
 

        if cam.ExposureAuto.GetAccessMode() != PySpin.RW:
            print "Unable to disable automatic exposure. Aborting..."
            

        cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        print "Automatic exposure disabled..."

        if cam.ExposureTime.GetAccessMode() != PySpin.RW:
            print "Unable to set exposure time. Aborting..."
          
        # Ensure desired exposure time does not exceed the maximum
        exposure_time_to_set = rospy.get_param('exposure_value')
        exposure_time_to_set = min(cam.ExposureTime.GetMax(), exposure_time_to_set)
        cam.ExposureTime.SetValue(exposure_time_to_set)

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex


def configure_whitebalance(cam):


	print "*** CONFIGURING WHITE BALANCE ***\n"

    try:
 

        if cam.BalanceWhiteAuto.GetAccessMode() != PySpin.RW:
            print "Unable to disable automatic white balance. Aborting..."
            

        cam.BalanceWhiteAuto.SetValue(PySpin.BalanceWhiteAuto_Off)
        print "Automatic white balance disabled..."

          
        # Ensure desired exposure time does not exceed the maximum
        white_balance_to_set = rospy.get_param('white_balance')
        upper,lower = white_balance_to_set['upper'],white_balance_to_set['lower']
        white_balance_to_set_upper = min(cam.BalanceWhiteAutoUpperLimit.GetMax(), upper)
        white_balance_to_set_lower = max(cam.BalanceWhiteAutoLowerLimit.GetMax(), lower)
        cam.BalanceWhiteAutoUpperLimit.SetValue(white_balance_to_set_upper)
        cam.BalanceWhiteAutoLowerLimit.SetValue(white_balance_to_set_lower)

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex

"""
I am using Spinnaker library to access the data from FLIR Camera GIGE as this library has official support from FLIR.

In the first block of the code I am creating the instance of the system and first finding the number of cameras in the system, then initializing
camera for our purpose and I will be taking the parameters from ROS Parameter Server if someone defines the parameters other then default then it 
will call configure function for exporsure and white balance respectively I am using in built API from Spinnaker for chaning the white balance and
exposure.This is the first part of the code and here the camera mode is set to be continuous so that we get all the frames untill ROS is working.

"""      

def main():

	system = PySpin.System.GetInstance()
	cam_list = system.GetCameras()

	if cam_list.GetSize() == 0:
	    print("no cameras found, aborting")
	    system.ReleaseInstance()
	    del system
	    sys.exit()

	camera_serial = args["camera"]
	if camera_serial == "0":
	    camera_serial = cam_list.GetByIndex(0).GetUniqueID()
	    print("no camera specified (use -c), using the first one in the list {}".format(camera_serial))
	cam = cam_list.GetBySerial(camera_serial)

	if rospy.get_param('exposure_value') != 0:

		configure_exposure(cam)


	white_balance = rospy.get_param('white_balance')
	upper,lower = white_balance['upper'],white_balance['lower']

	if upper != 0 or lower != 0:

		configure_whitebalance(cam)


	try:


	    cam.Init()
	    cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
	    cam.BeginAcquisition()

	except:
	    print("error initializing camera {}".format(camera_serial))
	    sys.exit()


	"""

	In the next block when ROS Server is on first I take the data from FLIR camera and covert it to Opencv, the purpose for this is two fold we cant do
	image processing from the data obtained by ROS so first I am converting the data from FLIR camera to OpenCV format so that we can directly use that 
	image for more processing and same time converting the OpenCV image to ROS and publishing at the rate of 10Hz and when ROS Server is shutdown first 
	we stop the acquisition and deinitialize the camera and delete all the unnecessary variables.

	"""
	while not rospy.is_shutdown():


		image = cam.GetNextImage()

		if image.IsIncomplete():
			pass

		else:

			img_conv = img.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)
			image_cv = image_conv.GetNDArray()
			# cvi = np.frombuffer(image_data, dtype=np.uint8)            
			# cvi = cvi.reshape((i.GetHeight(),i.GetWidth(),3))
			
			try:
				image_pub.publish(bridge.cv2_to_imgmsg(image_cv,"brg8"))
				rate.sleep()

			except CvBridgeError as e:
				print e


		image.Release()
		del image


	cam.EndAcquisition()
	cam.DeInit()
	del cam
	del cam_list




if __name__ == '__main__':


	ap = argparse.ArgumentParser()
	ap.add_argument("-c", "--camera", type=str, default="0", help="camera by id")
	args = vars(ap.parse_args())

	bridge = CvBridge()

	rospy.set_param('white_balance',{'upper': 0.0, 'lower':0.0})
	rospy.set_param('exposure_value',0.0)
	
	rospy.init_node('image_converter', anonymous=True)
	image_pub = rospy,Publisher("/image_raw",Image, queue_size=10)
	
	rate = rospy.Rate(10)

	main()

"""
Developed by Dhagash Desai, I was not given the hardware so from reading the docs on PySpin given by FLIR I came up with this simple implementation of 
task given to me

"""