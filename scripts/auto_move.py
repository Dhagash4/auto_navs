#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


#Current Position
xcur = 0
ycur = 0
thetacur = 0

#Input Position


a = 0
b = 0

xin = 0
yin = 0
thetain = 0

	



def odom_cb(data):

	a= 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
	b= 1.0 - (2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z))
	theta_cur = math.atan2(a,b)
	xcur = data.pose.pose.position.x 
	ycur = data.pose.pose.position.y
	#print 'cur: %f %f' %(xcur,ycur)



def main():
	
	VMAX = 0.1
	WMAX = 0.1

	v = 0
	w = 0

	v_reduced = 0
	w_reduced = 0

	#Previous Data

	xprev = 0
	yprev = 0
	thetaprev = 0
	thetacal = 0


#Error

	error_theta = 0
	error_pos = 0
	del_theta_error = 0.1
	del_error_pose = 0.1

#Distance and Variables 

	distance = 0
		
	xin = input("Enter X")
	yin = input("Enter y")

	rospy.init_node('auto_controller', anonymous=True)
	pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
	sub = rospy.Subscriber('/odom', Odometry,odom_cb)
	#rate = rospy.Rate(30)
	#rospy.spin()

	twist = Twist()

	#print " Node Initialized" 

	while not rospy.is_shutdown():
	
		#print "Started"

		

		xprev = xcur;
		yprev = ycur;
		thetaprev = thetacur;
		distance = math.sqrt(((xin - xcur) * (xin - xcur)) + ((yin - ycur) * (yin - ycur)))
		
		print distance

		if (distance>=del_error_pose):
		
			#print "started"
			thetacal = math.atan2(yin - ycur, xin - xcur)
			error_theta = thetacal - thetacur
			#print error_theta

			if(abs(error_theta)>del_theta_error):
		
				#print "I am here"

				if(error_theta > 0):

					w = WMAX * abs(error_theta)
					
				else:

					w = -(WMAX * abs(error_theta))
			else:

				w = 0
				v = VMAX
				print v
	
		#else:
			#if (xin == math.floor(xcur) and yin == math.floor(ycur)):
				#w = 0
				#v = 0

		
		#rospy.loginfo('i %f %f : d %f %f' %(v,w,xcur,ycur))
		
		twist.linear.x = v
		twist.angular.z = w	
		pub.publish(twist)

		distance = math.sqrt((xin-xcur)*(xin-xcur) + (yin-ycur)*(yin-ycur));
		#rate.sleep()
		


if __name__=='__main__':
    try:
    	
    	

	main()
	   

    except rospy.ROSInterruptException:
        pass

