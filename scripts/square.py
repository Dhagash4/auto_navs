#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535

def move(pub,speed,distance):

	
	vel_msg = Twist()
	rospy.Rate(10)	
	vel_msg.linear.x = speed	
	t0 = rospy.Time.now().to_sec()
	current_distance = 0
		
	while(current_distance < distance):
	
		pub.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_distance = abs(speed) * (t1 - t0)
		
	vel_msg.linear.x = 0
	pub.publish(vel_msg)

def rotate(pub,speed,angle):

	vel_msg = Twist()

	rospy.Rate(10)

	speed = speed*2*PI/360
	relative_angle = angle*2*PI/360 
	
	
	vel_msg.angular.z = speed
	
		
	t0 = rospy.Time.now().to_sec()
	current_angle = 0
		
	while( current_angle < relative_angle):
	
		pub.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = abs(speed) * (t1 - t0)
		
	vel_msg.angular.z = 0
	pub.publish(vel_msg)


def square(pub,distance):
	

    	    
    	    move(pub,1, distance)
    	    rotate(pub,0.7,90)
	    move(pub,1, distance)
	    rotate(pub,0.7,90)
	    move(pub,1, distance)
	    rotate(pub,0.7,90)		
	    move(pub,1, distance)


def callback_odom(data):

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z

	x_1 = data.pose.pose.orientation.x
	y_1 = data.pose.pose.orientation.y
	z_1 = data.pose.pose.orientation.z
	w_1 = data.pose.pose.orientation.w


	f.write("%f %f %f %f %f %f\n" % (x, y, z, x_1, y_1, z_1))

    	    



if __name__ == '__main__':

	rospy.init_node('turtlesim_move')
	velo_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	sub_odom = rospy.Subscriber('/turtle1/pose', Odometry, callback_odom)
	f = open("odom_traj.txt","a+")

	try:
            distance = input("Distance : ")
	    square(velo_pub,distance)

	    rospy.spin()
            
            # The following functions need to function correctly.
            # Uncomment and modify parameters to test them
            #move(velo_pub,0.5,3)
            #rotate(velo_pub, 10,90)

	except rospy.ROSInterruptException:
		pass

