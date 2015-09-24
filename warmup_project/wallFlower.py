#!/usr/bin/env python
#import everything we need
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
pub=rospy.Publisher("cmd_vel",Twist)
twist=Twist()
rospy.init_node("wallFlower")

def callback(msg):
	#why is the number always the same? the difference is always the same
	diff = msg.ranges[60]-msg.ranges[120]
	print diff

	if abs(diff)>.1:
		
		twist.linear.x=0.1
		twist.angular.z=diff*(0.5)
		print msg.ranges[60]
		print msg.ranges[120]
		#RADIAN per second 6.2 is one revolution 2 pie 
		pub.publish(twist)

	else:
		twist.linear.x=0.1
		twist.angular.z=0
		pub.publish(twist)




while not rospy.is_shutdown(): 	
	rospy.Subscriber("/scan",LaserScan, callback)
	print "sub"
