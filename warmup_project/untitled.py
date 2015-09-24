#!/usr/bin/env python
#import everything we need
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
sub=rospy.Subscriber("/scan",LaserScan, callback)
pub=rospy.Publisher("cmd_vel",Twist)
twist=Twist()
rospy.init_node("wallFlower")

def callback(msg):

	while msg.ranges[45]!= msg.ranges[315]:
		
		twist.linear.x=1
		twist.angular.z=(msg.ranges[45]-msg.ranges[315])*0.1


	twist.linear.x=1
	twist.angular.z=0

	



while not rospy.is_shutdown(): 
	pub.publish(twist)
 	callback()
