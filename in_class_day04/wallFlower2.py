#!/usr/bin/env python
#import everything we need
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
twist=Twist()
global align
align=False

def callback(msg):
	#why is the number always the same? the difference is always the same
	
	lissixty = [msg.ranges[60], msg.ranges[59],msg.ranges[61]]
	#rint msg.ranges[60]
	newsixty = []

	for elements in lissixty:
		if elements != 0:
			newsixty.append(elements)
	#return newsixty
	

	avsixty = sum(newsixty)/float(len(newsixty))

	listwelve = [msg.ranges[120],msg.ranges[121],msg.ranges[119]]
	newtwelve = []

	for k in listwelve:
		if k != 0:
			newtwelve.append(k)

	#return newtwelve

	avtwelve = sum(newtwelve)/float(len(newtwelve))
	

	diff = avtwelve - avsixty

	# if msg.ranges[60]==0 or msg.ranges[120]==0:
	# 	pass
	if abs(diff)>.001:
			twist.linear.x=0.1
			twist.angular.z=diff*(2) #RADIAN per second 6.2 is one revolution 2 pie 
			pub.publish(twist)
	else:
		align=True


def keepStraight():
	if align==True:

		twist.linear.x=0.1
		twist.angular.z=0
		pub.publish(twist)
		



if __name__ == "__main__":
	rospy.init_node('wallFlower1')
	#create a function that moves the robot forward
	
	pub=rospy.Publisher("cmd_vel",Twist,queue_size=5)
	laserScan = rospy.Subscriber("/scan",LaserScan, callback)

	while not rospy.is_shutdown(): 
		keepStraight()






