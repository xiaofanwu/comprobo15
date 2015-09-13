#!/usr/bin/env python

import tty
import select
import sys
import termios
import rospy


from geometry_msgs.msg import Twist

rospy.init_node('teleop')

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

settings = termios.tcgetattr(sys.stdin)
key = None

twist = Twist()
# while key=='k':
# 	print "hey"
	# twist.linear.x = 0.1
	# print twist.linear.x


while key != '\x03':
	key = getKey()
	while key=="i":
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		twist.linear.x = 0.1
		twist.angular.z = 0
		pub.publish(twist)
		break
	while key==",":
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		twist.linear.x = -0.1
		twist.angular.z = 0
		pub.publish(twist)
		break
	while key=="k":
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		twist.linear.x = 0
		twist.angular.z=0
		pub.publish(twist)
		break
	while key=="j":
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		twist.angular.z = 0.5
		twist.linear.z = 0
		pub.publish(twist)
		break
	while key=="l":
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		twist.angular.z = -0.5
		twist.linear.z = 0
		pub.publish(twist)
		break
