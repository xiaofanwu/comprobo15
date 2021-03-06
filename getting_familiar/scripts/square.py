#!/usr/bin/env python
#import everything we need
import rospy

from geometry_msgs.msg import Twist


rospy.init_node('square')
#create a function that moves the robot forward then turn 90 degrees
def mover():
    pub = rospy.Publisher('cmd_vel', Twist)

    twist = Twist()
    twist.linear.x = .1  #0.1 m/s x 10 =1 m/s 
    pub.publish(twist)
    rospy.sleep(10)

    twist.angular.z = 1
    twist.linear.x=0
    pub.publish(twist)
    rospy.sleep(1.5);


r=rospy.Rate(10)

while not rospy.is_shutdown(): 
    mover()
