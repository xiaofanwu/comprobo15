#!/usr/bin/env python
# import roslib; roslib.load_manifest('robot_mover')
import rospy

from geometry_msgs.msg import Twist

rospy.init_node('square')

def mover():
    pub = rospy.Publisher('cmd_vel', Twist)

    twist = Twist()
    twist.linear.x = .1
    pub.publish(twist)
    rospy.sleep(10)

    twist.angular.z = 1
    pub.publish(twist)
    rospy.sleep(1.5);


r=rospy.Rate(10)

while not rospy.is_shutdown(): 
    mover()
