#!/usr/bin/env python
#import everything we need
import rospy
from geometry_msgs.msg import Twist
import neato_node.Bump
global stop
stop=True


def bump_event_callback(msg):

	if msg.frontLeft==1 or msg.frontRight==1:
		stop=True

		

#movwe needs to check if

def mover(pubs):
	twist = Twist()

	if stop==False:
    	twist.linear.x = .1  #0.1 m/s x 10 =1 m/s 
    	pubs.publish(twist)
    else: 
    	twist.linear.x = 0  #0.1 m/s x 10 =1 m/s 
    	pubs.publish(twist)

    rospy.sleep(10)




if __name__ == "__main__":
	rospy.init_node('emergency_stop')
	#create a function that moves the robot forward
	r=rospy.Rate(5)

	bump_subscriber = rospy.Subscriber('bump',"neato_node/Bump", bump_event_callback)
	pub = rospy.Publisher('cmd_vel', Twist)

	while not rospy.is_shutdown(): 
    	mover(pub)

