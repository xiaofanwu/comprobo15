#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class personfollowfinal:

    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('personfollowfinal')
        #self.target_distance = rospy.get_param('~target_distance')
        self.sub=rospy.Subscriber("/scan",LaserScan,self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.twist=Twist()
        self.targetangle=0
        #self.currentangle=0



    def processScan(self,msg):
        #reset the list of angles each time in front each time 
        infront=[]
        #detect if there is anything infront of the robot with in two meter, if there is
        #add the angles the robot detects in a list
        for i in range(-45,45):
            if (msg.ranges[i] != 0) and (msg.ranges[i]<1):
                infront.append(i)
        #calculate the angle that the object is at. The robot will only move if something is 
        #of it.
        if len(infront) !=0:

            self.targetangle=sum(infront)/len(infront)
        else:
            #if nothing is in front
            self.targetangle=0

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            #compute the target angle we want to get to
            #constantly check if the object target angle is infront of us
            print "the avg angle is at " +str(self.targetangle)
            #find out the current orientation of the robot
            self.twist.angular.z=self.targetangle*.1
            print self.twist.angular.z
            # self.twist.linear.x=.5
            
            self.pub.publish(self.twist)
        	#while 
		    	#self.twist.angular.z=(self.currentangle-self.targetangle)*1
		    	#self.pub.publish(self.twist)
            time.sleep(0.1)

if __name__ == '__main__':
    node = personfollowfinal()
    node.run()