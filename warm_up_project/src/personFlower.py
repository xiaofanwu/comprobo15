#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class personfollowfinal:

    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('personfollowfinal')
        #self.target_distance = rospy.get_param('~target_distance')
        self.sub=rospy.Subscriber("/scan",LaserScan,self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.twist=Twist()
        self.infront=[]
        self.targetangle=0
        self.targetDistance =.5   #one meter target
        self.currentDistance=0
        #self.currentangle=0



    def processScan(self,msg):
        #reset the list of angles each time in front each time 
        self.infront=[]
        self.currentDistance=msg.ranges[0] 
        #detect if there is anything infront of the robot with in two meter, if there is
        #add the angles the robot detects in a list
        for i in range(-10,10):
            if msg.ranges[i] != 0 and msg.ranges[i]<1:
                self.infront.append(i)
        #calculate the angle that the object is at. The robot will only move if something is 
        #of it.
        if len(self.infront) !=0:

            self.targetangle=sum(self.infront)/len(self.infront)
        else:
            #if nothing is in front
            self.targetangle=0






    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            self.twist.angular.z=self.targetangle*.2
            if self.currentDistance!=0:
              self.twist.linear.x=(self.currentDistance-self.targetDistance)*.5
            self.pub.publish(self.twist)
            
            #find out the current orientation of the robot

          #while 
          #self.twist.angular.z=(self.currentangle-self.targetangle)*1
          #self.pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = personfollowfinal()
    node.run()