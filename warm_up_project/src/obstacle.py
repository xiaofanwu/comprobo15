#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time 

class obstacle:
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """

    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('obstacle')
        
        self.sub=rospy.Subscriber("/scan",LaserScan, self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.twist=Twist()
        self.objectdetected=False   




    def processScan(self,msg):
        #check in front of the robot and see if there is anything within the next 3 meters. Append the range data that
        #it checks in the infront list
        for i in range(-25,25):
            if 0<msg.ranges[i]<0.5:
                self.objectdetected=True
                return 

        self.objectdetected=False   
 





    def run(self):
        """ Our main 5Hz run loop """
        while not rospy.is_shutdown():
            timecount=0
            if self.objectdetected:
                print "I should be turning"
            # start to turn the robot at 0.1 speed until there is nothing in its field of vision within the next 3 meter.
                self.twist.angular.z=0.1
                self.twist.linear.x=0
                self.pub.publish(self.twist)
                print "turning speed" + str(self.twist.angular.z)
                #check if there is anything within the the three meter, if there is, keep turning until nothing is there. 

                while self.objectdetected:
                    timecount=timecount+1
                    time.sleep(0.1)
                #when nothing is there, let the robot go straight for 0.5 meter and then turn back. 
                self.twist.angular.z=0
                self.twist.linear.x=0.1
                self.pub.publish(self.twist)

                time.sleep(3)

                #then stop the robot and turn backwards of what it have turned
                self.twist.linear.x=0
                self.twist.angular.z=-0.1
                #PUBLISH BEFORE YOU SLEEP!!!!
                self.pub.publish(self.twist)

                time.sleep(0.1*timecount)

            else:
                self.twist.linear.x=0.1
                self.twist.angular.z=0
                self.pub.publish(self.twist)


             

        

if __name__ == '__main__':
    node = obstacle()
    node.run()