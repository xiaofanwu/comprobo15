#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
twist=Twist()

class wallFollowing():
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """

    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('wallFollowing')
        #self.target_distance = rospy.get_param('~target_distance')
        self.sub=rospy.Subscriber("/scan",LaserScan, self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.twist=Twist()
        self.top=0
        self.bottom=0

    def processScan(self,msg):
        
        for 

        self.top = msg.ranges[60]
        self.bottom=msg.ranges[130]
        print self.top
        print self.bottom

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():

            if self.top!=0 and self.bottom!=0:
                self.twist.angular.z=(self.top-self.bottom)*1
                self.twist.linear.x=0.1
                self.pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = wallFollowing()
    node.run()