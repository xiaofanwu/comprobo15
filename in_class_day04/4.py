#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wallFollowing:
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """

    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """

        self.twist=Twist()
        #create a target distance to the wall
        self.target_distance=0.5

        self.li270=[]
        self.li90=[]
        self.avg90=0
        self.avg270=0

    def processScan(self,msg):
        for i in range(85,95):
          #Check if object on left side of robot
          if (msg.ranges[i]) != 0:
            self.li90.append(msg.ranges[i])

        for z in range(265,275):
          #Check on right side
          if (msg.ranges[z] != 0):
            self.li270.append(msg.ranges[z])            
          
        if len(self.li90) != 0:
          self.avg90=(sum(self.li90))/(len(self.li90))
        else:
          self.avg90=0

        if len(self.li270) != 0:
          self.avg270=(sum(self.li270))/(len(self.li270))
        else:
          self.avg270=0
              

    def run(self):
        """ Our main 5Hz run loop """
        rospy.init_node('test3')
        r = rospy.Rate(5)
        rospy.Subscriber("/scan",LaserScan, self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        prev_value = 300
        scalar = 1
        while not rospy.is_shutdown():
          print "left: ", self.avg90, "  right: ", self.avg270
          #chekc for the left side
          if (0<self.avg270<1.2):
            # if (self.avg270-self.target_distance)<0.2:
            #   self.twist.angular.z=0
            #   self.twist.linear.x=0.1
                         
            # else:
            #   self.twist.angular.z=(self.avg270-self.target_distance)*0.2
            #   self.twist.linear.x=0.1

            self.twist.angular.z=-(self.avg270-self.target_distance)*0.2
            prev_value = self.avg270
            self.twist.linear.x=0.05

          #check right side
          elif (0<self.avg90<1.2):
            if (self.avg90-self.target_distance)<0.2:
              self.twist.linear.x=0.1
              self.twist.angular.z=0

            else:
              self.twist.linear.x=0.1
              self.twist.angular.z=(self.avg90-self.target_distance)*0.2
          
          else:
            self.twist.angular.z=0
            self.twist.linear.x=0.1

          print "angz: ", self.twist.angular.z
          self.pub.publish(self.twist)
          r.sleep()

if __name__ == '__main__':
    node = wallFollowing()
    node.run()