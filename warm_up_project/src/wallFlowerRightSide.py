#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
twist=Twist()

class wallFollowing():
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """

    def __init__(self):
        print "at init"
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('test3')
        #self.target_distance = rospy.get_param('~target_distance')
        self.sub=rospy.Subscriber("/scan",LaserScan, self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.twist=Twist()
        self.avgT90=0
        self.avgTR=0
        self.avgb90=0
        self.avgBR=0

        

    def processScan(self,msg):
        self.bottomLeftList=[]
        self.bottomRightList=[]
        self.topLeftList=[]
        self.topRightList=[]
        self.topRight10=[]
        self.bottomRight10=[]
        self.topLeft10=[]
        self.bottomLeft10=[]
        print "in process scan", msg.header.stamp
        i=0

        while i<2:
          
          self.bottomRightList.append(msg.ranges[260-i])
          self.topRightList.append(msg.ranges[280+i])

        #   print "bottomList= " + str(self.bottomRightList)
        #   print "topList= " + str(self.topRightList)
        #   #self.topLeftList.append(msg.ranges[90-i])
        #   #self.bottomLeftList.append(msg.ranges[90+i])
          
          i+=1
          
        #   # for eachnum in self.topLeftList:
        #   #   if eachnum != 0:
        #   #     self.TopLeft10.append(eachnum)
              
          for eachnum in self.topRightList:
            if eachnum != 0:
              self.topRight10.append(eachnum)      
                        
          for eachnum in self.bottomRightList:
            if eachnum != 0:
              self.bottomRight10.append(eachnum)   
        #   # for eachnum in self.bottomLeftList:
        #   #   if eachnum != 0:
        #   #     self.bottomLeft10.append(eachnum) 

        # self.bottomRight10= [.04,.04,.04,.04]
        # self.topRight10= [.05,.05,.05,.05]

        self.avgTR=sum(self.topRight10)/float(len(self.topRight10))
        self.avgBR=sum(self.bottomRight10)/float(len(self.bottomRight10))  
        #print "avTR= " + str(self.avgTR)
        #print "avBR= " + str(self.avgBR)
        # self.avgT90=sum(self.TopLeft10)/float(len(self.TopLeft10))
        # self.avgB90=sum(self.TopRight10)/float(len(self.TopRight10))    


              

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
         # print self.bottomList
          if self.avgTR !=0 and self.avgBR !=0:
            self.twist.angular.z=(self.avgBR-self.avgTR)
        # elif self.avgT90 != 0 and self.avgB90 !=0:
        #     self.twist.angular.z=(self.avgT90-self.avgB90)*0.5
        
        
          self.twist.linear.x=0.1
          #print self.twist
          self.pub.publish(self.twist)
          r.sleep()

if __name__ == '__main__':
    node = wallFollowing()
    node.run()