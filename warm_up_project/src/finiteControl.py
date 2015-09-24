#!/usr/bin/env python

""" This is one of my recommended methods for implementing
  finite state control in ROS Python
  Note: this is kind of a hodgepodge of actual Python and
      pseudo code, so there may be small typos """
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FiniteStateController(object):
  """ The intention of this example is to show how you would
    use object-oriented principles to create a finite-state
    controller that wall follows until an person is detected
    in front of the robot, and then attempts to follow that person. """

  # these are constants that let us give a name to each of our states
  # the names don't necessarily have to match up with the names of
  # the methods that are used to implement each behavior.
  WALL_FOLLOW_STATE = "wall_follow"
  PERSON_FOLLOW_STATE = "person_follow"

  def __init__(self):
    rospy.init_node('state_controller')
    self.state = FiniteStateController.WALL_FOLLOW_STATE
    
    # subscribe to relevant sensor topics
    self.sub=rospy.Subscriber('/scan', LaserScan, self.process_scan)
    self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
    self.twist=Twist()
    self.infront=[]
    self.targetangle=0
    self.targetDistance =.5   #one meter target
    self.currentDistance=0
    self.avgT90=0
    self.avgTR=0
    self.avgb90=0
    self.avgBR=0
    self.forward_obstacle_detected = False


  def process_scan(self, msg):
        print "obstacle detected: " + str(self.forward_obstacle_detected)
        self.infront=[]
        self.bottomLeftList=[]
        self.bottomRightList=[]
        self.topLeftList=[]
        self.topRightList=[]
        self.topRight10=[]
        self.bottomRight10=[]
        self.topLeft10=[]
        self.bottomLeft10=[]
        
        
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
        self.forward_obstacle_detected = msg.ranges[0] != 0.0

        i=0

        while i<2:
          self.bottomRightList.append(msg.ranges[260-i])
          self.topRightList.append(msg.ranges[280+i])
          i+=1

          for eachnum in self.topRightList:
            if eachnum != 0:
              self.topRight10.append(eachnum)      
                        
          for eachnum in self.bottomRightList:
            if eachnum != 0:
              self.bottomRight10.append(eachnum)   

        self.avgTR=sum(self.topRight10)/float(len(self.topRight10))
        self.avgBR=sum(self.bottomRight10)/float(len(self.bottomRight10))  
        
        if msg.ranges[0]!=0:
          self.forward_obstacle_detected =True
        else:
          self.forward_obstacle_detected = False
          

  def wall_follow(self):
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
      if self.avgTR !=0 and self.avgBR !=0:
        self.twist.angular.z=(self.avgBR-self.avgTR)
    # elif self.avgT90 != 0 and self.avgB90 !=0:
    #     self.twist.angular.z=(self.avgT90-self.avgB90)*0.5
    
    
      self.twist.linear.x=0.1
      #print self.twist
      self.pub.publish(self.twist)
      r.sleep()
      # handle wall following in here
      # turn proportionally to self.distance_diff (not shown)
      if self.forward_obstacle_detected:
        return FiniteStateController.PERSON_FOLLOW_STATE

  def person_follow(self):
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
      self.twist.angular.z=self.targetangle*.2
      if self.currentDistance!=0:
        self.twist.linear.x=(self.currentDistance-self.targetDistance)*.5
      self.pub.publish(self.twist)
      r.sleep()

      if not self.forward_obstacle_detected:
        return FiniteStateController.WALL_FOLLOW_STATE

  def run(self):
    while not rospy.is_shutdown():
      if self.state == FiniteStateController.WALL_FOLLOW_STATE:
        self.state = self.wall_follow()
      elif self.state == FiniteStateController.PERSON_FOLLOW_STATE:
        self.state = self.person_follow()
      else:
        print "invalid state!!!" # note this shouldn't happen

if __name__ == '__main__':
  node = FiniteStateController()
  node.run()