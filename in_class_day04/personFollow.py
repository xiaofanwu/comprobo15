import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollow(object):
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """
    def __init__ (self,distance):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('PersonFollow')
        #self.target_distance = rospy.get_param('~target_distance')
        self.sub=rospy.Subscriber("/scan",LaserScan, self.processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.desire_distance=distance
        self.twist=Twist()
        #self.currentDistance=0


    def processScan(self,msg):
        self.listofleft=[]
        self.listofright=[]

        for i in range(0,46):
            if msg.ranges[i] != 0:
                self.listofleft.append(msg.ranges[i])
        for z in range(315,361):
            if msg.ranges[i]!=0:
                self.listofright.append(msg.ranges[z])

        self.avgofright=(sum(self.listofright)/len(self.listofright))
        self.avgofleft=(sum(self.listofleft)/len(self.listofleft))






    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.angular.z=(self.avgofleft-self.avgofright)*0.5
            self.pub.publish(self.twist)
            r.sleep()
       #  	if self.currentDistance!=0:
		    	# self.twist.linear.x=(self.currentDistance-self.target.distance)*1
		    	# self.pub.publish(self.twist)
       #      r.sleep()

if __name__ == '__main__':
    node = PersonFollow(1)
    node.run()