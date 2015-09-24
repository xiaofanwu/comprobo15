import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
twist=Twist()

class warmup2(object):
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """

    def __init__(self, target_distance):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('warmup2')
        #self.target_distance = rospy.get_param('~target_distance')
        self.sub=rospy.Subscriber("/scan",LaserScan, processScan)
        self.pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)

        self.target_distance=target_distance
        self.twist=Twist()
        self.currentDistance=0


    def processScan(self,msg):
        self.dicOfleft={}
        self.dicOfright={}
        for i in (0,46):
            dicOfleft[i]=msg.ranges[i]
        for z in (315,361):
            dicOfright[z]=msg.ranges[z]
            
        for eachdistance in dicOfleft.values():
            if eachdistance != 0:
                eachdistance[]






    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.currentDistance!=0:
                self.twist.linear.x=(self.currentDistance-self.target.distance)*1
                self.pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = WallApproach(1.0)
    node.run()