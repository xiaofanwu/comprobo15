#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class DetectPicture(object):
	def __init__ (Self,image_topic):
		rospy.init_node("DetectPicture")
		self.cv_image=None
		self.bridge=CvBridge()
		rospy.Subscriber(image_topic,Image,self.process_image)
		cv2.namedWindow("video_window")
	#process images that come in from ROS
	def process_image(self,msg):
		#convert the image coming in as CV image
		self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('video_window', self.cv_image)
        cv2.waitKey(5)
	def run(self):
   		r=rospy.Rate(5)
   		while not rospy.is_shutdown():
   			r.sleep()
    		
if __name__ == '__main__':
	node = DetectPicture("/camera/image_raw")
	node.run()    		

