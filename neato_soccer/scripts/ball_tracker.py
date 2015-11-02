#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import rospkg
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import match_keypoints1 as mk

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        print "in init"
        rospy.init_node('ball_tracker')
        self.cv_image = None 
        self.first_image=None                       # the latest image from the camera
        self.bridge = CvBridge()     
        rospack = rospkg.RosPack()
               # used to convert ROS messages to OpenCV
        # self.detector = cv2.FeatureDetector_create('SIFT')
        # self.extractor = cv2.DescriptorExtractor_create('SIFT')

        rospy.Subscriber(image_topic, Image, self.process_image)
        # self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        # self.corner_threshold = 0.0

        # self.ratio_threshold = 1.0
        # self.im2_file = rospack.get_path('neato_soccer') + '/scripts/' + im2_file


    def process_image(self, msg):
        print "in process_image"
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        #cature image every 2 seconds
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray=cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        gray=cv2.GaussianBlur(gray,(21,21),0)
        if self.first_image is None:
            self.first_image=gray
        frameChanges=cv2.absdiff(self.first_image,gray)
        thresh=cv2.threshold(frameChanges,25,255,cv2.THRESH_BINARY)[1]
        thresh=cv2.dilate(thresh,None,iterations=2)
        (cnts, _)=cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c)<500:
                continue
        (x,y,w,h)=cv2.boundingRect(c)
        cv2.rectangle(self.cv_image,(x,y),(x+w,y+h),(0,255,0),2)

        cv2.imshow('video_window', self.cv_image)
        # self.matcher = cv2.BFMatcher()

        #get the image, compare it using Paul's code,
        # cv2.imwrite('after8.png',binary_image)
        cv2.waitKey(50)
        # self.compute_matches(self.cv_image)
        #compare the image taken in with already stored image.
 #        #storedImage=[]
 #        #KeyPointMatcherDemo(binary_image,)

 #    def compute_matches(self,im1_file):
 #        """ reads in two image files and computes possible matches between them using SIFT """
 #        im1 = im1_file
 #        im2 = cv2.imread(self.im2_file)
 
 #        im1_bw = cv2.cvtColor(im1,cv2.COLOR_BGR2GRAY)
 #        im2_bw = cv2.cvtColor(im2,cv2.COLOR_BGR2GRAY)

 #        kp1 = self.detector.detect(im1_bw)
 #        kp2 = self.detector.detect(im2_bw)

 #        dc, des1 = self.extractor.compute(im1_bw,kp1)
 #        dc, des2 = self.extractor.compute(im2_bw,kp2)

 #        matches = self.matcher.knnMatch(des1,des2,k=2)
 # #find the good matches 
 #        good_matches = []
 #        for m,n in matches:
 #            # make sure the distance to the closest match is sufficiently better than the second closest
 #            if (m.distance < self.ratio_threshold*n.distance and
 #                kp1[m.queryIdx].response > self.corner_threshold and
 #                kp2[m.trainIdx].response > self.corner_threshold):
 #                good_matches.append((m.queryIdx, m.trainIdx))
 #        print len(good_matches)>40

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
        	# start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()