#!/usr/bin/env python

'''
Outputs /sonar_image and /sonar_image_labelled for visualisation
'''

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 

import sys
import cv2
import numpy as np

bridge = CvBridge()
sonarImg = None
labelledImg = None 

def rosimg2cv(ros_img):
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(ros_img, desired_encoding="mono8")
    except CvBridgeError as e:
        rospy.logerr(e)
        
    return frame

def sonar_callback(rosImg):
    global sonarImg 
    rawImg = rosimg2cv(rosImg)
    rawImg = cv2.cvtColor(rawImg, cv2.COLOR_GRAY2BGR)
    sonarImg = cv2.resize(rawImg, (320, 250))
    #print sonarImg.shape
    
def sonar_labelled_callback(rosImg):
    global labelledImg
    rawImg = rosimg2cv(rosImg)
    rawImg = cv2.cvtColor(rawImg, cv2.COLOR_GRAY2BGR)
    labelledImg = cv2.resize(rawImg, (320, 250))
    #print labelledImg.shape
    #rawImg = cv2.cvtColor(rawImg, cv2.COLOR_GRAY2BGR)
    createOutImg()
    
def createOutImg():
    global sonarImg, labelledImg
    try:
        outImg = np.hstack((sonarImg, labelledImg))
        cv2.imshow("Sonar", outImg)
    except Exception, e:
        pass
    cv2.waitKey(5)

def main():
    cv2.namedWindow("Sonar")
    
    rospy.Subscriber("/sonar_image", Image, sonar_callback)
    rospy.Subscriber("/sonar_image_labelled", Image, sonar_labelled_callback)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.waitKey(1)
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("sonar_gui")
    main()