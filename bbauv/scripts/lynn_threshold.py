#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

import sys

bridge = CvBridge()
loThresh = np.array([0, 0, 0])
hiThresh = np.array([255, 255, 255])


def rosimg2cv(ros_img):
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    return frame


def onChange(data):
    global loThresh, hiThresh
    loThresh[0] = cv2.getTrackbarPos("loH", "output")
    hiThresh[0] = cv2.getTrackbarPos("hiH", "output")
    loThresh[1] = cv2.getTrackbarPos("loS", "output")
    hiThresh[1] = cv2.getTrackbarPos("hiS", "output")
    loThresh[2] = cv2.getTrackbarPos("loV", "output")
    hiThresh[2] = cv2.getTrackbarPos("hiV", "output")




def camCallback(rosImg):
    global loThresh, hiThresh
    rawImg = rosimg2cv(rosImg)
    rawImg = cv2.resize(rawImg, (320, 250))
    
    threshImg = cv2.inRange(rawImg, loThresh, hiThresh)
    threshImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)

    outImg = np.hstack((rawImg, threshImg))
    cv2.imshow("output", outImg)
    cv2.waitKey(5)

def main():
    if len(sys.argv) < 2:
        print "Please specify image topic"
        return

    cv2.namedWindow("output")
    cv2.createTrackbar("loH", "output", 0, 180, onChange)
    cv2.createTrackbar("hiH", "output", 0, 180, onChange)
    cv2.createTrackbar("loS", "output", 0, 255, onChange)
    cv2.createTrackbar("hiS", "output", 0, 255, onChange)
    cv2.createTrackbar("loV", "output", 0, 255, onChange)
    cv2.createTrackbar("hiV", "output", 0, 255, onChange)

    rospy.Subscriber(sys.argv[1], Image, camCallback)

if __name__ == "__main__":
    rospy.init_node("lynn_threshold")
    main()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.waitKey(1)
        r.sleep
