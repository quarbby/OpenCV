#!/usr/bin/env/python 

'''
To test out the torpedo side states
'''
#testing
import math
import numpy as np
import cv2
from vision import TorpedoVision

from utils.frontComms import FrontComms

def main():    
    global loThres
    global hiThres
    global img
    global outImg
    
    frontComms = FrontComms("torpedo.vision")
    
    img = cv2.imread("torpedo/torpedo_side.png")
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    img = cv2.resize(img, (640, 480))
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img = cv2.GaussianBlur(img, ksize=(3, 3), sigmaX = 0)
    
    screenCtr = (640/2, 480/2)
    
    edged = cv2.Canny(img, 30, 200)
    scratchImgCol = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
    
    lines = cv2.HoughLinesP(edged, 2, math.pi/180.0, 30, 50, 10)
    for line in lines[0]:
        cv2.line(scratchImgCol,
                (line[0], line[1]), (line[2], line[3]),
                (0,255,0), 1)
    
#     circles = cv2.HoughCircles(edged, cv2.cv.CV_HOUGH_GRADIENT, 1,
#                                 minDist=5, param1=300, param2=30,
#                                 minRadius = 0, maxRadius = 100)   
#     
#     if circles is not None:
#         for circle in circles[0,:,:]:
#             circleCentroid = (circle[0], circle[1])
#             cv2.circle(scratchImgCol, circleCentroid, circle[2], (255, 255, 0), 2)
#             cv2.circle(scratchImgCol, circleCentroid, 2, (255, 0, 255), 3)    
    
    cv2.namedWindow("Torpedo")
    cv2.imshow("Torpedo", scratchImgCol)
    
    cv2.waitKey()
        
        
