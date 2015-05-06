#/usr/bin/env/python

''' Backup previous code '''

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class RgbBuoyVision:
    screen = {'width': 640, 'height': 480}

    # Vision parameters
    
    redParams = {
                'lo1': (108, 0, 0), 'hi1': (184, 255, 255),
                 'lo2': (0, 0, 0), 'hi2': (23, 255, 255),
#                 'lo1': (108, 0, 0), 'hi1': (180, 255, 160),
#                  'lo1': (115, 0, 0), 'hi1': (168, 255, 255),
                 'dilate': (15,15), 'erode': (5,5), 'open': (5,5)}


    greenParams = {'lo': (24, 30, 50), 'hi': (111, 255, 255),
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    blueParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (11,11), 'erode': (5,5), 'open': (3,3)}
    curCol = None

    # Hough circle parameters
    circleParams = {'minRadius':20, 'maxRadius': 0 }
    houghParams = {'param1': 80, 'param2': 15}
    allCentroidList = []
    allAreaList = []
    allRadiusList = []

    minContourArea = 300
    
    # Keep track of the previous centroids for matching 
    previousCentroid = (-1, -1)
    previousArea = 0

    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms

    def gotFrame(self, img):
        # Set up parameters
        outImg = None

        # Preprocessing
        img = cv2.resize(img, (640, 480))
        rawImg = vision.shadesOfGray(img)
        
        blurImg = cv2.GaussianBlur(rawImg, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(rawImg, 2.5, blurImg, -1.5, 0)
    
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = self.normalise(hsvImg)
        # return cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    
        # Find red image 
        redImg = self.threshold(hsvImg, "RED")
        outImg = redImg
        
        # Find green image
        #greenLen, greenImg = self.threshold(img, "GREEN")
        
        # Find blue image
        #blueLen, blueImg = self.threshold(img, "BLUE")
        
        #outImg = redImg | greenImg | blueImg 

        return outImg

    def threshold(self, img, color):
        self.allCentroidList = []
        self.allAreaList = []
        self.allRadiusList = []        
        
        #params = self.getParams(color)
        
        # Perform thresholding
        binImg1 = cv2.inRange(img, self.redParams['lo1'], self.redParams['hi1'])
        binImg2 = cv2.inRange(img, self.redParams['lo2'], self.redParams['hi2'])
        binImg = cv2.bitwise_or(binImg1, binImg2)
#         binImg = binImg1
#         return binImg
          #binImg = self.erodeAndDilateImg(binImg, params)
        #binImg = vision.erodeAndDilateImg(binImg1, self.redParams)
        erodeEl = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.redParams['erode'])
        dilateEl = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.redParams['dilate'])
        openEl = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.redParams['open'])

        binImg = cv2.erode(binImg, erodeEl)
        binImg = cv2.dilate(binImg, dilateEl)
        binImg = cv2.morphologyEx(binImg, cv2.MORPH_OPEN, openEl)
        
        # Find contours
        scratchImg = binImg.copy()
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
        contours, hierachy = cv2.findContours(scratchImg, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour
        
        # If centering, just find the center of largest contour
        if self.comms.isCentering:
            if len(contours) > 0:
                largestContour = contours[0]
                mu = cv2.moments(largestContour)
                muArea = mu['m00']
                self.comms.centroidToBump = (int(mu['m10']/muArea), int(mu['m01']/muArea))
                self.comms.rectArea = muArea
                
                self.previousCentroid = self.comms.centroidToBump
                self.previousArea = self.comms.rectArea
                
            else:
                self.comms.centroidToBump = self.previousCentroid
                self.comms.rectArea = self.previousArea
        else:
            # Find hough circles
            circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                               minDist=30, param1=76, 
                               param2=13,
                               minRadius = self.circleParams['minRadius'],
                               maxRadius = self.circleParams['maxRadius'])
            
            # Check if center of circles inside contours
            if contours is not None:
                for contour in contours:
                    mu = cv2.moments(contour)
                    muArea = mu['m00']
                    centroid = (mu['m10']/muArea, mu['m01']/muArea)
                    if circles is not None:
                        for circle in circles[0,:,:]:
                            circleCentroid = (circle[0], circle[1])
                            if abs((Utils.distBetweenPoints(centroid,circleCentroid))) < circle[2]:
                                self.comms.foundBuoy = True
                                # Find new centroid by averaging the centroid and circle centroid
                                newCentroid =(int(centroid[0]+circleCentroid[0])/2,
                                              int(centroid[1]+circleCentroid[1])/2)
                                self.allCentroidList.append(newCentroid)
                                self.allAreaList.append(cv2.contourArea(contour))
                                self.allRadiusList.append(circle[2])
                                # Draw circles
                                cv2.circle(scratchImgCol, newCentroid, circle[2], (255, 255, 0), 2)
                                cv2.circle(scratchImgCol, newCentroid, 2, (255, 0, 255), 3)        
            
            # Find the circle with the largest radius
            if not len(self.allCentroidList) == 0:
                maxIndex = self.allRadiusList.index(max(self.allRadiusList))
                self.comms.centroidToBump = self.allCentroidList[maxIndex]
                self.comms.rectArea = self.allAreaList[maxIndex]
                
                self.previousCentroid = self.comms.centroidToBump
                self.previousArea = self.comms.rectArea
            else:
                self.comms.centroidToBump = self.previousCentroid
                self.comms.rectArea = self.previousArea

        # Draw new centroid
        cv2.circle(scratchImgCol, self.comms.centroidToBump, 3, (0, 255, 255), 2)
        # rospy.loginfo("Area: {}".format(self.comms.rectArea)) # To put on the scratchImg
        cv2.putText(scratchImgCol, "Area: " + str(self.comms.rectArea), (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
            
        # How far centroid is off screen center
        self.comms.deltaX = float((self.comms.centroidToBump[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])

        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        self.comms.deltaY = float((self.comms.centroidToBump[1] - vision.screen['height']/2)*1.0/
                                  vision.screen['height'])
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))


        # Draw center rect
        scratchImgCol = vision.drawCenterRect(scratchImgCol)

        return scratchImgCol

    def normalise(self, img):
        channel = cv2.split(img)
#         for i in channel[1]:
#             i += 10
#         for i in channel[2]:
#             i -= 3
        cv2.normalize(channel[1], channel[1], 0, 255, cv2.NORM_MINMAX)
        cv2.normalize(channel[2], channel[2], 0, 255, cv2.NORM_MINMAX)
        return cv2.merge(channel, img)

    def toBumpCol(self, redLen, blueLen, greenLen):
        if not redLen == 0:
            self.curCol = 0
        elif not greenLen == 0:
            self.curCol = 1
        elif not blueLen == 0:
            self.curCol = 2
            
        if self.curCol == self.comms.colourToBump:
            self.finishBump = True
            return 
        
        bumpMatrix = [[0,2,1], [1,0,2], [2,1,0]]
        self.comms.timesToBump = bumpMatrix[curCol][self.comms.colourToBump]

    def getParams(self, inColour):
        colours = ["RED", "GREEN", "BLUE"]

        if inColour == colours[0]:
            return self.redParams
        elif inColour == colours[1]:
            return self.greenParams
        else:
            return self.blueParams

    def updateParams(self):
        self.redParams['lo1'] = self.comms.params['hsvLoThres']
        self.redParams['hi1'] = self.comms.params['hsvHiThres']
        self.houghParams = self.comms.params['HoughParams']
        self.minContourArea = self.comms.params['minContourArea']
    
def main():
    cv2.namedWindow("RGB")
    
    inImg = cv2.imread("rgb_buoy/rgb1.png")
    from comms import Comms
    detector = RgbBuoyVision(comms = Comms())
    outImg = detector.gotFrame(inImg)

    if outImg is not None: cv2.imshow("RGB", outImg)
    cv2.waitKey()