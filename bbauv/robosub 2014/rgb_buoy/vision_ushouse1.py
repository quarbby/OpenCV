#/usr/bin/env/python

''' With white balance and others '''

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
                'lo1': (108, 0, .0), 'hi1': (184, 255, 255),
                 'lo2': (0, 0, 0), 'hi2': (23, 255, 255),
                 # 'lo3': (0,34,0), 'hi3':(22,255,255),           # US House 1 morning
                'lo3': (0, 0, 113), 'hi3': (8, 255, 255),       # Queenstown values
                 'lo4': (150, 30, 0), 'hi4': (180, 255, 255), # Bottom dark colours (US)
                 'dilate': (9, 9), 'erode': (5,5), 'open': (5,5)}
# 
    greenParams = {'lo': (24, 30, 50), 'hi': (111, 255, 255),
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}

    blueParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (11,11), 'erode': (5,5), 'open': (3,3)}
    curCol = None

    # Hough circle parameters
    circleParams = {'minRadius':10, 'maxRadius': 0 }
    houghParams = (74, 12)
    allCentroidList = []
    allAreaList = []
    allRadiusList = []

    minContourArea = 1000
    
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
        
        img = self.whiteBal(img)
        # img = self.enhance(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = np.array(hsvImg, dtype=np.uint8)

        # Blur image
        # gauss = cv2.GaussianBlur(hsvImg, ksize=(5,5), sigmaX=9)
        # sum = cv2.addWeighted(hsvImg, 1.5, gauss, -0.6, 0)
        # enhancedImg = cv2.medianBlur(sum, 3)
                
        # Find red image 
        # redImg = self.threshold(enhancedImg, "RED")
        redImg = self.threshold(hsvImg, "RED")
        outImg = redImg

        return outImg

    def threshold(self, img, color):
        self.allCentroidList = []
        self.allAreaList = []
        self.allRadiusList = []        
        
        #params = self.getParams(color)
        
        # Perform thresholding
        mask = cv2.inRange(img, self.redParams['lo3'], self.redParams['hi3'])
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

        # return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        thresImg1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
        # thresImg1 = cv2.dilate(mask, kern, iterations=1)
        
        mask2 = cv2.inRange(img, self.redParams['lo4'], self.redParams['hi4'])
        thresImg2 = cv2.dilate(mask2, kern, iterations=2)

        # binImg = cv2.bitwise_or(thresImg1, thresImg2)

        binImg = thresImg1

        # return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
        
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
                               minDist=30, param1=self.houghParams[0], 
                               param2=self.houghParams[1],
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

                self.comms.grad = self.getGradient()
            else:
                self.comms.centroidToBump = self.previousCentroid
                self.comms.rectArea = self.previousArea

        cv2.putText(scratchImgCol, "Ang: " + str(self.comms.grad), (30, 100),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204,204,204))

        # Draw new centroid
        cv2.circle(scratchImgCol, self.comms.centroidToBump, 3, (0, 255, 255), 2)
        # rospy.loginfo("Area: {}".format(self.comms.rectArea)) # To put on the scratchImg
        cv2.putText(scratchImgCol, "Area: " + str(self.comms.rectArea), (30, 80),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))
            
        # How far centroid is off screen center
        self.comms.deltaX = float((self.comms.centroidToBump[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])

        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))
        self.comms.deltaY = float((self.comms.centroidToBump[1] - vision.screen['height']/2)*1.0/
                                  vision.screen['height'])
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))


        # Draw center rect
        scratchImgCol = vision.drawCenterRect(scratchImgCol)

        return scratchImgCol

    def enhance(self, img):
        blurImg = cv2.GaussianBlur(img, ksize=(0,0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 1.5, blurImg, -0.6, 0)
        return enhancedImg

    def getGradient(self):
        angle = self.radToDeg(math.atan2(self.comms.centroidToBump[1]-vision.screen['height'],
                self.comms.centroidToBump[0]-vision.screen['width']))
        return angle*(-1.0) 

    def radToDeg(self, angle):
        return angle*180.0/math.pi

    def whiteBal(self, img):
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        channels[1] = cv2.equalizeHist(channels[1])
        img = cv2.merge(channels, img)
        img = cv2.bilateralFilter(img, -1, 5, 0.1)
        
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)

    def normalise(self, img):
        channel = cv2.split(img)
        # for i in channel[1]:
        #     i += 10
        for i in channel[2]:
            i += 10
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
        self.circleParams['minRadius'] = self.comms.params['minRadius']
    
def main():
    cv2.namedWindow("RGB")
    
    inImg = cv2.imread("rgb_buoy/rgb1.png")
    from comms import Comms
    detector = RgbBuoyVision(comms = Comms())
    outImg = detector.gotFrame(inImg)

    if outImg is not None: cv2.imshow("RGB", outImg)
    cv2.waitKey()
