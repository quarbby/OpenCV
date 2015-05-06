#/usr/bin/env/python 

'''
Torpedo Vision
'''

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class TorpedoVision:    
    # Vision parameters
    
    thresParams = {
                    # 'lo': (0, 0, 112), 'hi': (195, 255, 255), # For day 1 robosub
                    # 'lo': (0, 161, 0), 'hi': (128, 255, 255),   # Comp ground 9am
                    'lo': (0, 161, 0), 'hi': (128, 255, 230), # Torpedo hole 9am
                    # 'lo': (0, 0, 0), 'hi': (170, 255, 255),   # US Tech tank
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)}

    greenParams = {
                 # 'lo': (98, 0, 143), 'hi': (230, 103, 255), # Lab for night
                    'lo': (95, 0, 0), 'hi': (230, 103, 116),   # RoboSub Day 1: Sunny
                    # 'lo': (40, 211, 0), 'hi': (143, 255, 255),  # Just HSV Threshold
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)
                  }
    minContourArea = 200
    
    # For circles 
    previousCentroid = (-1, -1)
    previousRadius = 0

    # For board 
    previousBoardCentroid = (-1, -1)

    sonarOffset = 10.0

    aimingCentroid = (-1, -1)
    skewLimit = 0.01
    sonarThres = 150
        
    def __init__(self, comms = None):
        self.comms = comms

    def gotFrame (self, img): 
        allCentroidList = []
        
        img = cv2.resize(img, (640, 480))

        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        loThres = np.array(self.thresParams['lo'], np.uint8)
        hiThres = np.array(self.thresParams['hi'],np.uint8)
        binImg = cv2.inRange(hsvImg, loThres, hiThres)

        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        binImg = cv2.morphologyEx(binImg, cv2.MORPH_OPEN, kern)
        dilateKern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        binImg = cv2.dilate(binImg, kern, iterations=1)
        # return cv2.cvtColor(circleBinImg, cv2.COLOR_GRAY2BGR)

        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        #Get the largest contour and mask it
        cont, hie = cv2.findContours(binImg.copy(), 
            cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        cont = filter(lambda c: cv2.contourArea(c) > self.minContourArea, cont)
        if len(cont) > 0:
            self.comms.foundSomething = True
        cont = sorted(cont, key=cv2.contourArea, reverse=True)

        if len(cont) == 0:
            cv2.putText(scratchImgCol, "Nothing found", (410, 80),
                        cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))    
            return scratchImgCol          

        largestContour = cont[0]

        rect = cv2.minAreaRect(largestContour)
        box = cv2.cv.BoxPoints(rect)
        boxInt = np.int0(box)
        cv2.drawContours(scratchImgCol, [boxInt], 0, (0,0,255), 2)

        mask = np.zeros_like(binImg, dtype=np.uint8)
        cv2.fillPoly(mask, [np.int32(largestContour)], 255)
        binImg2 = binImg.copy()
        binImg2 = cv2.bitwise_not(binImg2, mask=mask)
        # return cv2.cvtColor(binImg2, cv2.COLOR_GRAY2BGR)

        circleCont, circleHie = cv2.findContours(binImg2.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        circleCont = filter(lambda c: cv2.contourArea(c) > 2500, circleCont)
        circleCont = sorted(circleCont, key=cv2.contourArea, reverse=True)
        
        scratchImgCol = cv2.cvtColor(binImg2, cv2.COLOR_GRAY2BGR)
        if len(circleCont) == 0:
            cv2.putText(scratchImgCol, "No circles found ", (410, 80),
                        cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))    
            return scratchImgCol  

        #Use Hough circles
        circles = cv2.HoughCircles(binImg2, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                    minDist=20, param1=74, param2=13,
                                    minRadius=25,
                                    maxRadius=100)

        # if circles is None:
        #     cv2.putText(scratchImgCol, "No circles found ", (410, 80),
        #                 cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))    
        #     return scratchImgCol  

        # for i in circles[0,:,:]:
        #     # draw the outer circle
        #     centroid = (int(i[0]), int(i[1]))
        #     radius = int(i[2])
        #     cv2.circle(scratchImgCol, centroid, radius, (0,255,0), 2)
        #     # draw the center of the circle
        #     cv2.circle(scratchImgCol,centroid, 2, (0,0,255), 3)
        #     allCentroidList.append((centroid[0], centroid[1], radius))

        for cont in circleCont:
            (cx, cy), radius = cv2.minEnclosingCircle(cont)
            if 25 < radius < 90:
                cv2.circle(scratchImgCol, (int(cx), int(cy)), int(radius), (255,255,0), 2)
                cv2.circle(scratchImgCol, (int(cx), int(cy)), 1, (255, 0, 255), 2)
                allCentroidList.append((cx, cy, radius))

        if len(allCentroidList) > 0:
            self.comms.foundCircles = True 

        allCentroidList = sorted(allCentroidList, key=lambda centroid:centroid[2], reverse=True)
        if len(allCentroidList) > 0:
            self.comms.centroidToShoot = (int(allCentroidList[0][0]), int(allCentroidList[0][1]))
            self.comms.radius = allCentroidList[0][2]

        cv2.circle(scratchImgCol, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
        cv2.circle(scratchImgCol, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
            
        # How far the centroid is off the screen center
        self.comms.deltaX = float((self.comms.centroidToShoot[0] - self.aimingCentroid[0])*1.0/
                                    vision.screen['width'])
        self.comms.deltaY = float((self.comms.centroidToShoot[1] - self.aimingCentroid[1])*1.0/
                              vision.screen['height'])

        '''
        Draw Everything
        '''
        if self.comms.centroidToShoot[0]== -1 and self.comms.centroidToShoot[1]== -1:
            self.comms.centroidToShoot = self.getAimingCentroid()
        self.comms.angleFromCenter = self.calculateAngle(self.comms.centroidToShoot, self.aimingCentroid)
        cv2.line(scratchImgCol, (int(self.comms.centroidToShoot[0]),int(self.comms.centroidToShoot[1])), 
            (int(self.aimingCentroid[0]), int(self.aimingCentroid[1])), (0,255,0), 2)
        centerx = int((self.comms.centroidToShoot[0]+self.aimingCentroid[0])/2)
        centery = int((self.comms.centroidToShoot[1]+self.aimingCentroid[1])/2)
        cv2.putText(scratchImgCol, "{0:.2f}".format(self.comms.angleFromCenter), (centerx+20, centery-20),
            cv2.FONT_HERSHEY_PLAIN, 1, (255,150,50), 1)

        cv2.drawContours(scratchImgCol, [boxInt], 0, (0,0,255), 2)

        # Circle variables
        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,153,51))
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,153,51))
        
        cv2.putText(scratchImgCol, "Rad " + str(self.comms.radius), (30,85),
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,153,51))    


        # Draw stuff in case return things
        # Draw center of screen
        scratchImgCol = self.drawCenterRect(scratchImgCol)     
        # scratchImgCol = vision.drawCenterRect(scratchImgCol) 
        cv2.putText(scratchImgCol, "TORPEDO", (20, 430),
            cv2.FONT_HERSHEY_PLAIN, 2, (255,153,51))
        cv2.putText(scratchImgCol, str(self.comms.state), (20,460),  
            cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148)) 

        
        # return np.hstack((scratchImgCol, circleBinImg))
        return scratchImgCol

    def whiteBal(self, img):
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        channels[1] = cv2.equalizeHist(channels[1])
        img = cv2.merge(channels, img)
        img = cv2.bilateralFilter(img, -1, 5, 0.1)
        
        # Morphological operations
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern, iterations=1)
    
    def updateParams(self):
        # self.greenParams['lo'] = (self.comms.params['loH'], 
        #                             self.comms.params['loS'],
        #                             self.comms.params['loV'])
        # self.greenParams['hi'] = (self.comms.params['hiH'],
        #                             self.comms.params['hiS'],
        #                             self.comms.params['hiV'])
        self.thresParams['lo'] = (self.comms.params['loH_b'], 
                                    self.comms.params['loS_b'],
                                    self.comms.params['loV_b'])
        self.thresParams['hi'] = (self.comms.params['hiH_b'],
                                    self.comms.params['hiS_b'],
                                    self.comms.params['hiV_b'])
        self.minContourArea = self.comms.params['minContourArea'] 
        self.skewLimit = self.comms.params['skewLimit']
        self.sonarThres = self.comms.params['sonarThres']

    def normaliseImg(self, img):
        channel = cv2.split(img)
        for i in channel[1]:
            i += 5
        # cv2.normalize(channel[1], channel[1], 0, 255, cv2.NORM_MINMAX)
        cv2.normalize(channel[2], channel[2], 0, 255, cv2.NORM_MINMAX)
        return cv2.merge(channel, img)  
    
    def illuminanceMask(self, img):
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayImg = cv2.equalizeHist(grayImg)
        return cv2.threshold(grayImg, 200, 255, cv2.THRESH_BINARY)[1]

    def isInRect(self, centroid, rect, size):
        if centroid[0] > (rect[0] - size[0]/2) and \
            centroid[0] < (rect[0] + size[0]/2) and \
            centroid[1] > (rect[1] - size[1]/2) and \
            centroid[1] < (rect[1] + size[1]/2):
            return True
        else:
            return False 

    def morphology(self, img):
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)

    def illuminanceRemoval(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        img = cv2.merge(channels, img)
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR) 
        return img
     
    def illumMask(self, img):
        illumMask = self.illuminanceMask(img)
        illumMask = cv2.bitwise_not(illumMask)
        img = cv2.bitwise_and(cv2.cvtColor(illumMask, cv2.COLOR_GRAY2BGR), img)
        return img

    def toHSV(self, img):
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = np.array(hsvImg, dtype=np.uint8)  
        return hsvImg

    def enhance(self, hsvImg):
        gauss = cv2.GaussianBlur(hsvImg, ksize=(5,5), sigmaX=9)
        sum = cv2.addWeighted(hsvImg, 1.5, gauss, -0.6, 0)
        enhancedImg = cv2.medianBlur(sum, 3)    
        enhancedImg = cv2.GaussianBlur(enhancedImg, ksize=(5,5), sigmaX=2)
        return enhancedImg

    def camCallback(self, rosImg):
        outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish and outImg is not None:
            try:
                self.outPub.publish(Utils.cv2rosimg(outImg))
            except Exception, e:
                pass
        rospy.sleep(rospy.Duration(0.05))

    def drawCenterRect(self, img):
        midX = vision.screen['width']/2.0 - 50.0
        midY = vision.screen['height']/2.0 - 60.0
        maxDeltaX = vision.screen['width']*0.07
        maxDeltaY = vision.screen['height']*0.07

        self.aimingCentroidOne = (midX-8.0, midY+10.0)
        self.aimingCentroidTwo = (midX-8.0, midY+25.0)

        if self.comms.numShoot == 0:
            self.aimingCentroid = self.aimingCentroidOne
            colour = (0, 255, 255)
        else:
            self.aimingCentroid = self.aimingCentroidTwo
            colour = (0, 0, 255)
        cv2.rectangle(img,
                      (int(self.aimingCentroid[0]-maxDeltaX), int(self.aimingCentroid[1]-maxDeltaY)),
                      (int(self.aimingCentroid[0]+maxDeltaX), int(self.aimingCentroid[1]+maxDeltaY)),
                      colour, 2)  
        return img 

    def getAimingCentroid(self):
        midX = int(vision.screen['width']/2.0 - 50.0)
        midY = int(vision.screen['height']/2.0 - 50.0)
        return (midX, midY)

    def calculateAngle(self, pt1, pt2):
        # grad = (pt1[1]-pt2[1])*1.0 / (pt1[0]-pt2[0])
        angle = self.radToDeg(math.atan2((pt2[1]-pt1[1]), (pt2[0]-pt1[0])))
        return angle*(-1.0)

    def radToDeg(self, angle):
        return angle*180/math.pi

    def whiteBal(self, img):
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        channels[1] = cv2.equalizeHist(channels[1])
        img = cv2.merge(channels, img)
        img = cv2.bilateralFilter(img, -1, 5, 0.1)
        
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)

def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo1.png")
    from comms import Comms
    detector = TorpedoVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()
