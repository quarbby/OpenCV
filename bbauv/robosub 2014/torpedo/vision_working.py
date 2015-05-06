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
                    # 'lo': (0, 200, 0), 'hi': (8, 255, 0), # Jin's parameters
                    'lo': (21, 0, 0), 'hi': (140, 255, 255), 
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)}

    greenParams = {
                    # 'lo': (100, 0, 125), 'hi': (230, 85, 255), # Lab params
                    'lo': (100, 0, 125), 'hi': (230, 103, 255), # Lab params to try
                    # 'lo': (75,0,0), 'hi': (85,255,255),    # Enhanced params
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)
                  }
        
    minContourArea = 300
    
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

        # cv2.circle(img, (vision.screen['width']/2, vision.screen['height']/2),
        #     97, (255, 255, 0), 2)
        # cv2.circle(img, (int(self.aimingCentroid[0]), int(self.aimingCentroid[1])),
        #     97, (255, 255, 0), 2)
        
        # img = self.illumMask(img)
        
        # img = self.whiteBal(img)

        # img = self.illuminanceRemoval(img)

        # blurImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        # enhancedImg = cv2.addWeighted(img, 2.5, blurImg, -1.5, 0)
        # hsvImg = self.toHSV(enhancedImg)
        
        labImg = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        # channels = cv2.split(labImg)
        # return cv2.cvtColor(channels[2], cv2.COLOR_GRAY2BGR)

        binImg = self.morphology(cv2.inRange(labImg, 
            self.greenParams['lo'], self.greenParams['hi']))
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        binImg = cv2.morphologyEx(binImg, cv2.MORPH_OPEN, kern)

        # return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        # binImg = self.morphology(cv2.inRange(enhancedImg,
        #     self.greenParams['lo'], self.greenParams['hi']))
        # erodeKern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        # binImg = cv2.erode(binImg, erodeKern, iterations=2)
        dilateKern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        binImg = cv2.dilate(binImg, dilateKern, iterations=3)

        # Find contours and fill them
        for i in range(4):
            binImgCopy = binImg.copy()
            contours, hierarchy = cv2.findContours(binImgCopy,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image=binImg, contours=contours, contourIdx=-1, color=(255,255,255), thickness=-1)  
        
        '''
        Detect the board first
        '''

        # Detecting the board 
        # Find the largest contours and make sure its a square
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        '''
        Board contour detection
        '''
        binImgCopy = binImg.copy()
        contours, hierarchy = cv2.findContours(binImgCopy, 
            cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea,
            contours)
        if len(contours) == 0 :
            return scratchImgCol

        self.comms.foundSomething = True
        sorted(contours, key=cv2.contourArea, reverse=True)  

        largestContour = contours[0]
        rect = cv2.minAreaRect(largestContour)

        ((center_x,center_y),(width,height),angle) = cv2.minAreaRect(largestContour)
        box = cv2.cv.BoxPoints(rect)
        boxInt = np.int0(box)
        cv2.drawContours(scratchImgCol, [boxInt], 0, (0,0,255), 2)

        '''
        Determine skew
        '''
        boxpoints = np.int32(box)
        leftEdge = cv2.norm(boxpoints[2] - boxpoints[1])
        rightEdge = cv2.norm(boxpoints[3] - boxpoints[0])
        self.comms.skew = rightEdge - leftEdge
        if abs(self.comms.skew) < self.skewLimit:
            self.comms.direction = "NONE"
        if rightEdge > leftEdge:
            self.comms.direction = "RIGHT"
        else:
            self.comms.direction = "LEFT"

        # Find centroid of rect returned 
        if not self.comms.isMovingState:
            mu = cv2.moments(largestContour)
            muArea = mu['m00']
            tempBoardCentroid = (int(mu['m10']/muArea), int(mu['m01']/muArea))
            tempBoardArea = muArea

            self.comms.boardCentroid = tempBoardCentroid
            self.comms.boardArea = tempBoardArea

            # Dist where centroid of board is off 
            self.comms.boardDeltaX = float((self.comms.boardCentroid[0] - self.aimingCentroid[0])*1.0/
                                        vision.screen['width'])
            self.comms.boardDeltaY = float((self.comms.boardCentroid[1] - self.aimingCentroid[1])*1.0/
                                        vision.screen['height'])

        '''
        Detect black circles
        '''

        circleBinImg = self.morphology(cv2.inRange(labImg,
            self.thresParams['lo'], self.thresParams['hi']))
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        circleBinImg = cv2.morphologyEx(circleBinImg, cv2.MORPH_OPEN, kern)
        dilateKern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        circleBinImg = cv2.dilate(circleBinImg, kern)

        circleCont, circleHie = cv2.findContours(circleBinImg.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        circleCont = filter(lambda c: cv2.contourArea(c) > 1000, circleCont)
        sorted(circleCont, key=cv2.contourArea, reverse=True)

        mask = np.zeros_like(binImg, dtype=np.uint8)
        cv2.fillPoly(mask, [np.int32(largestContour)], 255)
        binImg2 = binImg.copy()
        binImg2 = cv2.bitwise_not(binImg2, mask=mask)

        # Find contours of the binImg2
        binCont, binhie = cv2.findContours(binImg2.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        circleBinImg = cv2.cvtColor(binImg2, cv2.COLOR_GRAY2BGR)
        if len(binCont) > 0: 
            for cont in binCont:
                if cv2.contourArea(cont) > 600:
                    (cx, cy), radius = cv2.minEnclosingCircle(cont)

                    if len(circleCont) > 0:
                        for contour in circleCont:
                            if cv2.contourArea(contour) < 20000:
                                (circx, circy), circrad = cv2.minEnclosingCircle(contour)
                                # If circle radius inside contour
                                if cx - radius < circx < cx + radius and cy - radius < circy < cy + radius:
                                    cv2.circle(circleBinImg, (int(circx), int(circy)), int(circrad), (255, 255, 0), 2)
                                    cv2.circle(circleBinImg, (int(circx), int(circy)), 1, (255, 0, 255), 2)
                                    allCentroidList.append((cx, cy, radius))

        if self.comms.isMovingState:
            scratchImgCol = circleBinImg

        if len(allCentroidList) > 0:
            self.comms.foundCircles = True 

        '''
        if self.comms.isCenteringState:
            mask = np.zeros_like(binImg, dtype=np.uint8)
            cv2.fillPoly(mask, [np.int32(largestContour)], 255)
            binImg = cv2.bitwise_not(binImg, mask=mask)

            scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        binImgCopy = binImg.copy()
        contours, hierarchy = cv2.findContours(binImgCopy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            self.comms.foundCircles = True

        if not self.comms.isCenteringState:
            contours = filter(lambda c: cv2.contourArea(c) < 5000, contours)

        for contour in contours:
            # if cv2.contourArea(contour) < 10000 and cv2.contourArea(contour) > self.minContourArea:
            if cv2.contourArea(contour) > self.minContourArea:
                # (cx, cy), radius = cv2.minEnclosingCircle(contour)
                rect = cv2.minAreaRect(contour)
                box = np.int32(cv2.cv.BoxPoints(rect))
                
                edge1 = cv2.norm(box[1] - box[0])
                edge2 = cv2.norm(box[2] - box[1])
                radius = edge1/2.0 if edge1 < edge2 else edge2/2.0
                (cx, cy) = rect[0]

                # cv2.drawContours(scratchImgCol, [box], 0,  (255, 0, 255), 2)
                cv2.circle(scratchImgCol, (int(cx), int(cy)), int(radius), (255, 255, 0), 2)
                cv2.circle(scratchImgCol, (int(cx), int(cy)), 1, (255, 0, 255), 2)
                allCentroidList.append((cx, cy, radius))
        '''

        # average = np.mean([c[0] for c in allCentroidList])
        # self.comms.centerDiff = average - (vision.screen['width']/2)

        sorted(allCentroidList, key=lambda centroid:centroid[2], reverse=True)
        # screenCenter = (vision.screen['width']/2, vision.screen['height']/2)
        # sorted(allCentroidList, key=lambda c:Utils.distBetweenPoints((c[0],c[1]), self.aimingCentroid))

        if len(allCentroidList) > 0:
            self.comms.centroidToShoot = (int(allCentroidList[0][0]), int(allCentroidList[0][1]))
            self.comms.radius = allCentroidList[0][2]

        cv2.circle(scratchImgCol, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
        cv2.circle(circleBinImg, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
            
        # How far the centroid is off the screen center
        self.comms.deltaX = float((self.comms.centroidToShoot[0] - self.aimingCentroid[0])*1.0/
                                    vision.screen['width'])
        self.comms.deltaY = float((self.comms.centroidToShoot[1] - self.aimingCentroid[1])*1.0/
                              vision.screen['height'])
        angleFromCenter = self.calculateAngle(self.comms.centroidToShoot, self.aimingCentroid)
        cv2.line(scratchImgCol, self.comms.centroidToShoot, self.aimingCentroid,
            (0,255,0), 2)
        center = ((self.comms.centroidToShoot[0]+self.aimingCentroid[0])/2 , 
            (self.comms.centroidToShoot[1],self.aimingCentroid[1])/2)
        cv2.putText(scratchImgCol, "("+str("{0:.2f}".format(center[0]))+","+
                str("{0:.2f}".format(center[1])+")" ,(center[0], center[1]-20),
            cv2.FONT_HERSHEY_PLAIN, (0,204,255), 1)

        cv2.putText(scratchImgCol, "SHOOT TO KILL", (20, 430),
            cv2.FONT_HERSHEY_PLAIN, 2, (255,153,51))
        cv2.putText(scratchImgCol, str(self.comms.state), (20,460),  
            cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))

        # Board variables
        cv2.putText(scratchImgCol, "Board Area: " + str(self.comms.boardArea), (410, 30),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))        
        cv2.circle(scratchImgCol, self.comms.boardCentroid, 2, (0,0,255), 2)
        cv2.putText(scratchImgCol, "Board X: " + str(self.comms.boardDeltaX), (410, 60),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))    
        cv2.putText(scratchImgCol, "Board Y: " + str(self.comms.boardDeltaY), (410, 80),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))  

        # Circle variables
        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,153,51))
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,153,51))
        
        cv2.putText(scratchImgCol, "Rad " + str(self.comms.radius), (30,85),
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,153,51))    

        # Skew variables
        cv2.putText(scratchImgCol, "Skew: " + str(self.comms.direction), (430, 430),
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,51,153))
        cv2.putText(scratchImgCol, "Skew angle: " + str(self.comms.skew), (430, 460),
                    cv2.FONT_HERSHEY_PLAIN, 1, (255,51,153))


        # Draw center of screen
        scratchImgCol = self.drawCenterRect(scratchImgCol)     
        # scratchImgCol = vision.drawCenterRect(scratchImgCol)  
        
        # return np.hstack((scratchImgCol, circleBinImg))
        return scratchImgCol

    def gotSonarFrame(self, img):
        img = cv2.resize(img, (vision.screen['width'], vision.screen['height']))

        binImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(binImg, self.sonarThres, 255, cv2.THRESH_BINARY)[1]

        scratchImgCol = img
        cv2.putText(scratchImgCol, "SONAR PROCESSED", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))

        zerosmask = np.zeros((480,640,3), dtype=np.uint8)
        sobel = cv2.Sobel(mask, cv2.CV_8U, 0, 1, (3, 11))
        # return cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)

        contours, hierarchy = cv2.findContours(sobel, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        allLinesList = []
        allBearingList = []

        for i in contours:
            mask = np.zeros((vision.screen['width'], vision.screen['height']), 
                dtype=np.uint8)
            cv2.drawContours(mask, [i], 0, 255, -1)
            lines = cv2.HoughLinesP(mask, 1, math.pi/2, 1, None, 1, 0)

            if lines is not None:
                line = lines[0]
                x1 = [i[0] for i in line]
                y1 = [i[1] for i in line]
                x2 = [i[2] for i in line]
                y2 = [i[3] for i in line]

                pt1List = [(i[0], i[1]) for i in line]
                pt2List = [(i[2],i[3]) for i in line]
                sorted(pt1List, key=lambda x:x[0])
                sorted(pt2List, key=lambda x:x[0])
                pt1 = pt1List[0] if pt1List[0] < pt2List[0] else pt2List[0]
                pt2 = pt1List[-1] if pt1List[-1] > pt2List[-1] else pt2List[-1]

                length = Utils.distBetweenPoints(pt1, pt2)

                # if 45 < length < 100:
                if 25 < length < 80:
                    angle = math.atan2((pt2[1]-pt1[1]), (pt2[0]-pt1[0]))
                    if -30 < angle < 30:

                        allLinesList.append((pt1, pt2))

                        cv2.line(scratchImgCol, pt1, pt2, (0,0,255), 3)
                        angleStr = "{0:.2f}".format(angle)
                        cv2.putText(scratchImgCol, "Ang " + str(angleStr),
                            (int(pt1[0]), int(pt1[1]-5)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1)

                        offset = 60
                        point = (vision.screen['height']-offset)
                        dist = ((point-pt1[1])*1.0/point) * 10.0  # 10m the FOV of sonar
                        distStr = "{0:.2f}".format(dist)
                        cv2.putText(scratchImgCol, "Dist " + str(distStr),
                            (int(pt1[0]), int(pt1[1]-20)), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 1)

                        allBearingList.append((angle, dist))

        if len(allBearingList) > 0:
            sorted(allBearingList, key=lambda p:p[1])
            self.sonarDist = allBearingList[0][1]
            self.sonarBearing = allBearingList[0][0]

        cv2.putText(scratchImgCol, "Sonar Dist " + str(self.sonarDist),
            (30, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

        cv2.putText(scratchImgCol, "Sonar Bearing " + str(self.sonarBearing), (30, 60),
                cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

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
        self.greenParams['lo'] = self.comms.params['loThreshold']
        self.greenParams['hi'] = self.comms.params['hiThreshold']
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
        midY = vision.screen['height']/2.0 - 50.0
        maxDeltaX = vision.screen['width']*0.06
        maxDeltaY = vision.screen['height']*0.06
        self.aimingCentroid = (midX, midY)
        cv2.rectangle(img,
                      (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                      (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                      (0, 255, 255), 2)  
        return img 

    def calculateAngle(self, pt1, pt2):
        grad = (pt1[1]-pt2[1])*1.0 / (pt1[0]-pt2[0])
        angle = self.radToDeg(math.atan2((pt2[1]-pt1[1]), (pt2[0]-pt1[0])))
        return angle*(-1.0)

    def radToDeg(self, angle):
        return angle*180/math.pi

def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo1.png")
    from comms import Comms
    detector = TorpedoVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()
