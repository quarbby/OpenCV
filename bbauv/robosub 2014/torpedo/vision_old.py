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
                    'lo': (0, 200, 0), 'hi': (8, 255, 0), # Jin's parameters
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)}

    greenParams = {
                    'lo': (103, 0, 115), 'hi': (220, 125, 255), # Lab params
                    # 'lo': (43,0,0), 'hi': (75,255,150),    # Enhanced params
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)
                  }
    
    circleParams = {'minRadius': 10, 'maxRadius': 100}
    houghParams = (79, 17)     #Hough circle parameters 
    
    minContourArea = 300
    
    # For circles 
    previousCentroid = (-1, -1)
    previousRadius = 0

    # For board 
    previousBoardCentroid = (-1, -1)
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):        
        allCentroidList = []
        allRadiusList = []
        
        img = cv2.resize(img, (640, 480))
        
        # img = self.illumMask(img)
        
        # img = self.whiteBal(img)

        # img = self.illuminanceRemoval(img)

        # hsvImg = self.toHSV(img)
        
        # enhancedImg = self.enhance(hsvImg)

        # return cv2.cvtColor(enhancedImg, cv2.COLOR_HSV2BGR)

        labImg = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        channels = cv2.split(labImg)
        # return cv2.cvtColor(channels[2], cv2.COLOR_GRAY2BGR)

        # Threshold out something
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
            contours, hierarchy = cv2.findContours(binImgCopy,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(image=binImg, contours=contours, contourIdx=-1, color=(255,255,255), thickness=-1)  
        
        '''
        Detect the board first
        '''

        # Detecting the board 
        # Find the largest contours and make sure its a square
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

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
        box = np.int0(box)
        cv2.drawContours(scratchImgCol, [box], 0, (0,0,255), 2)

        # Find centroid of rect returned 
        mu = cv2.moments(largestContour)
        muArea = mu['m00']
        tempBoardCentroid = (int(mu['m10']/muArea), int(mu['m01']/muArea))
        tempBoardArea = muArea

        # self.comms.skew = mu['m30']/(pow(mu['02'],1.5))

        self.comms.boardCentroid = tempBoardCentroid
        self.comms.boardArea = tempBoardArea

        # Dist where centroid of board is off 
        self.comms.boardDeltaX = float((self.comms.boardCentroid[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])
        self.comms.boardDeltaY = float((self.comms.boardCentroid[1] - vision.screen['height']/2)*1.0/
                                    vision.screen['height'])

        cv2.putText(scratchImgCol, "Board Area: " + str(self.comms.boardArea), (410, 30),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))        
        cv2.circle(scratchImgCol, self.comms.boardCentroid, 2, (0,0,255), 2)
        cv2.putText(scratchImgCol, "Board X: " + str(self.comms.boardDeltaX), (410, 60),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))    
        cv2.putText(scratchImgCol, "Board Y: " + str(self.comms.boardDeltaY), (410, 80),
                    cv2.FONT_HERSHEY_PLAIN, 1, (204, 204, 204))  

        binImgCopy = binImg.copy()
        contours, hierarchy = cv2.findContours(binImgCopy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            self.comms.foundCircles = True

        if not self.comms.isCenteringState:
            contours = filter(lambda c: cv2.contourArea(c) < 10000, contours)

        for contour in contours:
            # if cv2.contourArea(contour) < 10000 and cv2.contourArea(contour) > self.minContourArea:
            if cv2.contourArea(contour) > self.minContourArea:
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                cv2.drawContours(scratchImgCol, [contour], 0,  (255, 0, 255), 2)
                cv2.circle(scratchImgCol, (int(cx), int(cy)), int(radius), (255, 255, 0), 2)
                cv2.circle(scratchImgCol, (int(cx), int(cy)), 1, (255, 0, 255), 2)
                allCentroidList.append((cx, cy, radius))

        if contours is None:
            self.comms.foundCount = self.comms.foundCount + 1
            return scratchImgCol

        sorted(allCentroidList, key=lambda centroid:centroid[2], reverse=True)

        '''
        Detect the circles 
        ''' 

        '''
        # Find Hough circles - used to be edges
        scratchImg = binImg.copy()
        circles = cv2.HoughCircles(scratchImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=10, param1=self.houghParams[0], 
                                   param2=self.houghParams[1],
                                   minRadius = self.circleParams['minRadius'],
                                   maxRadius = self.circleParams['maxRadius'])      
        if circles is None:
            self.comms.foundCount = self.comms.foundCount + 1

            return scratchImgCol
        
        self.comms.foundCircles = True
        circlesSorted = np.array(sorted(circles[0], key=lambda x:x[2], reverse=True))
        
        for circle in circlesSorted:
            circleCentroid = (circle[0], circle[1])

            # Only find circles within the bounding rect
            if self.isInRect(circleCentroid, (center_x, center_y), (width, height)):
                allCentroidList.append(circleCentroid)
                allRadiusList.append(circle[2])
                # Draw Circles
                cv2.circle(scratchImgCol, circleCentroid, circle[2], (255, 255, 0), 2)
                cv2.circle(scratchImgCol, circleCentroid, 2, (255, 0, 255), 3)
        '''

        # Centroid resetted
        if self.comms.centroidToShoot is None:
            '''
            # Pick the largest circle
            if self.comms.numShoot == 0 or len(circles) < 2:
                self.comms.centroidToShoot = (circlesSorted[0][0], circlesSorted[0][1])
                self.comms.radius = circlesSorted[0][2]
            elif self.comms.numShoot == 1:
                self.comms.centroidToShoot = (circlesSorted[1][0], circlesSorted[1][1])
                self.comms.radius = circlesSorted[1][2]
            '''
            if self.comms.numShoot == 0:
                self.comms.centroidToShoot = (int(allCentroidList[0][0]), int(allCentroidList[0][1]))
                self.comms.radius = allCentroidList[0][2]
            elif self.comms.numShoot == 1:
                self.comms.centroidToShoot = (int(allCentroidList[1][0]), int(allCentroidList[1][1]))
                self.comms.radius = allCentroidList[1][2]

        else:
            # Find the centroid closest to the previous 
            if len(allCentroidList) != 0:
                for centroid in allCentroidList:
                    distDiff = []
                    distDiff.append(Utils.distBetweenPoints(
                                        self.previousCentroid, (centroid[0], centroid[1])))
                    minIndex = distDiff.index(min(distDiff))
                    self.comms.centroidToShoot = (int(allCentroidList[minIndex][0]), int(allCentroidList[minIndex][1]))
                    self.comms.radius = allCentroidList[minIndex][2]
                    # self.comms.radius = allRadiusList[minIndex]        
            else:
                # If not then just use the previous one 
                self.comms.centroidToShoot = self.previousCentroid
                self.comms.radius = self.previousRadius
                               
        # Draw centroid to shoot 
        self.previousCentroid = self.comms.centroidToShoot   
        self.previousRadius = self.comms.radius
        cv2.circle(scratchImgCol, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
            
        # How far the centroid is off the screen center
        self.comms.deltaX = float((self.comms.centroidToShoot[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])

        # Draw everything
        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        self.comms.deltaY = float((self.comms.centroidToShoot[1] - vision.screen['height']/2)*1.0/
                                  vision.screen['height'])
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        
        cv2.putText(scratchImgCol, "Rad " + str(self.comms.radius), (30,85),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))           

        cv2.putText(scratchImgCol, "SHOOT TO KILL", (30, 430),
            cv2.FONT_HERSHEY_PLAIN, 2, (255, 200, 255))

        # Draw center of screen
        scratchImgCol = vision.drawCenterRect(scratchImgCol)
                
        return scratchImgCol 

    # Processing sonar image
    def sonarFrame(self, img):
        img = cv2.resize(img, (640, 480))

        allLinesList = []

        # img = cv2.GaussianBlur(img, (5, 5), 0)

        # Gray thresholding
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, im_bw) = cv2.threshold(img, 160, 255, cv2.THRESH_BINARY)
        # return cv2.cvtColor(im_bw, cv2.COLOR_GRAY2BGR)

        sobel = cv2.Sobel(im_bw, cv2.CV_8U, 0, 1, (3,9))

        scratchImgCol = cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)
        cv2.putText(scratchImgCol, "BROCOLLIS", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))


        '''
        contours, hierarchy = cv2.findContours(sobel.copy(), 
            cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0 :
            return scratchImgCol
        rects = []
        for cont in contours:
            rect = cv2.approxPolyDP(cont, 40, True).copy().reshape(-1, 2)
            rects.append(rect)

        cv2.drawContours(sobel, rects, -1, (255,255,255), -1)
        cv2.drawContours(scratchImgCol, rects, -1, (0,255,0), 1)
        '''

        # Find Hough lines 
        lines = cv2.HoughLinesP(sobel, 1, math.pi/2, 50, minLineLength=4, maxLineGap=10)

        if lines is None:
            return scratchImgCol

        # Try merge close by lines 
        lineList = []
        gradList = []
        for line in lines[0]:
            pt1 = (line[0], line[1])
            pt2 = (line[2], line[3])
            lineList.append((pt1, pt2))
            grad = (pt2[1]-pt1[1]) / (pt2[0]-pt1[0])
            gradList.append(grad)
        aveList = []
        for i in range(len(gradList)-1):
            if abs(gradList[i]-gradList[i+1]) <= 3:
                ave1_x = (lineList[i][0][0] + lineList[i+1][0][0])/2
                ave1_y = (lineList[i][0][1] + lineList[i+1][0][1])/2

                ave2_x = (lineList[i][1][0] + lineList[i+1][1][0])/2
                ave2_y = (lineList[i][1][1] + lineList[i+1][1][1])/2

                aveList.append((ave1_x, ave1_y, ave2_x, ave2_y))
                # print aveList

                cv2.line(scratchImgCol, (int(ave1_x), int(ave1_y)), 
                    (int(ave2_x), int(ave2_y)), (0, 255, 0), 2)

        # average_pt1.x = (line1.pt1.x+line2.pt1.x)/2 
        # average_pt2.x = (line1.pt2.x+line2.pt2.x)/2 

        # average_pt1.y = (line1.pt1.y+line2.pt1.y)/2 
        # average_pt2.x = (line1.pt2.y+line2.pt2.y)/2 

        for line in aveList:
            pt1 = (line[0], line[1])
            pt2 = (line[2], line[3])

            angle = math.degrees(math.atan2((pt2[1]-pt1[1]), (pt2[0]-pt1[0])))
            if -30 < angle < 30:
                center_x = int((pt1[0]+pt2[0])/2)
                center_y = int((pt1[1]+pt2[1])/2)
                center = (center_x, center_y)

                # Ratio of center to whole pic 
                dist = (center_y*1.0/vision.screen['height']) * 10.0  # 10m the FOV of sonar

                allLinesList.append((center, angle, dist, pt1, pt2))

                # sorted(allLinesList, key=lambda p:p[3][0])
                # leftSide = allLinesList[0][3]
                # sorted(allLinesList, key=lambda p:p[4][0], reverse=True)
                # rightSide = allLinesList[0][4]

                # cv2.line(scratchImgCol, leftSide, rightSide, (0, 0, 255), 2)

                cv2.line(scratchImgCol, pt1, pt2, (0, 0, 255), 1)
                cv2.circle(scratchImgCol, center, 3, (0, 255, 255), 2)

                angleStr = "{0:.2f}".format(angle)
                cv2.putText(scratchImgCol, "Angle " + angleStr, (center_x+5, center_y-5),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

                distStr = "{0:.2f}".format(dist)
                cv2.putText(scratchImgCol, "Dist " + distStr, (center_x+20, center_y-20),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

        if len(allLinesList) > 0:
            self.comms.sonarDist = max(allLinesList, key=lambda line: line[2])[2]
            # May have to offset because sonar on the left
            self.comms.sonarBearing = self.comms.curHeading + (-1.0)*allLinesList[0][1] - 10

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
        self.houghParams = self.comms.params['houghParams']
        self.minContourArea = self.comms.params['minContourArea']     

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
        rospy.sleep(rospy.Duration(0.5))

def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo1.png")
    from comms import Comms
    detector = TorpedoVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()
