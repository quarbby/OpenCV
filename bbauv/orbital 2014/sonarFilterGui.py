'''
The filter chain
'''

import cv2
import numpy as np
import math

import rospy
from bbauv_msgs.srv import sonar_pixel

class SonarFilter():
    params = {'lenHi': 1000, 'lenLo': 0,
            'widthHi': 1000, 'widthLo': 0,
            'thres': 160, 'dyMult': 80,
            }
    screen = {'height': 480, 'width': 640}
    imgOffset = 60
    def __init__(self, testing=True):
        rospy.loginfo("Vision filter initialised")
        self.testing = testing
        rospy.loginfo(self.testing)
        if not self.testing:
            self.sonarSrv = rospy.ServiceProxy('/sonar_pixel', sonar_pixel, persistent=True)
            rospy.loginfo("Live")

    def setParams(self, params):
        self.params = params

    def createThresImg(self, sourceImg):
        sourceImg = cv2.resize(sourceImg, (640, 480))
        sourceImg = cv2.cvtColor(sourceImg, cv2.COLOR_BGR2GRAY)

        #binaryImg = cv2.medianBlur(binaryImg, 3)
        binaryImg = cv2.GaussianBlur(sourceImg, (5,5), 0)
        highest = cv2.minMaxLoc(binaryImg)[1]
        threshold = highest - self.params['thresOffset']
        if threshold < self.params['threshBound']:
            threshold = self.params['threshBound']  
                
        self.filterchainImg = cv2.threshold(binaryImg, threshold, 255, cv2.THRESH_BINARY)[1] 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        self.filterchainImg = cv2.dilate(self.filterchainImg, kernel)
        self.filterchainImg = cv2.morphologyEx(self.filterchainImg, cv2.MORPH_CLOSE, kernel, iterations=3)
        self.filterchainImg = cv2.erode(self.filterchainImg, (3,9), iterations=2)

        mask = cv2.threshold(self.filterchainImg, threshold, 255, cv2.THRESH_TOZERO)[1]

        grayImg = sourceImg.copy()
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        allBoxList = []
        for i in contours:
            mask = np.zeros((640,480), dtype=np.uint8)
            cv2.drawContours(mask, [i], 0, 255, -1)

            rect = cv2.minAreaRect(i)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box) 
            
            point1 = np.int32(box[0])
            point2 = np.int32(box[1])
            point3 = np.int32(box[2])
             
            edge1 = point2-point1
            edge2 = point3-point2 

            if(cv2.norm(edge1) > cv2.norm(edge2)):
                dx = cv2.norm(edge1)
                dy = cv2.norm(edge2)
            else:
                dx = cv2.norm(edge2)
                dy = cv2.norm(edge1)

            cv2.drawContours(grayImg, [box], 0, (255,0,0), 1)

            if self.params['lenLo'] < abs(dx) < self.params['lenHi'] \
                and self.params['widthLo'] < abs(dy) < self.params['widthHi'] \
                and self.checkTargetXAxis(box): 
                    cv2.drawContours(grayImg, [box], 0, (0,255,0), 2)  
                    allBoxList.append([box, abs(dx), abs(dy)])
 
        if len(allBoxList) > 0:
            targetObj = allBoxList[0][0]
            firstPoint = targetObj[0] 
            secondPoint = targetObj[3]
            self.midPoint = ((firstPoint[0]+secondPoint[0])/2, (firstPoint[1]+secondPoint[1])/2)
            if not self.testing:
                self.getRangeBearing(grayImg, allBoxList[0][1], allBoxList[0][2])

        return grayImg 

    def checkTargetXAxis(self, box):
        point1 = np.int32(box[0])
        point2 = np.int32(box[1])
        point3 = np.int32(box[2])
             
        edge1 = point2-point1
        edge2 = point3-point2 
        
        if cv2.norm(edge1) > cv2.norm(edge2):
            self.angle = math.degrees(math.atan2(edge1[1], edge1[0]))
        else:
            self.angle = math.degrees(math.atan2(edge2[1], edge2[0]))

        if self.angle < 0.0:
            self.angle = -self.angle

        if  150.0 < self.angle or self.angle < 30.0:
            return True
        else:
            return False


    def getRangeBearing(self, procImg, dx, dy): 
        resp = self.sonarSrv(x=int(self.midPoint[0]), y=int(self.midPoint[1])) 
        self.params['dist'] = resp.range
        self.params['bearing'] = resp.bearing

    def getBearingDist(self):
        return self.params['bearing'], self.params['dist']
