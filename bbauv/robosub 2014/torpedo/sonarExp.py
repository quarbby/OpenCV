#!/usr/bin/env python

'''
Sonar Filter
'''

import math
import numpy as np
import cv2
import sys as sys
import roslib
import rospy
import signal
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from bbauv_msgs.srv import sonar_pixel
from utils.utils import Utils
#from dynamic_reconfigure.server import Server as DynServer


class SonarFilter():
    requiredFrames = 5
    bridge = CvBridge()
    targetXAxisBound = 30
    threshold = 200
    threshBound = 100
    lenLowerBound = 20
    lenUpperBound = 130
    widthLowerBound = 5
    midPoint = (-1, -1)  
    pixelRange = -1
    pixelBearing = -1
    gammaCorrectionVal = 1.43
    filterchainImg = None
    angle=0.0 
    allBoxList = []
    collectedBoxList = []
    minBoxDistance = 50.0
    freqFactor = 4


    def __init__(self):
        self.sonarImgSub = None
        self.sonarFilterPub = None
        #self.sonarPixelSrv = None
        self.registerNode()
    
    def registerNode(self):
        self.sonarImgSub = rospy.Subscriber("/sonar_image", Image, self.sonarFilterCallback)
        #self.sonarPixelSrv = rospy.ServiceProxy('/sonar_pixel', sonar_pixel)
        #self.sonarPixelSrv.wait_for_service()
        self.sonarFilterPub = rospy.Publisher("/Vision/sonar_filterpool", Image)
        self.sonarFilterChainPub = rospy.Publisher("/Vision/sonar_filterchainpool", Image)


    def sonarFilterCallback(self, rosImg): 
        filterImg = self.processSonarImage(self.rosImgToCVMono(rosImg))
        if filterImg is not None:
            try:
               self.publishImage(filterImg) 
            except Exception, e:
                pass
        else:
            rospy.logwarn("filter image null") 

    
    def publishImage(self, filterImg):
        self.sonarFilterPub.publish(Utils.cv2rosimg(filterImg)) 

    
    def publishFilterChainImg(self, filterchainImg):
        filterchainImg = cv2.cvtColor(filterchainImg, cv2.COLOR_GRAY2BGR)
        self.sonarFilterChainPub.publish(self.cv2rosimgBgr(filterchainImg))

    def expProcessing(self, sourceImg):
        gammaImg = sourceImg.copy() 
        cv2.cv.ConvertScale(cv2.cv.fromarray(gammaImg), cv2.cv.fromarray(gammaImg), 1.0/255, 0)
        cv2.pow(gammaImg, self.gammaCorrectionVal, gammaImg)
        return binaryImg


    def processSonarImage(self, sourceImg):
        procImg = None
        gammaImg = None
        sourceImg = cv2.resize(sourceImg, (640, 480))
        #binaryImg = cv2.cvtColor(sourceImg, cv2.COLOR_BGR2GRAY)
        binaryImg = cv2.cvtColor(sourceImg, cv2.COLOR_BGR2GRAY)

        #binaryImg = cv2.medianBlur(binaryImg, 3)
        binaryImg = cv2.GaussianBlur(binaryImg, (5,5), 0)
        
        highest = cv2.minMaxLoc(binaryImg)[1] 
        self.threshold = highest - self.threshOffset
        if self.threshold < self.threshBound:
            self.threshold = self.threshBound  
                
        self.filterchainImg = cv2.threshold(binaryImg, self.threshold, 255, cv2.THRESH_BINARY)[1] 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        self.filterchainImg = cv2.dilate(self.filterchainImg, kernel)
        self.filterchainImg = cv2.morphologyEx(self.filterchainImg, cv2.MORPH_CLOSE, kernel, iterations=3)
        self.filterchainImg = cv2.erode(self.filterchainImg, (3,9), iterations=2) 

        mask = cv2.threshold(self.filterchainImg, self.threshold, 255, cv2.THRESH_TOZERO)[1]

        grayImg = sourceImg.copy()
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #allBoxList = []
        for i in contours:
            mask = np.zeros((640,480), dtype=np.uint8)
            cv2.drawContours(mask, [i], 0, 255, -1)

            rect = cv2.minAreaRect(i)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box) 
            
            dim = self.getDimensions(box)
            dx = dim[0]
            dy = dim[1]

            cv2.drawContours(grayImg, [box], 0, (255,0,0), 1)
            if self.lenLowerBound < abs(dx) < self.lenUpperBound \
                and self.widthLowerBound < abs(dy) < self.widthUpperBound \
                and self.checkTargetXAxis(box):
                    cv2.drawContours(self.filterchainImg, [box], 0, (255,0,0), 2)
                    cv2.drawContours(grayImg, [box], 0, (255,0,0), 1)
                    self.collectedBoxList.append(box)
      
        self.checkDetections(grayImg)
        self.publishFilterChainImg(self.filterchainImg)
        return grayImg
            
            
    def checkDetections(self, grayImg):
        allBoxList = []
        if self.numFrames > self.requiredFrames:
            for box in self.collectedBoxList:
                if self.checkFrequency(box):
                    firstPoint = box[0]
                    secondPoint = box[3]
                    midPoint = ((firstPoint[0]+secondPoint[0])/2, (firstPoint[1]+secondPoint[1])/2)
                    dimensions = self.getDimensions(box)
                    allBoxList.append([box, dimensions[0], dimensions[1], midPoint])
            self.numFrames = 0
            self.collectedBoxList = []
        else:
            self.numFrames += 1
            
        if len(allBoxList) > 0 and self.numFrames == 0:
#            rospy.loginfo("box detected")
            boxMap = self.findNearestBox(allBoxList)
            box = boxMap[0]
            cv2.drawContours(grayImg, [box], 0, (0,255,0), 2)
            grayImg = self.getRangeBearing(grayImg, boxMap[1], boxMap[2], boxMap[3])
        return grayImg


    def findNearestBox(self, allBoxList):
        origin = [0, 0]
        minDist = 0.0
        for boxMap in allBoxList:
            centroid = boxMap[3]
            dist = math.sqrt(math.pow(origin[0]-centroid[0], 2) + math.pow(origin[1]-centroid[1], 2))
            if dist < minDist:
                nearestBox = boxMap
        return boxMap

    
    
    def getDimensions(self, box):
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
        return [dx, dy]


    def checkFrequency(self, box):
        freqCount = 0.0
        midpoint = (box[0] + box[3])/2
        for box in self.collectedBoxList:
            mpoint = (box[0] + box[3])/2 
            if math.sqrt(math.pow(mpoint[0]-midpoint[0], 2) + math.pow(mpoint[1]-midpoint[1], 2)) < self.minBoxDistance :
                freqCount += 1
        if freqCount >= len(self.collectedBoxList)/self.freqFactor:
            return True
        return False    


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
        
        #self.angle = self.normalizeAngle(self.angle)
        if self.angle < 0.0:
            self.angle = -self.angle

        if  150.0 < self.angle or self.angle < 30.0:
            #rospy.loginfo("angle within +-30") 
            return True
        else:
            #rospy.loginfo("angle > 30")
            return False


    def normalizeAngle(self, angle):
        while angle < 0.0:
            angle += 360.0
        return (angle % 360)


    def getRangeBearing(self, procImg, dx, dy, midPoint):
        #rsp = self.sonarPixelSrv(x=int(midPoint[0]), y=int(midPoint[1]))
        #self.pixelRange = rsp.range
        #self.pixelBearing = rsp.bearing
        
        cv2.putText(procImg, "Angle " + str(self.angle), (430, 370), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        cv2.putText(procImg, "Threshold " + str(self.threshold), (430, 385), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        cv2.putText(procImg, "dx: " + str(dx), (430, 400), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        cv2.putText(procImg, "dy: " + str(dy), (430, 415), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        #cv2.putText(procImg, "Range " + str(self.pixelRange), (430, 430), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        #cv2.putText(procImg, "Bearing " + str(self.pixelBearing), (430, 445), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))

        return procImg
        

    def rosImgToCVMono(self, rosImg):
        try:
            cvImg =  self.bridge.imgmsg_to_cv2(rosImg, desired_encoding="mono8")
            cvImg = cv2.cvtColor(cvImg, cv2.COLOR_GRAY2BGR)
        except CvBridgeError as e:
            rospy.logerr(e)
        return cvImg


    def cv2rosimgBgr(self, filterchainImg):
        try:
            return Utils.bridge.cv2_to_imgmsg(filterchainImg, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return


def main():
    rospy.init_node("sonarFilter", anonymous=False)
    filterChain = SonarFilter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("sonarFilter interrupted");

