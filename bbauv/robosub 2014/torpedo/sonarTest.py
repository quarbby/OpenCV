#!/usr/bin/env python

import roslib
import rospy

from sensor_msgs.msg import Image
from bbauv_msgs.srv import sonar_pixel

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

from dynamic_reconfigure.server import Server as DynServer
from utils.config import sonarConfig as Config

import math
import numpy as np
import cv2

class Sonar():
    threshold = 130
    sobelKern = (3, 11)
    lenLowerBound = 30
    lenUpperBound = 200
    imgOffset = 60
    dyMult = 80
    centerPoint = (-1, -1)

    def __init__(self, comms=None):
        self.comms = comms

        self.sonarDist = 0.0
        self.sonarBearing = 0.0
        self.registerSonar()

        # if self.comms is None:
        #     self.dynServer = DynServer(Config, self.dynReconfigure)

        # self.sonarSrv = rospy.ServiceProxy('/sonar_pixel', sonar_pixel, persistent=True)

    def gotSonarFrame(self, img):
        img = cv2.resize(img, (vision.screen['width'], vision.screen['height']))

        binImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # binImg = img

        # Calculate mean of pixels and threshold
        mean = cv2.mean(binImg)[0]
        lowest = cv2.minMaxLoc(binImg)[0]
        self.threshold = mean*4.5

        binImg = cv2.GaussianBlur(binImg, ksize=(5,5), sigmaX=10)
        return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        # mask = cv2.threshold(binImg, self.threshold, 255, cv2.THRESH_TOZERO)[1]

        # Convolution kernel
        # kernel = np.array([[1,-2,1], [2,-4,2],[1,-2,1]])
        # mask = cv2.filter2D(binImg, cv2.CV_8U, kernel)
        # return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        mask = cv2.erode(mask, (3,9), iterations=2)
        mask = cv2.erode(mask, (3,9), iterations=2)

        scratchImgCol = img
        self.drawDist(scratchImgCol)
        cv2.putText(scratchImgCol, "SONAR PROCESSED", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))
        cv2.putText(scratchImgCol, "Today's Threshold: " + str(self.threshold), (20, 430),
            cv2.FONT_HERSHEY_PLAIN, 1, (204,204,204))

        # Sobel image
        # zerosmask = np.zeros((480,640,3), dtype=np.uint8)
        # sobel = cv2.Sobel(mask, cv2.CV_8U, 0, 1, self.sobelKern)
        # dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        # sobel = cv2.dilate(sobel, dilateEl, iterations=2)
        # return cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)

        # scratchImgCol = cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)
        # cv2.putText(scratchImgCol, "SONAR PROCESSED", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))

        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        allLinesList = []

        for i in contours:
            mask = np.zeros((640,480), 
                dtype=np.uint8)
            cv2.drawContours(mask, [i], 0, 255, -1)

            rect = cv2.minAreaRect(i)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)

            dx = box[3][0] - box[0][0]
            dy = box[3][1] - box[0][1]
            grad = np.rad2deg(math.atan(float(dy)/dx))

            centerX = (box[3][0]+box[0][0])/2
            centerY = (box[3][1]+box[0][1])/2

            point = vision.screen['height'] - self.imgOffset
            dist = (point*1.0 - centerY)/point * 10.0

            cv2.drawContours(scratchImgCol, [box], 0, (255,0,0), 2)
            # cv2.putText(scratchImgCol, str("{0:.2f}".format(dy)),
            #     (centerX, centerY-10), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1)

            # if float(dx)/dy > 1.0 and self.lenLowerBound < abs(dx) < self.lenUpperBound and \
            #      abs(dy) < 50 \
            #     and dist > 2.0:
            if self.lenLowerBound < abs(dx) < self.lenUpperBound \
                and dist > 2.0 \
                and abs(dy) < self.dyMult \
                and (float(dx)/dy) < 0:

                cv2.drawContours(scratchImgCol, [box], 0, (0,255,0), 2)

                # cv2.putText(scratchImgCol, str("{0:.2f}".format(dx)) + " " + 
                #     str("{0:.2f}".format(dy)) + " " + str("{0:.2f}".format(dist)),
                #     (centerX, centerY), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

                allLinesList.append((dist, grad, box))

        sorted(allLinesList, key=lambda x:x[0])
        if len(allLinesList) > 0:
            target = allLinesList[0]
            startPoint = tuple(target[2][0])
            endPoint = tuple(target[2][3])
            self.centerPoint = ((startPoint[0]+endPoint[0])/2, (startPoint[1]+endPoint[1])/2)

            cv2.line(scratchImgCol, startPoint, endPoint, (0,0,255), 2)
            cv2.circle(scratchImgCol, self.centerPoint, 5, (20, 255, 255), 2)
            centerStr = "("+ "{0:.2f}".format(self.centerPoint[0])+","+ \
                "{0:.2f}".format(self.centerPoint[1])+")"
            cv2.putText(scratchImgCol, str(centerStr), 
                (int(self.centerPoint[0]), int(self.centerPoint[1]-20)),
                cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1)

            self.sonarDist = target[0]
            self.sonarBearing = target[1]

            '''

            lines = cv2.HoughLinesP(mask, 1, math.pi/2, 1, None, 1, 0)

            if lines is not None:
                line = lines[0]

                pt1List = [(i[0], i[1]) for i in line]
                pt2List = [(i[2],i[3]) for i in line]
                sorted(pt1List, key=lambda x:x[0])
                sorted(pt2List, key=lambda x:x[0])
                pt1temp = pt1List[0] if pt1List[0] < pt2List[0] else pt2List[0]
                pt2temp = pt1List[-1] if pt1List[-1] > pt2List[-1] else pt2List[-1]

                # Check which x is smaller 
                if pt1temp[0] < pt2temp[0]:
                    pt1 = pt1temp
                    pt2 = pt2temp
                else:
                    pt1 = pt2temp
                    pt2 = pt1temp

                length = Utils.distBetweenPoints(pt1, pt2)

                if self.lenLowerBound < length < self.lenUpperBound:
                    centerPoint = ((pt1[0]+pt2[0])/2, (pt2[1]+pt2[1])/2 )
                    angle = self.calculateAngle(pt1, pt2)

                    if -35 < angle < 35:
                        # cv2.circle(scratchImgCol, pt1, 5, (20, 255, 255), 2)
                        # cv2.circle(scratchImgCol, pt2, 5, (20, 255, 255), 2)

                        allLinesList.append((pt1, pt2))

                        cv2.line(scratchImgCol, pt1, pt2, (0,0,255), 3)
                        angleStr = "{0:.2f}".format(angle)
                        cv2.putText(scratchImgCol, "Ang " + str(angleStr),
                            (int(pt1[0]), int(pt1[1]-5)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1)

                        offset = self.imgOffset
                        point = (vision.screen['height']-offset)
                        dist = ((point-pt1[1])*1.0/point) * 10.0  # 10m the FOV of sonar
                        distStr = "{0:.2f}".format(dist)
                        cv2.putText(scratchImgCol, "Dist " + str(distStr),
                            (int(pt1[0]), int(pt1[1]-20)), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 1)

                        allLinesList.append((angle, dist, centerPoint))

        if len(allBearingList) > 0:
            sorted(allLinesList, key=lambda p:p[1])
            self.sonarDist = allBearingList[0][1]
            self.sonarBearing = allBearingList[0][0]
            self.centerPoint = (int(allBearingList[0][2][0]), int(allBearingList[0][2][1]))

            cv2.circle(scratchImgCol, point, 5, (20, 255, 255), 2)

        '''

        cv2.putText(scratchImgCol, "Sonar Dist " + str(self.sonarDist),
            (30, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

        cv2.putText(scratchImgCol, "Sonar Bearing " + str(self.sonarBearing), (30, 60),
                cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

        if self.comms is not None:
            self.comms.sonarBearing = self.sonarBearing

        # scratchImgCol = self.getSonarPoint(scratchImgCol)

        return scratchImgCol

    def getSonarPoint(self, outImg):
        resp = self.sonarSrv(x=int(self.centerPoint[0]), y=int(self.centerPoint[1]))
        self.ashBearing = resp.bearing
        self.ashDist = resp.range

        # Put these to comms sonar dist and bearing
        if self.comms is not None:
            self.comms.sonarDist = self.ashDist
            self.comms.sonarBearing = self.ashBearing

        cv2.putText(outImg, "BV Ang " + str(self.ashBearing), (430, 430),
            cv2.FONT_HERSHEY_PLAIN, 1, (255,51,153))
        cv2.putText(outImg, "BV Dist "+str(self.ashDist), (430, 460),
            cv2.FONT_HERSHEY_PLAIN, 1, (255,51,153))

        return outImg

    def registerSonar(self):
        rospy.loginfo("SONAR SONAR")
        self.sonarSub = rospy.Subscriber("/sonar_image", Image, self.sonarImageCallback)
        self.sonarPub = rospy.Publisher("/sonar_pub", Image)
        rospy.sleep(rospy.Duration(0.05))
    
    def sonarImageCallback(self, rosImg):
        outImg = self.gotSonarFrame(self.rosImgToCVMono(rosImg))
        if outImg is not None:
            try:
                self.sonarPub.publish(Utils.cv2rosimg(outImg))
            except Exception, e:
                pass

        # rospy.sleep(rospy.Duration(0.05))

    def reconfigure(self):
        self.threshold = self.comms.sonarParams['threshold']
        self.sobelKern = self.comms.sonarParams['sobelKern']
        self.lenLowerBound = self.comms.sonarParams['lenLowerBound']
        self.lenUpperBound = self.comms.sonarParams['lenUpperBound']

        rospy.loginfo("Sonar reconfigured")

    def dynReconfigure(self, config, levels):
        # self.threshold = config.binThres
        self.sobelKern = (config.sobel1, config.sobel2)
        self.lenLowerBound = config.length1
        self.lenUpperBound = config.length2

        return config

    def rosImgToCVMono(self, img):
        try:
            frame = Utils.bridge.imgmsg_to_cv2(img, desired_encoding="mono8")
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame

    def drawDist(self, img):
        width = vision.screen['width']/2
        imgOffset = 47
        imgHeight = vision.screen['height'] - imgOffset
        colour = (224,224,224)

        rangeList = [1, 2, 4, 6, 8, 10]

        # for i in range(len(rangeList)):
        #     val = rangeList[i]
        #     ptPoz = (val/10.0) * imgHeight - self.imgOffset
        #     centerX = int(imgHeight - ptPoz)
        #     cv2.putText(img, str(val)+"m", (width, centerX-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)
        #     cv2.ellipse(img, (width, centerX), (280-20*i, 33), 0, 180, 360, colour, 1)

        # 10 m
        cv2.ellipse(img, (width, 98), (280, 33), 0, 180, 360, colour, thickness=1)
        cv2.putText(img, "10m", (width, 98-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)

        ptPoz = (10.0/10.0) * imgHeight - imgOffset
        centerX = int(imgHeight - ptPoz)
        cv2.putText(img, "THIS SEEMS 10 M", (width-5, centerX+3), cv2.FONT_HERSHEY_DUPLEX, 1, colour, 1)
        cv2.ellipse(img, (width, centerX), (280, 33), 0, 180, 360, colour, 1)

        # 8 m
        ptPoz = (8.0/10.0) * imgHeight - imgOffset
        centerX = int(imgHeight - ptPoz)
        cv2.putText(img, "8m", (width, centerX-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)
        cv2.ellipse(img, (width, centerX), (260, 33), 0, 180, 360, colour, 1)

        # 6 m
        ptPoz = (6.0/10.0) * imgHeight - imgOffset
        centerX = int(imgHeight - ptPoz)
        cv2.putText(img, "6m", (width, centerX-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)
        cv2.ellipse(img, (width, centerX), (210, 33), 0, 180, 360, colour, 1)

        # 4 m
        ptPoz = (4.0/10.0) * imgHeight - imgOffset
        centerX = int(imgHeight - ptPoz)
        cv2.putText(img, "4m", (width, centerX-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)
        cv2.ellipse(img, (width, centerX), (160, 33), 0, 180, 360, colour, 1)

        # 2 m
        ptPoz = (2.0/10.0) * imgHeight - imgOffset
        centerX = int(imgHeight - ptPoz)
        cv2.putText(img, "2m", (width, centerX-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)
        cv2.ellipse(img, (width, centerX), (100, 25), 0, 180, 360, colour, 1)

        # 1 m
        ptPoz = (1.0/10.0) * imgHeight - imgOffset
        centerX = int(imgHeight - ptPoz)
        cv2.putText(img, "1m", (width, centerX-38), cv2.FONT_HERSHEY_PLAIN, 1, colour, 1)
        cv2.ellipse(img, (width, centerX), (75, 15), 0, 180, 360, colour, 1)

        return img


def main():
    rospy.init_node('SonarRanger')
    sonar = Sonar()
    rospy.spin()
