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
            'thres': 160, 'dyMult': 80,
            'bearing': 0, 'dist': 0}
    screen = {'height': 480, 'width': 640}
    imgOffset = 60

    def __init__(self):
        rospy.loginfo("Vision filter initialised")
        # self.sonarSrv = rospy.ServiceProxy('/sonar_pixel', sonar_pixel, persistent=True)

    def setParams(self, params):
        self.params = params

    def createThresImg(self, img):
        binImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(binImg, self.params['thres'], 255, cv2.THRESH_TOZERO)[1]

        mask = cv2.erode(mask, (3,9), iterations=2)

        scratchImgCol = img

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
            try:
                grad = np.rad2deg(math.atan(float(dy)/dx))
            except Exception, e:
                grad = 0

            centerX = (box[3][0]+box[0][0])/2
            centerY = (box[3][1]+box[0][1])/2

            point = self.screen['height'] - self.imgOffset
            dist = (point*1.0 - centerY)/point * 10.0   # Assume sonar max reach = 10

            # cv2.drawContours(point*1.0 - centerY)/point * 10.0

            if self.params['lenLo'] < abs(dx) < self.params['lenHi'] \
                    and dist > 2.0 \
                    and abs(dy) < self.params['dyMult']:

                    cv2.drawContours(scratchImgCol, [box], 0, (0, 255, 0), 3)
                    allLinesList.append((dist, grad, box))

        sorted(allLinesList, key=lambda x:x[0])
        if len(allLinesList) > 0:
            target = allLinesList[0]
            self.params['dist'] = target[0]
            self.params['bearing'] = target[1]
            
            startPoint = tuple(target[2][0])
            endPoint = tuple(target[2][3])
            self.centerPoint = ((startPoint[0]+endPoint[0])/2, (startPoint[1]+endPoint[1])/2)

            # If service is up
            # self.getSonarPoint(self.centerPoint)

            cv2.circle(scratchImgCol, self.centerPoint, 8, (20,255,255), 3)

        return scratchImgCol

    def getSonarPoint(self, point):
        resp = self.sonarSrv(x=int(point[0]), y=int(point[1]))
        self.params['bearing'] = resp.bearing
        self.params['dist'] = resp.dist

    def getBearingDist(self):
        return self.params['bearing'], self.params['dist']
