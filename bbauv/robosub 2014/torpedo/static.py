#!/usr/bin/env python
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError

#GLOBAL
screen_h = 240
screen_w = 320
bridge = CvBridge()


def normlize(img):
    channels = cv2.split(img)
    channels[2] = cv2.normalize(channels[2], 0,255,cv2.NORM_MINMAX)
    return cv2.merge(channels, img)

def grayWorld(img):
    channels = cv2.split(img)
    meanB = int(cv2.mean(channels[0])[0])
    meanG = int(cv2.mean(channels[1])[0])
    meanR = int(cv2.mean(channels[2])[0])
    img = cv2.merge(channels, img)
    rows, cols, channel = img.shape 
    for i in xrange(rows):
        for j in xrange(cols):
            temp = img[i,j]
            mean = np.mean(temp)
            scaleB = mean/meanB
            scaleG = mean/meanG
            scaleR = mean/meanR
            img[i,j] = [scaleB*temp[0], scaleG*temp[1], scaleR*temp[2]]
    return img
    
def whiteBal(img):
    channels = cv2.split(img)
    channels[0] = cv2.equalizeHist(channels[0])
    channels[1] = cv2.equalizeHist(channels[1])
    #channels[1] = cv2.normalize(channels[1], 0, 255, cv2.NORM_MINMAX)
    channels[2] = cv2.equalizeHist(channels[2])
    img = cv2.merge(channels, img)
    img = cv2.medianBlur(img, 9)
    #img = cv2.bilateralFilter(img, -1, 5, 0.3)
    return img

def equalHist(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(img)
    channels[0] = cv2.equalizeHist(channels[0])
    img = cv2.merge(channels, img)
    return cv2.cvtColor(img, cv2.COLOR_YCR_CB2BGR)
 
def removeLumi(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    channels = cv2.split(img)
    channels[0] = cv2.equalizeHist(channels[0])
    img = cv2.merge(channels, img)
    return cv2.cvtColor(img, cv2.COLOR_YUV2BGR)

def powerUp(img):
    gauss = cv2.GaussianBlur(img, (5,5), 9)
    sum = cv2.addWeighted(img, 1.5, gauss, -0.6, 0)
    sum = cv2.medianBlur(sum, 3)
    return sum

def morphRed(img):
    kern = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    img = cv2.dilate(img, kern, iterations=4)
    img = cv2.erode(img, kern, iterations=5)
    img = cv2.dilate(img, kern, iterations=3)
    img = cv2.erode(img, kern, iterations=4)
    #img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
    return img
   
def morphGreen(img):
    kern = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    img = cv2.dilate(img, kern, iterations=4)
    #img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
    return img

def inRange(img, min, max):
    mask = cv2.inRange(img, min, max)
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    mask = cv2.dilate(mask, kern, iterations = 3)
    return mask



def deBlue(img):
    channel = cv2.split(img)
    blue_mean = cv2.mean(channel[0])
    print(blue_mean)
    for i in channel[1]:
        i -= blue_mean
    for i in channel[2]:
        i -= blue_mean
    img = cv2.merge(channel, img)
    #img = cv2.equalizeHist(img)
    return img

def toHsv(img, lowH=0, highH=1, lowW=0, highW=1):
    img = whiteBal(img) #Perfect 
    #img = equalHist(img) #Red turns black 
    #img = removeLumi(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = np.array(hsv, dtype=np.uint8)
    return hsv

def findContour(img, draw):
    cnt = 0
    contours, hira = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    sorted(contours, key=cv2.contourArea, reverse=True) 
    for i in contours:
        rect = cv2.minAreaRect(i)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        if cv2.contourArea(i) > 100 and isRect(i):
            #cv2.drawContours(draw, [box], -1, (0,0,255), 2)
            cnt = i
    return cnt

def findContourGreen(img):
    contours, hira = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    sorted(contours, key=cv2.contourArea, reverse=True) 
    if not contours:
        return 0
    elif len(contours) > 1:
        #approx = [approxCnt(i) for i in contours]
        #sorted(approx, key=cv2.contourArea, reverse=True)
        return contours[0]
    else:
        cnt = contours[0]
        #epsilon = 0.005*cv2.arcLength(cnt, True)
        #approx = cv2.approxPolyDP(cnt, epsilon, True)
        return cnt
    

def approxCnt(cnt):
    epsilon = 0.005*cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    return approx

def isRect(cnt):
    box = getBox(cnt)
    corner = getCorner(box)  
    width = abs((corner[0])[0] - (corner[1])[0])
    length = abs((corner[0])[1] - (corner[2])[1])
    return (length/width) > 2

def getBox(cnt):
    rect = cv2.minAreaRect(cnt)
    box = cv2.cv.BoxPoints(rect)
    box = np.int0(box)
    return box

def getCentroid(mom, draw):
    centroid_x = int((mom['m10']+0.0001)/(mom['m00']+0.0001))
    centroid_y = int((mom['m01']+0.0001)/(mom['m00']+0.0001))
    return (centroid_x, centroid_y)


def getCorner(box):
    x = [i[0] for i in box]
    y = [i[1] for i in box]
    top_left = (min(x), max(y))
    top_right = (max(x), max(y))
    bot_right = (max(x), min(y))
    bot_left = (min(x), min(y))
    return [top_left, top_right, bot_left, bot_right]

def getAngle(cnt):
    x =[i[0][0] for i in cnt]
    y =[i[0][1] for i in cnt]
    top_left = (min(x), max(y))
    top_right = (max(x), max(y))
    bot_right = (max(x), min(y))
    bot_left = (min(x), min(y))
    hypo = math.sqrt(math.pow(bot_right[0] - top_left[0], 2) + math.pow(top_left[1]-bot_right[1], 2))
    hypo2 = math.sqrt(math.pow(top_right[0] - bot_left[0], 2) + math.pow(top_right[1]-bot_left[1], 2))
    if hypo > hypo2:
        ang = np.rad2deg(math.asin((top_left[1]-bot_right[1])/hypo))
        return normalizeAng(ang, "left")
    else:
        ang =  np.rad2deg(math.asin((top_right[1]-bot_left[1])/hypo2))
        return normalizeAng(ang, "right")
  
def normalizeAng(ang, str):
    if str == "left":
        return (180 - ang, "left")
    else:
        return (ang, "right")
