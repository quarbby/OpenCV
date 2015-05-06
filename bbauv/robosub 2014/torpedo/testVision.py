#/usr/bin/env/python 

'''
Power Vision
'''
import math
import static
import numpy as np
import cv2
from sensor_msgs.msg import Image

import rospy
import signal

from utils.utils import Utils


#GLOBAL
b_lowThresh = np.array([100,0,130], dtype=np.uint8)
b_hiThresh = np.array([230,85,255], dtype=np.uint8)
p_lowThresh = np.array([0,100,0], dtype=np.uint8)
p_hiThresh = np.array([120,255,255], dtype=np.uint8)
s_lowThresh = np.array([130,0,0], dtype=np.uint8)
s_hiThresh = np.array([180,150,150], dtype=np.uint8)

class Img:

    def __init__(self, img, min, max):
        self.rawImg = cv2.resize(img, (640, 480))
        #self.preHsv = static.whiteBal(self.rawImg)
        self.preHsv = static.blurr(self.rawImg)
        #self.preHsv = static.removeLumi(self.preHsv)
        self.hsvImg = cv2.cvtColor(self.preHsv, cv2.COLOR_BGR2HSV)
        self.enhancedImg = self.hsvImg
        self.labImg = cv2.cvtColor(self.rawImg, cv2.COLOR_BGR2LAB)
        self.lab_bgr = cv2.cvtColor(static.labSetup(self.labImg), cv2.COLOR_GRAY2BGR)
        self.grayImg = cv2.cvtColor(self.rawImg, cv2.cv.CV_BGR2GRAY)
        self.grayImg = cv2.threshold(self.grayImg,50,255, cv2.THRESH_TRUNC)[1]
        self.gray_bgr = cv2.cvtColor(self.grayImg, cv2.COLOR_GRAY2BGR)
        self.enhanced_bgr = cv2.cvtColor(self.enhancedImg, cv2.COLOR_HSV2BGR)
        self.maskImg = static.morph(static.inRange(self.labImg, min, max))
        self.mask_bgr = cv2.cvtColor(self.maskImg, cv2.COLOR_GRAY2BGR)
        #static.circleCont(self.maskImg, self.mask_bgr)
        self.detected = False
        self.container = []
        

class PowerVision:    
        
    def __init__(self):
        signal.signal(signal.SIGINT, self.signalHandler)
        self.isSonar = rospy.get_param("~sonar", True)
        #self.cam_sub = rospy.Subscriber("/front_camera/camera/image_raw_jin", Image, self.camCallback)
        if self.isSonar:
            self.sonar_sub = rospy.Subscriber("/sonar_image", Image, self.sonarCallback)

    def publishImg(self, img, img2):
        maskPub = rospy.Publisher("/Vision/image_filter_mask", Image)
        mask_redPub = rospy.Publisher("/Vision/image_filter_mask_red", Image)
        mask_bluePub = rospy.Publisher("/Vision/image_filter_mask_blue", Image)
        maskPub.publish(Utils.cv2rosimg(img.mask_bgr))
        mask_redPub.publish(Utils.cv2rosimg(img2))
        #mask_bluePub.publish(Utils.cv2rosimg(img3.mask_bgr))

    def sonarCont(self, img, draw):
        contours, hierr = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = [cv2.convexHull(i) for i in contours]
        sorted(contours, key=cv2.contourArea, reverse=True)
        cnt = contours[0] 
        rect = cv2.minAreaRect(cnt)
        box = cv2.cv.BoxPoints(rect)
        corners = static.getCorner(box)
        pt1 = (int(corners[2][0]), int(corners[2][1]))
        pt2 = (int(corners[3][0]), int(corners[3][1]))
        cv2.line(draw, pt1, pt2, (0,0,255), 3)
        
   
    def sobelLine(self,mask):
        final_mask = np.zeros((480,640,3), dtype=np.uint8)
        sobel = cv2.Sobel(mask, cv2.CV_8U, 0, 1, (3,11))
        contours, hiera = cv2.findContours(sobel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i in contours:
            mask = np.zeros((640,480), dtype=np.uint8)
            cv2.drawContours(mask, [i], 0, 255, -1)
            lines = cv2.HoughLinesP(mask, 1, math.pi/2, 1, None, 1, 8)
            if lines is not None:
                line = lines[0]
                x1 = [i[0] for i in line]
                x2 = [i[2] for i in line]
                y1 = [i[1] for i in line]
                y2 = [i[3] for i in line]
                pt1 = (min(min(x1), min(x2)), min(min(y1), min(y2)))
                pt2 = (max(max(x1), max(x2)), max(max(y1), max(y2)))
                cv2.line(final_mask, pt1, pt2 , (0,0,255), 3)
        return final_mask

    def isLine(self,line):
        dy = abs((line[1] - line[3]))
        dx = abs((line[0] - line[2]))
        return dy ==0 and dx > 30 

    def sonarCallback(self, rosimg):
        rospy.loginfo("Inside sonar")
        cvImg = Utils.rosimg2cv(rosimg)
        gray = cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        sobel_bgr = self.sobelLine(mask)
        #self.sonarCont(mask, mask_bgr)
        sonarPub = rospy.Publisher("/Vision/image_filter_sonar", Image)
        sonarPub.publish(Utils.cv2rosimg(sobel_bgr))

    def gotPeg(self,cnt):
        all = cv2.minMaxLoc(cnt)
        return all[0] < 20

    def isRect(self,cnt):
        x,y,w,h = cv2.boundingRect(cnt)
        return float(w)/h < 1.5 and cv2.contourArea(cnt) > 300
        
    def drawSlot(self, mask, draw):
        container = []
        contours,hierr = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in contours:
            if cv2.contourArea(i) > 200:
                circle = cv2.minEnclosingCircle(i)
                cnt_x = int(circle[0][0])
                cnt_y = int(circle[0][1])
                cv2.circle(draw, (cnt_x, cnt_y), int(circle[1]),(255,0,0),2)
                cv2.circle(draw, (cnt_x, cnt_y), 1 ,(255,0,0),2)
                #cv2.drawContours(draw, [i], -1, (255,0,0), 2)
                container.append(i)

    def drawPeg(self, mask, draw):
        contours,hierr = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i in contours:
            circle = cv2.minEnclosingCircle(i)
            cnt_x = int(circle[0][0])
            cnt_y = int(circle[0][1])
            cv2.circle(draw, (cnt_x, cnt_y), int(circle[1]),(0,0,255),2)
            cv2.circle(draw, (cnt_x, cnt_y), 1 ,(0,0,255),2)

    def drawBoard(self,mask,draw):
        contours, hierr = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = 0
        skew = None
        sorted(contours, key=cv2.contourArea, reverse=True)
        for i in contours:
            if cnt is 0 and self.isRect(i):
                cnt = cv2.convexHull(i)
                epsilon = 0.005*cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, epsilon, True)
                mom = cv2.moments(cnt)
                mom3 = mom['m30']
                mom2 = pow(mom['m02'], 3/2)
                skew = (mom3+0.00001)/(mom2+0.00001)

        if cnt is not 0:
            centroid = static.getCentroid(cv2.moments(cnt))
            rect = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(draw, [box], -1, (0,255,255), 3)
            cv2.circle(draw, centroid, 5, (0,255,255), 7)
            

    def skewtry(self, img):
        w, h = img.shape[:2]
        contours, hierr = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        print(len(contours))
        m = cv2.moments(contours[0])
        x = m['m10']/m['m00']
        y = m['m01']/m['m00']
        mu02 = m['mu02']
        mu20 = m['mu20']
        mu11 = m['mu11']
         
        lambda1 = 0.5*( mu20 + mu02 ) + 0.5*( mu20**2 + mu02**2 - 2*mu20*mu02 + 4*mu11**2 )**0.5
        lambda2 = 0.5*( mu20 + mu02 ) - 0.5*( mu20**2 + mu02**2 - 2*mu20*mu02 + 4*mu11**2 )**0.5
        lambda_m = max(lambda1, lambda2)
         
        # Convert from radians to degrees
        angle = math.ceil(math.atan((lambda_m - mu20)/mu11)*18000/math.pi)/100
        print angle
        # Create a rotation matrix and use it to de-skew the image.
        center = tuple(map(int, (x, y)))
        rotmat = cv2.getRotationMatrix2D(center, angle , 1)
        rotatedImg = cv2.warpAffine(img, rotmat, (w, h), flags = cv2.INTER_CUBIC)
        return rotatedImg

    def camCallback(self, rosimg):
        rospy.loginfo("Inside cam")
        cvImg = Utils.rosimg2cv(rosimg)
        board = Img(cvImg, b_lowThresh, b_hiThresh)
        self.drawBoard(board.maskImg,board.mask_bgr)
        peg = Img(cvImg, p_lowThresh, p_hiThresh)
        slot = Img(cvImg, s_lowThresh, s_hiThresh)
        slot.maskImg = static.morph(slot.maskImg)
        slot.mask_bgr = cv2.cvtColor(slot.maskImg, cv2.COLOR_GRAY2BGR)
        total_img = cv2.bitwise_or(slot.mask_bgr, peg.mask_bgr)
        total_img = cv2.bitwise_or(total_img, board.mask_bgr)
        self.drawSlot(slot.maskImg, total_img)
        self.drawPeg(peg.maskImg, total_img)
        self.publishImg(board, total_img)

    def signalHandler(self,signum, frame):
        self.cam_sub.unregister()
        rospy.signal_shutdown("Killed")


def main():
    rospy.init_node("PowerMaster")
    power = PowerVision()
    rospy.spin()

if __name__ == "__main__":
    main()
