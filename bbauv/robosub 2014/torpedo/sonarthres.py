#!/usr/bin/env python

''' A Sonar thresholding UI with simple algorithm '''

import roslib
import rospy
import os
import sys

import signal
import threading
import Queue

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt5

from qrangeslider import QRangeSlider
from sonarFilter import SonarFilter

class Sonar(QWidget):
    rate = 20
    params = {'lenHi': 150, 'lenLo': 20,
            'widthHi': 20, 'widthLo':2, 
            'thres': 200, 'dyMult': 80,
            'bearing': 0, 'dist': 0, 'thresOffset': 65}
    video_pixmap = Queue.Queue()
    cam_pixmap = Queue.Queue()

    def __init__(self, parent=None):
        super(Sonar, self).__init__(parent)
        self.bridge = CvBridge()
        self.sonarFilter = SonarFilter()
        self.initUI()
        self.initSub()

    def initUI(self):
        self.main_layout = QVBoxLayout()
        self.setGeometry(300, 300, 1000, 200)
        self.setWindowTitle("Sonar Filter GUI")

        # QRange sliders
        slider_layout = QVBoxLayout()
        thres_layout, self.thres_slider, self.thres_l = self.makeqslider_box("Thresh", 0, 1000, self.params['thres'])
        thres_offset_layout, self.thres_offset_slider, self.thresOffset_l = self.makeqslider_box("Thres Offset", 0, 1000, self.params['thresOffset'])
        len_layout, self.len_slider = self.make_slider_box("Length", 0, 1000, self.params['lenLo'], self.params['lenHi'])
        width_layout, self.width_slider = self.make_slider_box("Width", 0, 1000, self.params['widthLo'], self.params['widthHi'])
        
        slider_layout.addLayout(thres_layout)
        slider_layout.addLayout(thres_offset_layout)
        slider_layout.addLayout(len_layout)
        slider_layout.addLayout(width_layout)
        self.thres_slider.valueChanged.connect(self.thresSliderChanged)
        self.thres_offset_slider.valueChanged.connect(self.thresOffsetSliderChanged)
       
        # Calculated parameters layout
        params_layout = QHBoxLayout()
        heading_l = QLabel("<b>Heading</b>")
        self.heading_num_l = QLabel("0.0")
        dist_l = QLabel("<b>Dist</b>")
        self.dist_num_l = QLabel("0.0")

        params_layout.addWidget(heading_l)
        params_layout.addWidget(self.heading_num_l)
        params_layout.addWidget(dist_l)
        params_layout.addWidget(self.dist_num_l)

        # Sonar image layout
        image_layout = QHBoxLayout()
        video_l = QLabel("<b>Sonar</b>")
        cam_l = QLabel("<b>Camera</b>")
        threshold_l = QLabel("<b>Threshold</b>")

        self.video_cb = QLabel()
        self.cam_cb = QLabel()
        self.thres_cb = QLabel()

        video_layout = QVBoxLayout()
        video_layout.addWidget(video_l)
        video_layout.addWidget(self.video_cb)
        cam_layout = QVBoxLayout()
        cam_layout.addWidget(cam_l)
        cam_layout.addWidget(self.cam_cb)
        thres_layout = QVBoxLayout()
        thres_layout.addWidget(threshold_l)
        thres_layout.addWidget(self.thres_cb)

        image_layout.addLayout(cam_layout)
        image_layout.addLayout(video_layout)
        image_layout.addLayout(thres_layout)

        # Main frame layout
        self.main_layout.addLayout(slider_layout)
        self.main_layout.addLayout(params_layout)
        self.main_layout.addLayout(image_layout)

        self.init_timer(self.rate)
        self.setLayout(self.main_layout)
        self.show()

    def initSub(self):
        self.sonar_sub = rospy.Subscriber(rospy.get_param('~sonar',
            "/sonar_image"), Image, self.sonar_callback)
        self.cam_sub = rospy.Subscriber(rospy.get_param('~image',
            "/front_camera/camera/image_raw"), Image, self.cam_callback)

    def init_timer(self, time):
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.on_timer)
        self.timer.start(1000.0 / time)

    def sonar_callback(self, image):
        try:
            self.video_pixmap.put(image)
        except CvBridgeError, e:
            pass

    def cam_callback(self, image):
        try:
            self.cam_pixmap.put(image)
        except CvBridgeError, e:
            pass

    def on_timer(self):
        videoImg = None
        camImg = None
        thresImg = None

        self.updateParams()
        
        try:
            videoImg = self.video_pixmap.get(False, 0)
            self.video_pixmap = Queue.Queue()
        except Exception, e:
            pass

        try:
            camImg = self.cam_pixmap.get(False, 0)
            self.cam_pixmap = Queue.Queue()
        except Exception, e:
            pass

        if videoImg is not None:
            self.sonarFilter.setParams(self.params)
            thresImg = self.sonarFilter.createThresImg(self.rosImgToCVMono(videoImg))
            bearing, dist = self.sonarFilter.getBearingDist()
            self.setBearingDist(bearing, dist)
            thresImg = self.updateCameraImage(thresImg, isProcessed=True)
            self.thres_cb.setPixmap(thresImg.scaledToHeight(250))

            videoImg = self.updateCameraImage(videoImg)
            self.video_cb.setPixmap(videoImg.scaledToHeight(250))

        if camImg is not None:
            camImg = self.updateFrontCamImage(camImg)
            self.cam_cb.setPixmap(camImg.scaledToHeight(250))

    def rosImgToCVMono(self, img):
        try:
            frame = self.bridge.imgmsg_to_cv2(img, desired_encoding="mono8")
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        except CvBridgeError as e:
            rospy. logerr(e)
        return frame 

    def rosImgToCVBGR(self, img):
        try:
            frame = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame

    def make_qslider_box(self, name, min, max, val):
        label = QLabel(name)
        qsl = QSlider(Qt.Horizontal)
        qsl.setRange(min, max)
        qsl.setValue(val)
        qsl.setSingleStep(1)
        curVal = QLabel(str(qsl.value()))
        layout = QHBoxLayout()
        layout.addWidget(label)
        layout.addWidget(qsl)
        layout.addWidget(curVal)

        return (layout, qsl, curVal)

    def make_slider_box(self, name, min, max, start, end):
        label = QLabel(name)
        qse = QRangeSlider()
        qse.setMax(max)
        qse.setMin(min)

        qse.setRange(start,end)

        layout = QHBoxLayout()
        layout.addWidget(label)
        layout.addWidget(qse)

        return (layout, qse)

    def updateFrontCamImage(self, image):
        bbLock = threading.Lock()

        try:
            bbLock.acquire()
            image = self.rosImgToCVBGR(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            qimg = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
            qpm = QPixmap.fromImage(qimg)
        finally:
            bbLock.release()

        return qpm

    def updateCameraImage(self, image, isProcessed=False):
        bbLock = threading.Lock()

        try:
            bbLock.acquire()
            
            if not isProcessed:
                image = self.rosImgToCVMono(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            qimg = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
            qpm = QPixmap.fromImage(qimg)
        finally:
            bbLock.release()

        return qpm

    '''
    All the value changed updates
    '''
    def thresSliderChanged(self):
        self.params['thres'] = self.thres_slider.value()
        self.thres_l.setText(str(self.params['thres']))

    def thresOffsetSliderChanged(self):
        self.params['thresOffset'] = self.thres_offset_slider.value()
        self.thres_offset_l.setText(str(self.params['thresOffset']))

    def updateParams(self):
        minL, maxL = self.len_slider.getRange()
        self.params['lenLo'] = minL
        self.params['lenHi'] = maxL

        minW, maxW = self.width_slider.getRange()
        self.params['widthLo'] = minW
        self.params['widthHi'] = maxW

    def setBearingDist(self, bearing, dist):
        self.heading_num_l.setText(str(bearing))
        self.dist_num_l.setText(str(dist))

    def signal_handler(self, signal, frame):
        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('Sonar_GUI')
    app = QApplication(sys.argv)
    form = Sonar()
    signal.signal(signal.SIGINT, form.signal_handler)
    form.show()
    app.exec_()
