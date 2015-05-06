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
	thresParams = {
		'lo': (100, 0, 125), 'hi': (230, 85, 255)
	}

	def __init__(self, comms=None):
		self.comms = comms 

	def gotFrame(self, img):
		labImg = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
		channels = cv2.split(labImg)
		labImgRGB = cv2.cvtColor(channels[2], cv2.COLOR_GRAY2BGR)

		binImg = self.morphology(cv2.inRange(labImg, 
			self.thresParams['lo'], self.thresParams['hi']))
		kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
		binImg = cv2.morphologyEx(binImg, cv2.MORPH_OPEN, kern)

		return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

	def morphology(self, img):
		kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
		return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)

	def updateParams(self):
		pass