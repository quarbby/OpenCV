'''
Created on Jul 20, 2013

@author: gohew
'''
import cv2
import numpy as np

class Threshold_constants():
    DEFAULT=0
    ADAPTIVE = 1
    OTSU = 2
    
class Thresholding():
    params = {'C':0,'block':2, 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':255,'valLow':0,'valHigh':255}
    mode = 0
    channel =0
    constants = Threshold_constants()
    def __init__(self):
        '''
        Constructor
        '''
        pass
    
    def setParams(self,parameters):
        self.params = parameters
        
    def threshold_image(self,image):
        hsv_image = cv2.cvtColor(image, cv2.cv.CV_BGR2HSV)
        hsv_array = cv2.split(hsv_image)
        if self.mode == self.constants.ADAPTIVE:
            threshold_image = cv2.adaptiveThreshold(hsv_array[self.channel], 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,self.params["block"],self.params["C"])
        elif self.mode == self.constants.DEFAULT:
            COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
            COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
            threshold_image = cv2.inRange(hsv_image, COLOR_MIN, COLOR_MAX)
        elif self.mode == self.constants.OTSU:
            retval, shape_image = cv2.threshold(hsv_array[self.channel], self.params['valLow'], 255, cv2.THRESH_OTSU)
            COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
            COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
            threshold_image = cv2.inRange(hsv_image, COLOR_MIN, COLOR_MAX)
        return threshold_image
    
        
        