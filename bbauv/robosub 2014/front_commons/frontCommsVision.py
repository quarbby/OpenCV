'''
Common methods for front camera vision
'''

import cv2
import math
import numpy

class FrontCommsVision():
    screen = { 'width': 640, 'height': 480 }
    minContourArea = 5000
    
    # For morphological operations 
    @staticmethod
    def erodeAndDilateImg(image, params):
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['erode'])
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['dilate'])
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['open'])

        image = cv2.erode(image, erodeEl)
        image = cv2.dilate(image, dilateEl)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, openEl)

        return image
    
    # For preprocessing 
    @staticmethod
    def preprocessImg(image):
        # Cut out reflection
#         image = image[FrontCommsVision.screen['height']/4:(FrontCommsVision.screen['height'])*3/4,
#                       0:FrontCommsVision.screen['width'],:]
        image = cv2.resize(image, (FrontCommsVision.screen['width'], 
                                   FrontCommsVision.screen['height']))
#         enhancedImg = cv2.GaussianBlur(image, ksize=(0, 0), sigmaX=10)
#         enhancedImg = cv2.addWeighted(image, 2.5, enhancedImg, -1.5, 0)
        return image 
    
    # Contour finding and sorting
    @staticmethod
    def findAndSortContours(image):
        contours, hierachy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = filter(lambda c: cv2.contourArea(c) > FrontCommsVision.minContourArea, contours)
        sorted(contours, key=cv2.contourArea, reverse=True)
        return contours            
    
    # Draw a rect at center of screen
    @staticmethod
    def drawCenterRect(image):
        midX = FrontCommsVision.screen['width']/2.0
        midY = FrontCommsVision.screen['height']/2.0
        maxDeltaX = FrontCommsVision.screen['width']*0.05
        maxDeltaY = FrontCommsVision.screen['height']*0.05
        cv2.rectangle(image,
                      (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                      (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                      (0, 255, 0), 2)  
        return image 
    
    # Check if centroid in center RECT
    @staticmethod
    def centroidInCenterRect(centroidX, centroidY):
        centroidX = abs(centroidX)
        centroidY = abs(centroidY)
        
        midX = FrontCommsVision.screen['width']/2.0
        midY = FrontCommsVision.screen['height']/2.0

        maxDeltaX = FrontCommsVision.screen['width']*0.15
        maxDeltaY = FrontCommsVision.screen['height']*0.15

        if centroidX > (midX-maxDeltaX) and centroidX < (midX + maxDeltaX) and \
            centroidY > (midY-maxDeltaY) and centroidY < (midY + maxDeltaY):
            return True
        else:
            return False       
    
    # Normalise image
    @staticmethod
    def normaliseImg(img):
        channel = cv2.split(img)
        cv2.normalize(channel[1], channel[1], 0, 255, cv2.NORM_MINMAX)
        cv2.normalize(channel[2], channel[2], 0, 255, cv2.NORM_MINMAX)
        return cv2.merge(channel, img)

    # Capult from Jin
    @staticmethod
    def whiteBal(img):
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        channels[1] = cv2.equalizeHist(channels[1])
        img = cv2.merge(channels, img)
        img = cv2.bilateralFilter(img, -1, 5, 0.1)
        
        # Morphological operations
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
    
    @staticmethod
    def shadesOfGray(img):
        """ Implementation of the Shades of Gray algorithm, which is a
        combination of Gray World algorithm and MaxRGB algorithm, which are
        used to normalize color images to achieve color constancy """

        inB, inG, inR = cv2.split(img)
        avgR = numpy.mean(inR)
        avgG = numpy.mean(inG)
        avgB = numpy.mean(inB)
        avgGray = numpy.mean((avgB, avgG, avgR))

        if avgB == 0:
            outB = inB
        else:
            outB = (avgGray/avgB)*inB

        if avgG == 0:
            outG = inG
        else:
            outG = (avgGray/avgG)*inG

        if avgR == 0:
            outR = inR
        else:
            outR = (avgGray/avgR)*inR

        maxRGB = (numpy.max(outR), numpy.max(outG), numpy.max(outB))
        factor = numpy.max(maxRGB)
        if factor > 1:
            outR = 255*outR/factor
            outG = 255*outG/factor
            outB = 255*outB/factor

        outImg = cv2.merge((numpy.uint8(outB), numpy.uint8(outG), numpy.uint8(outR)))
        return outImg

    @staticmethod
    def illuminanceMask(img):
        ''' Trying to remove illuminance '''
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayImg = cv2.equalizeHist(grayImg)
        return cv2.threshold(grayImg, 200, 255, cv2.THRESH_BINARY)[1]
