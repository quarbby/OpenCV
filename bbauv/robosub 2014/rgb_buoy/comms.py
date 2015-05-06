#/usr/bin/env python

'''
Communication b/w ROS class and submodules
RoboSub Day 1
'''

import time

from front_commons.frontComms import FrontComms
from vision import RgbBuoyVision

from dynamic_reconfigure.server import Server as DynServer
from utils.config import rgbConfig as Config
from bbauv_msgs.msg import controller, compass_data
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = True
    isStart = False 
    
    # Vision boolean
    toBumpColor = False
    foundBuoy = False
    centroidToBump = (-1,-1)
    rectArea = None
    deltaX = 0
    
    isCentering = False 
    depthFromMission = 0

    missionStart = None
    curTime = None

    grad = None

    heading = None
    
    def __init__(self):
        FrontComms.__init__(self, RgbBuoyVision(comms=self))
        self.defaultDepth = 1.50
        # self.defaultDepth = 0.50    # For US house
        self.depthFromMission = self.defaultDepth
        
         self.dynServer = DynServer(Config, self.reconfigure)

        self.regCompass()
        
        if not self.isAlone:
            #Initialise mission planner
            rospy.loginfo("Starting /rgb/mission_to_vision")
            self.comServer = rospy.Service("/rgb/mission_to_vision",
                                           mission_to_vision, self.handle_srv)
            rospy.loginfo("Waiting for vision to mission service")
            self.toMission = rospy.ServiceProxy("/rgb/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service()   #Indefinitely waiting for timeout
        
    # Handle mission services
    def handle_srv(self, req):

        rospy.loginfo("RGB Service handled")

        if req.start_request:
            rospy.loginfo("RGB starting")
            self.isStart = True
            self.isAborted = False
            self.canPublish = True
            
            self.missionStart = time.time()

            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            self.curHeading = self.inputHeading
            self.depthFromMission = self.defaultDepth

            self.registerMission()

            rospy.loginfo("Received heading: {}".format(self.inputHeading))
            rospy.loginfo("Received depth: {}".format(self.defaultDepth))

            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=self.inputHeading))

        if req.abort_request:
            rospy.loginfo("RGB abort received")
            self.isAborted=True
            # self.isStart = False 
            self.canPublish = False
            
            self.sendMovement(forward=0.0, sidemove=0.0)
            self.unregisterMission()
            
            rospy.loginfo("Time taken: {}".format(time.time()-self.missionStart))

            rospy.loginfo("Aborted complete")
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=self.curHeading))
    
    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic reconfigure request")
        self.params = {'hsvLoThres': (config.loH, config.loS, config.loV),
                       'hsvHiThres': (config.hiH, config.hiS, config.hiV),
                       'HoughParams': (config.Hough1, config.Hough2), 
                       'minContourArea' : config.contourMinArea,
                       'minRadius': config.MinRadius}
        self.visionFilter.updateParams()
        return config

    def regCompass(self):
        self.rgbCompass = rospy.Subscriber('/euler', compass_data,
                                            self.rgbCompassCallback)

    def rgbCompassCallback(self, data):
        self.heading = data.yaw

def main():
    testCom = Comms()
