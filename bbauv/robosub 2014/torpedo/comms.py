#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import roslib; roslib.load_manifest('vision')
import rospy
from front_commons.frontComms import FrontComms
from vision import TorpedoVision 
# from sonarExp import SonarFilter

from bbauv_msgs.msg import controller, manipulator, compass_data, depth
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission
        
from dynamic_reconfigure.server import Server as DynServer
from utils.config import torpedoConfig as Config

from sensor_msgs.msg import Image
from utils.utils import Utils
import time

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = True
    isStart = False
    
    # Circle booleans
    foundCircles = False 
    foundSomething = False 
    lostCount = 0
    
    # Shooting parameters
    numShoot = 0    # Only given 2 shoots 
    centroidToShoot = (-1, -1)
    torpedoOffset = 0.11 

    # Board parameters
    boardCentroid = (-1, -1)
    boardArea = 0
    boardDeltaX = 0
    boardDeltaY = 0

    lockedCentroid = (-1, -1)
    isMovingState = False
    # isMovingState = True
    # isCenteringState = False
    
    # Movement parameters
    radius = 0
    deltaX = 0
    deltaY = 0
    skew = 0
    direction = None
    heading = 0.0
    angleFromCenter = 0.0

    sonarRange = None
    sonarBearing = None

    state = None

    missionStart = None
    curTime = None

    def __init__(self):
        FrontComms.__init__(self, TorpedoVision(comms=self))
        self.defaultDepth = 2.5
        self.depthFromMission = self.defaultDepth

         self.sonarFilter = SonarFilter(comms=self) 

         self.dynServer = DynServer(Config, self.reconfigure)
         self.depthSub = rospy.Subscriber('/depth', depth, self.depthCallback)
        self.maniPub = rospy.Publisher("/manipulators", manipulator, latch=True)

        # Initialise mission planner 
        if not self.isAlone:
            self.comServer = rospy.Service("/torpedo/mission_to_vision", 
                                           mission_to_vision,
                                           self.handle_srv)
            rospy.loginfo("Waiting for mission planner")
            self.toMission = rospy.ServiceProxy("/torpedo/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service()   #Indefinitely waiting for timeout

    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
        
        rospy.loginfo("Torpedo Service handled")
        
        if req.start_request:
            rospy.loginfo("Torpedo starting")
            self.isStart = True
            self.isAborted = False
            self.canPublish = True

            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            self.curHeading = self.inputHeading
            self.depthFromMission = self.defaultDepth

            rospy.loginfo("Received depth: {}".format(self.defaultDepth))
            rospy.loginfo("Received heading: {}".format(self.inputHeading))

            self.registerMission()
            self.regCompass()
            # self.registerSonar()
            
            # self.curHeading = self.curHeading - 10.0
            # self.sendMovement(forward=0.0, sidemove=0.0,
            #                     heading=self.curHeading, blocking=True)

            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        
        elif req.abort_request:
            rospy.loginfo("Torpedo abort received")
            self.isAborted=True
            # self.isStart = False
            self.canPublish = False 

            self.unregisterMission()

            # Randomly shoot stuff
            if self.numShoot == 0:
                self.shootTopTorpedo()
            else:
                self.shootBotTorpedo()

            self.sendMovement(forward=0.0, sidemove=0.0)
            rospy.loginfo("Aborted complete")

            rospy.loginfo("Time taken: {}".format(time.time()-self.missionStart))

            # if self.isMovingState:
            #     return mission_to_visionResponse(start_response=False,
            #                                     abort_response=True,
            #                                     search_request=True,
            #                                     data=controller(heading_setpoint=
            #                                                     self.curHeading))
            # else:
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        

    def regCompass(self):
        self.torCompass = rospy.Subscriber('/euler',
                                           compass_data,
                                           self.torCompassCallback)
    def torCompassCallback(self, data):
        self.heading = data.yaw

    def unRegCompass(self):
        self.torCompass.unregister()

    def shootTopTorpedo(self):
        self.maniPub.publish(1)
        self.maniPub.publish(1)
        self.maniPub.publish(1)
        rospy.loginfo("Firing top torpedo")
        rospy.sleep(rospy.Duration(0.3)) 

    def shootBotTorpedo(self):
        # maniPub = rospy.Publisher("/manipulators", manipulator, latch=True)
        self.maniPub.publish(2)
        self.maniPub.publish(2)
        rospy.loginfo("Firing bottom torpedo")
        rospy.sleep(rospy.Duration(0.3))       
    
    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic reconfigure request")
        self.params = {
                       # 'loV': config.loV,
                       # 'hiV': config.hiV,
                       # 'loS': config.loS,
                       # 'hiS': config.hiS,
                       # 'loH': config.loH,
                       # 'hiH': config.hiH,

                       'loV_b': config.loV_b,                       
                       'hiV_b': config.hiV_b,
                       'loS_b': config.loS_b,
                       'hiS_b': config.hiS_b,
                       'loH_b': config.loH_b,
                       'hiH_b': config.hiH_b,
                       # 'hiThreshold': (config.hiH, config.hiS, config.hiV),
                       # 'blackLoThreshold': [config.loH_b, config.loS_b, config.loV_b],
                       # 'blackHiThreshold': [config.hiH_b, config.hiS_b, config.hiV_b],
                       # 'sonarOffset': config.sonarOffset,
                       'torpedoOffset': config.torpedoOffset, 
                       'minContourArea': config.minContourArea,
                       'skewLimit': config.skewLimit, 
                       'sonarThres': config.sonarThres
                       }
        self.movementParams = {
                       'completeRadius': config.completeRadius,
                       'forwardRadius': config.forwardRadius,
                       'boardArea': config.boardArea,
                       'alignDeltaX': config.alignDeltaX,
                       'alignDeltaY': config.alignDeltaY,
                       'alignHeading': config.alignHeading,
                       'forwardDeltaX': config.forwardDeltaX,
                       'forwardDeltaY': config.forwardDeltaY,
                       'centerDeltaX': config.centerDeltaX,
                       'centerDeltaY': config.centerDeltaY
        }

        # self.sonarParams = {'threshold': config.sonarThres,
        #         'lenLo': config.lenLo, 'lenHi': config.lenHi,
        #         'widthLo': config.widthLo, 'widthHi': config.widthHi,
        #         'thresOffset': config.sonarThresOffset
        # }

        self.visionFilter.updateParams()
        # self.sonarFilter.reconfigure()

        return config
    
    def registerSonar(self):
        self.sonarFilter.registerNode()
    
    def sonarImageCallback(self, rosImg):
        # outImg = self.visionFilter.gotSonarFrame(Utils.rosimg2cv(rosImg))
        outImg = self.sonarFilter.gotSonarFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish and outImg is not None:
            try:
                self.sonarPub.publish(Utils.cv2rosimg(outImg))
            except Exception, e:
                pass
                
        # rospy.sleep(rospy.Duration(0.1))

    # def depthCallback(self, data):
    #     self.depth = data.depth
        
    def unregisterSonar(self):
        self.sonarSub.unregister()
    
def main():
    pass
