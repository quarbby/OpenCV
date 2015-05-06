#!/usr/bin/env python

'''
Buoy states
'''

import roslib; roslib.load_manifest('vision')
import rospy

import time
import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from vision import RgbBuoyVision

from dynamic_reconfigure.server import Server
from front_commons.frontCommsVision import FrontCommsVision as vision

#Globals
locomotionGoal = None

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed', 'start'])
        
        self.comms = comms
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))

        if self.comms.isAlone:
            self.comms.register()
            rospy.sleep(rospy.Duration(0.8))
            self.comms.inputHeading = self.comms.curHeading

            self.comms.missionStart = time.time()
        self.comms.sendMovement(depth=self.comms.defaultDepth,
                                heading=self.comms.inputHeading,
                                blocking=True)

        return 'start_complete'
    
class Search(smach.State):
    timeout = 300
    moveOnce = 0
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        while not self.comms.foundBuoy and not self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search pattern
            self.comms.sendMovement(forward=0.40, timeout=0.6, blocking=False)
#             if self.moveOnce < 10:
#                 self.comms.sendMovement(forward=0.2, sidemove=0.2, blocking=False)
#             else:
#                 self.comms.sendMovement(forward=0.2, sidemove=-0.2, blocking=False)        
            
            rospy.sleep(rospy.Duration(0.3))
        
        return 'search_complete'

# I'm just moving forward
class bangBuoy(smach.State):
    deltaXMult = 4.0
    deltaYMult = 1.3
    area = 8800
    count = 0
    forward_setpoint = 0.50

    gradMult = 2.0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['banging', 'bang_to_center', 'aborted', 'killed'])
        self.comms = comms
        self.curHits = 0
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        if self.count > 3:           
            return 'bang_to_center'
        
        if self.comms.rectArea > self.area: 
            self.count = self.count + 1
  
        # if abs(self.comms.grad) > 30:
        #     self.comms.curHeading = self.comms.heading + self.comms.grad*self.gradMult

        if abs(self.comms.deltaY) > 0.10:
            # if self.comms.rectArea > 5000:
            #     self.deltaYMult = 1.00
            self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*self.deltaYMult
            # Make sure it doesnt surface
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission
  
        if self.comms.rectArea < 5000:
            self.forward_setpoint = 1.2
        else:
            self.forward_setpoint = 0.5

        # Move forward & correct heading 
        self.comms.sendMovement(forward=self.forward_setpoint, sidemove=self.comms.deltaX*self.deltaXMult,
                                depth=self.comms.defaultDepth,
                                # heading=self.comms.curHeading,
                                blocking=False)
        return 'banging'

# Precise movements when near buoy 
class Centering (smach.State):
    deltaXMult = 2.3
    deltaYMult = 1.0
    depthCount = 0
    count = 0
    depthCorrected = False 
    
    bigArea = 12000
    changeMultArea = 95000
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.isCentering = True

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        if self.comms.rectArea > self.changeMultArea:
            self.deltaXMult = 1.6
            self.deltaYMult = 0.5

        if self.comms.rectArea > self.bigArea:
            if self.comms.deltaX < 0.06 and self.comms.deltaY < 0.06:
                self.count += 1

            if self.count > 3:
                self.comms.sendMovement(forward=2.3, timeout=4, blocking=False)   # Shoot forward
                rospy.loginfo("forward done")
                
                # self.comms.sendMovement(forward=-0.5, depth=self.comms.defaultDepth-0.5,
                #             timeout=3, blocking=False)  # Reverse a bit
                self.comms.isAborted = True
                self.comms.isKilled = True 

                rospy.loginfo("Time taken: {}".format(time.time()-self.comms.missionStart))

                self.comms.taskComplete()
                return 'centering_complete'
        
        if abs(self.comms.deltaY) > 0.050:           
            self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*self.deltaYMult
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission

        self.comms.sendMovement(forward=0.25,
                                sidemove=self.comms.deltaX*self.deltaXMult, 
                                depth=self.comms.defaultDepth, 
                                timeout=0.4, blocking=False)

        return 'centering'

def main():
    rospy.init_node('rgb_buoy_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("RGB Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'failed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'start': "DISENGAGE",
                                         'killed': 'failed'})
        
        smach.StateMachine.add("SEARCH", Search(myCom),
                               transitions={'search_complete': "BANGBUOY",
                                            'aborted': 'aborted', 
                                            'killed': 'failed'})
    
        smach.StateMachine.add("CENTERING", Centering(myCom),
                               transitions={'centering': "CENTERING",
                                            'centering_complete': 'succeeded',
                                            'aborted': 'aborted',
                                            'killed': 'failed'})
        
        smach.StateMachine.add("BANGBUOY", bangBuoy(myCom),
                               transitions={'banging': "BANGBUOY",
                                            'bang_to_center': "CENTERING",
                                            'aborted': 'aborted',
                                            'killed': 'failed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/RGB_BUOY')
    introServer.start()
    
    sm.execute()
