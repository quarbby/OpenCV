#!/usr/bin/env python

'''
Smach state machine for Torpedo
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from comms import Comms
import time
from utils.utils import Utils

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from dynamic_reconfigure.server import Server

from vision import TorpedoVision
from front_commons.frontCommsVision import FrontCommsVision as vision

# Globals 
locomotionGoal = None 

class Disengage(smach.State):
    global missionStart

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
        # self.comms.isMovingState = True
    
    def execute(self, userdata):
        self.comms.state = "DISENGAGE"

        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if self.comms.isAlone:
            self.comms.register()
            self.comms.regCompass()
            # self.comms.registerSonar()

            rospy.sleep(rospy.Duration(0.5))
            self.comms.inputHeading = self.comms.curHeading
            rospy.loginfo("Starting Torpedo")

            self.comms.sendMovement(depth=self.comms.defaultDepth,
                        heading=self.comms.curHeading,
                        blocking=True)


        self.comms.missionStart = time.time()
        self.comms.curTime = time.time()
        
        return 'start_complete'

class FollowSonar(smach.State):
    completeBoardArea = 11000
    forward_setpoint = 0.8
    # forward_setpoint = 0.3
    # sidemove_setpoint = 1.0
    sidemove_setpoint = 0.4
    sonarDist = 3.0

    timeout = 30

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['sonar_complete', 'following_sonar', 'aborted', 'killed'])
        self.comms = comms        
        rospy.sleep(duration=0.8)
        
    def execute(self, ud):
        self.comms.state = "SONARSONAR"
        self.comms.isMovingState = False

        start = time.time()
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        if (time.time()-start) > self.timeout:
            self.comms.isAborted = True
            return 'aborted'

        while self.comms.sonarRange is None and self.comms.sonarBearing is None:
            rospy.sleep(rospy.Duration(0.3))

        rospy.loginfo("Sonar bearing {}".format(self.comms.sonarBearing))
        rospy.loginfo("Sonar Dist {}".format(self.comms.sonarRange))
        if self.comms.sonarRange > self.sonarDist or \
            self.comms.boardArea > self.completeBoardArea:
            self.comms.curHeading = self.comms.heading + self.comms.sonarBearing
            # if self.comms.sonarBearing < -10 or \
            #     self.comms.sonarBearing >  10:
            sidemove = self.sidemove_setpoint if self.comms.sonarBearing > 0 else self.sidemove_setpoint*-1.0

            self.comms.sendMovement(forward=0.0,
                                    heading=self.comms.curHeading,
                                    sidemove=sidemove,
                                    blocking=False)
            rospy.loginfo("Movement side finish")
            self.comms.sendMovement(forward=self.forward_setpoint, blocking=False)

            # rospy.sleep(rospy.Duration(0.3))
            return 'sonar_complete'
            # return 'following_sonar'
        else:
            self.comms.curHeading = self.comms.heading

            rospy.loginfo("Follow Sonar time {}".format(time.time()-self.comms.curTime))
            self.comms.curTime = time.time()

            return 'sonar_complete'

        return 'following_sonar'

class SearchCircles(smach.State):
    timeout = 30
    lostCount = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchCircles_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
            
    def execute(self, ud):
        self.comms.state = "SEARCHING"
        self.comms.isMovingState = False
        # self.comms.isMovingState = True

        start = time.time()
        curTime = time.time()
        
        while not self.comms.foundCircles and not self.comms.foundSomething: 
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 

            if not self.comms.foundCircles and not self.comms.foundSomething:
                self.lostCount += 1
            if self.lostCount > 90:
                self.comms.abortMission()
                return 'lost'

            self.comms.sendMovement(forward=0.3, timeout=0.6, blocking=False)

            rospy.sleep(rospy.Duration(0.3))   

        rospy.loginfo("Search circles time {}".format(time.time()-self.comms.curTime))
        self.comms.curTime = time.time()
        return 'searchCircles_complete'

# Align with the center of the green board 
class AlignBoard(smach.State):
    timeout = 30

    forward_setpoint = 0.50
    # forward_setpoint = 0.30     # US Small pool
    halfCompleteArea = 49000
    # completeArea = 68000
    completeArea = 63000        # US Small pool
    # completeArea = 58000         # US House 2

    # deltaXMult = 4.9
    deltaXMult = 3.00   # US Small Pool
    deltaYMult = 1.6
    # deltaYMult = 0.5    # US Small Pool
    headingMult = 1.5

    skew = 1.00

    lostCount = 0

    # completeArea = self.comms.movementParams['boardArea']
    # deltaXMult = self.comms.movementParams['alignDeltaX']
    # deltaYMult = self.comms.movementParams['alignDeltaY']

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['alignBoard_complete', 'aligning_board', 'aborted', 'killed'])
        self.comms = comms

    def execute(self, ud):
        self.comms.state = "BOARDBOARD"
        self.comms.isMovingState = False
        self.comms.isCenteringState = False
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        if not self.comms.foundCircles or not self.comms.foundSomething:
            self.comms.sendMovement(forward=0.3, heading=self.comms.curHeading,
                                    depth=self.comms.defaultDepth, timeout=0.4,
                                    blocking=False)
            self.lostCount += 1
        if self.lostCount > 90:
            self.comms.abortMission()
            return 'aborted'

        #if self.comms.boardArea > self.completeArea and self.comms.skew < 0.35:
        #            self.comms.curHeading = self.comms.heading

        if self.comms.boardArea > self.completeArea:
            rospy.loginfo("Align Board time {}".format(time.time()-self.comms.curTime))
            self.comms.curTime = time.time()
            return 'alignBoard_complete'

        if self.comms.boardArea < self.halfCompleteArea:
            self.forward_setpoint = 1.5
            # self.forward_setpoint = 0.8     # US Small pool 1
        else:
            self.forward_setpoint = 0.50      # Tech tank
            # self.forward_setpoint = 0.30    # US House 2
        # rospy.loginfo("Depth {}".format(self.comms.depth))

        if abs(self.comms.boardDeltaY) > 0.035:
            # if self.comms.boardArea > self.halfCompleteArea:
            #     self.deltaYMult = 0.2
            #     self.deltaXMult = 3.0

            self.comms.defaultDepth = self.comms.depth + self.comms.boardDeltaY*self.deltaYMult
            # Prevent surfacing
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission
        # rospy.loginfo("Default d {}".format(self.comms.defaultDepth))

        # if self.comms.boardArea > self.halfCompleteArea:
        #     self.forward_setpoint = 0.30

        # if abs(self.comms.skew) < self.skew:
        #     self.comms.curHeading = self.comms.heading + abs(self.comms.skew) * self.headingMult
                
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.boardDeltaX * self.deltaXMult, 
                                heading=self.comms.curHeading, 
                                depth=self.comms.defaultDepth, timeout=0.4, 
                                blocking=False)

        return 'aligning_board'

# Just move forward 
class MoveForward(smach.State):

    # forward_setpoint = 0.43
    forward_setpoint = 0.28     # US Small pool
    completeRadius = 62
    # completeRadius = 60         # US Small pool
    completeRadius = 50           # US tech tank
    lostCount = 0

    # deltaXMult = 4.5
    deltaXMult = 2.5           # US Small pool
    # deltaXMult = 2.0
    deltaYMult = 2.0
    deltaYMult = 1.6          # US Small pool
    # deltaYMult = 0.5

    # completeRadius = self.comms.movementParams['forwardRadius']
    # deltaXMult = self.comms.movementParams['forwardDeltaX']
    # deltaYMult = self.comms.movementParams['forwardDeltaY']
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):    
        self.comms.state = "MOVING"
        self.comms.isMovingState = True
        self.comms.isCenteringState = False
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted' 
        
        if not self.comms.foundCircles or not self.comms.foundSomething:
            self.lostCount += 1
        if self.lostCount > 90:
            self.comms.abortMission()
            return 'lost'
        
        if not self.comms.foundCircles:
            self.lostCount = self.lostCount + 1
                    
        if self.comms.numShoot == 1:
            # self.completeRadius = 40        # US Tech Tank
            self.completeRadius = 45
            self.forward_setpoint = 0.13
            # self.forward_setpoint = 0.10
            # self.deltaXMult = 0.8

        if self.comms.radius > self.completeRadius-10:
            self.comms.isCenteringState = True

        if self.comms.radius > self.completeRadius:
            rospy.loginfo("Move circles time {}".format(time.time()-self.comms.curTime))
            self.comms.curTime = time.time()

            if not self.comms.isAlone:
                self.comms.searchComplete()
                rospy.loginfo("Sent to mission")
            return 'forward_complete'
        
        if abs(self.comms.deltaY) > 0.040:
            self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*self.deltaYMult
            # Prevent surfacing
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission
        
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.deltaX * self.deltaXMult, 
                                depth=self.comms.defaultDepth, timeout=0.4, 
                                blocking=False)
        
        return 'forwarding'

# Precise movements when near centroid
class Centering (smach.State):
    deltaXMult = 2.0    # US Small pool
    # deltaYMult = 2.3
    # deltaYMult = 2.0
    # deltaYMult = 1.5    # US Small pool
    # deltaYMult = 1.3
    deltaYMult = 0.6     # US House 2

    centeringCount = 0
    halfCompleteRadius = 65
    # completeRadius = 91
    completeRadius = 88     # US Small pool & Tech Tank
    # completeRadius = 83       # US house 2
    forward_half = 0.23
    # forward_half = 0.18
    # forward_setpoint = 0.22
    forward_setpoint = 0.18    # US Small pool
    # forward_setpoint = 0.10      # US House 2

    centering = 5

    # deltaXMult = self.comms.movementParams['centerDeltaX']
    # deltaYMult = self.comms.movementParams['centerDeltaY']
    # completeRadius = self.comms.movementParams['completeRadius']

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.state = "CENTERING"
        self.comms.isMovingState = True
        self.comms.isCenteringState = True
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        if self.comms.numShoot == 1:
            # self.completeRadius = 86
            # self.completeRadius = 74    # US Tech Tank
            self.completeRadius = 80  # US House 1
            self.forward_setpoint = 0.10
            self.deltaYMult = 1.0
            self.centering = 7

        if self.comms.radius > self.completeRadius:
            sidemove_setpoint = self.comms.deltaX * 1.3
            if self.comms.numShoot == 1:
                self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*1.0
            elif self.comms.numShoot == 0:
                self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*0.8
                # self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*0.6

            self.comms.sendMovement(forward = 0.0, 
                            sidemove = sidemove_setpoint,
                            depth = self.comms.defaultDepth, timeout=0.4, 
                            blocking = False)

            if abs(self.comms.deltaX) < 0.07 and abs(self.comms.deltaY) < 0.07:
                self.centeringCount += 1

            # if self.comms.numShoot == 1:
            #     self.centering = 5
            if self.centeringCount > self.centering:
                self.centeringCount = 0

                # Just move forward a little
                # if self.comms.numShoot == 1:
                #     self.comms.sendMovement(forward=0.4, sidemove=0.0,
                #             depth=self.comms.defaultDepth, blocking=True)
                # else:
                #     self.comms.sendMovement(forward=0.4, sidemove=0.0,
                #                             depth=self.comms.defaultDepth, blocking=True)

                    # self.comms.sendMovement(forward=0.6, sidemove=0.0,
                    #                         depth=self.comms.defaultDepth, blocking=True)

                rospy.loginfo("Centering time {}".format(time.time()-self.comms.curTime))
                self.comms.curTime = time.time()
                return 'centering_complete'
        else: 
            # heading = 0.0
            # if abs(self.comms.angleFromCenter) < 30:
            #     heading = self.heading + self.comms.angleFromCenter * self.headingMult

            if abs(self.comms.deltaY) > 0.040:
                self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*self.deltaYMult
            if self.comms.defaultDepth < 0.2:
                # self.comms.defaultDepth = self.comms.depthFromMission
                self.comms.defaultDepth = 0.2
            
            sidemove_setpoint = 0.0
            if abs(self.comms.deltaX) > 0.040:
                sidemove_setpoint = self.comms.deltaX * self.deltaXMult

            # Sidemove and center
            self.comms.sendMovement(forward = self.forward_setpoint,
                                    sidemove = sidemove_setpoint,
                                    depth = self.comms.defaultDepth, timeout=0.4,
                                    blocking = False)
        return 'centering'        

# Shoot torpedo forward
class ShootTorpedo(smach.State):

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_again', 'shoot_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.state = "SHOOTING"
        self.comms.isMovingState = True
        self.comms.isCenteringState = True

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        if self.comms.numShoot == 0:
            rospy.loginfo("First shoot")  

            self.comms.shootTopTorpedo()
            rospy.loginfo("Big circle {}".format(time.time()-self.comms.missionStart))

            # if not self.comms.isAlone:
            #     self.comms.searchComplete()

            # self.comms.shootBotTorpedo()
            # For US House 2
            # self.comms.taskComplete()
            # return 'shoot_complete'

        elif self.comms.numShoot == 1:
            rospy.loginfo("Second shoot")
            self.comms.shootBotTorpedo()
        
        # Reset parameters
        self.comms.centroidToShoot = (-1, -1)
        self.comms.numShoot = self.comms.numShoot + 1

        if self.comms.numShoot == 2:
            self.comms.taskComplete()
            rospy.loginfo("Completed task {}".format(time.time()-self.comms.missionStart))
            return 'shoot_complete'

        self.comms.state = "MOVING UP"
        # self.comms.defaultDepth = self.comms.defaultDepth - 0.80
        self.comms.defaultDepth = 2.0     #RoboSub Day 1
        if self.comms.defaultDepth < 0.2:
            self.comms.defaultDepth = 0.2
        self.comms.sendMovement(forward=-1.0, sidemove=0.0,
                                depth=self.comms.defaultDepth,
                                blocking=True)
       
        return 'shoot_again'

def main():
    rospy.init_node('torpedo_bangbangbang', anonymous=False)
    rosRate = rospy.Rate(30)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'killed', 'failed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={
                                         'start_complete': "ALIGNBOARD",
                                         'killed': 'failed'})      
        
        smach.StateMachine.add("FOLLOWSONAR", FollowSonar(myCom),
                               transitions={'following_sonar': "FOLLOWSONAR",
                                            'sonar_complete': 'succeeded',
                                           # 'sonar_complete': "ALIGNBOARD",
                                           'aborted': 'killed',
                                           'killed': 'failed'})

        smach.StateMachine.add("ALIGNBOARD", AlignBoard(myCom),
                                transitions={'aligning_board': "ALIGNBOARD",
                                            'alignBoard_complete': "MOVEFORWARD",
                                            'aborted': 'failed', 
                                            'killed': 'failed'
                                })
    
        smach.StateMachine.add("SEARCHCIRCLES", SearchCircles(myCom),
                                transitions={'searchCircles_complete': "ALIGNBOARD",
                                             'lost': 'failed',
                                             'aborted': 'killed',
                                             'killed': 'failed'})      
        
        smach.StateMachine.add("MOVEFORWARD", MoveForward(myCom),
                                transitions={'forwarding': "MOVEFORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': 'failed',
                                             'aborted': 'killed',
                                             'killed': 'failed'})       
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "SHOOTING",
                                             'lost': 'failed',
                                             'aborted': 'killed',
                                             'killed': 'failed'})  
        
        smach.StateMachine.add("SHOOTING", ShootTorpedo(myCom),
                                transitions={'shoot_again': "SEARCHCIRCLES",
                                             'shoot_complete': 'succeeded',
                                             'aborted': 'killed',
                                             'killed': 'failed'})     
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    try:
        sm.execute()
    except Exception as e:
        rospy.loginfo(str(e))
    finally:
        rospy.signal_shutdown("Bye")
        
