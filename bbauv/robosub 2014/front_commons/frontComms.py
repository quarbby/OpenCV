#!/usr/bin/env python

'''
Communication b/w ROS class and submodules for front camera
'''
import rospy
import actionlib
import signal
import time
import numpy as np

from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data, \
        ControllerAction, ControllerGoal, controller, depth
from bbauv_msgs.srv import set_controller

from utils.utils import Utils
import utils.config as config
from collections import deque

class FrontComms:
    
    def __init__(self, visionFilter):
        signal.signal(signal.SIGINT, self.userQuit)
        
        #Default parameters
        self.inputHeading = 0
        self.curHeading = 0
        self.gotHeading = False 
        self.retVal = 0
        self.defaultDepth = 2.0
        self.depth = self.defaultDepth
        
        # Flags 
        self.canPublish = False    #Flag for using non-publishing to ROS when testing with images 
        self.isAborted = True
        self.isKilled = False
        self.processingRate = 2
        self.processingCount = 0
        
        #Initialize vision Filter
        self.visionFilter = visionFilter
        
        #Get private params 
        self.imageTopic = rospy.get_param('~image', config.frontCamTopic)
        self.isAlone = rospy.get_param('~alone', True)
        self.onBot = rospy.get_param('~bot', True)
        self.isTest = rospy.get_param('~test', False)
        
        #Locomotion servers 
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)
        try:
            rospy.loginfo("Waiting for Locomotion Server...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(1))
        except:
            rospy.loginfo("Locomotion server timeout!")
            self.isKilled = True
        
        # Run if is alone mode
        if self.isAlone:
            setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
            setServer(forward = True, sidemove = True, heading = True, depth = True,
                      pitch = True, roll = True, topside = False, navigation = False)

            self.isAborted = False
            self.canPublish = True       
        
    def register(self):
        rospy.loginfo("Register")
        self.camSub = rospy.Subscriber(self.imageTopic, Image, self.camCallback)
        self.compassSub = rospy.Subscriber(config.compassTopic,
                                           compass_data,
                                           self.compassCallback)
        self.outPub = rospy.Publisher(config.visionFilterTopic, Image)
        self.depthSub = rospy.Subscriber("/depth", depth, self.depthCallback)

    def registerMission(self):
        rospy.loginfo("Register with mission")
        self.depthSub = rospy.Subscriber("/depth", depth, self.depthCallback)
        self.compassSub = rospy.Subscriber(config.compassTopic,
                                           compass_data,
                                           self.compassCallback)
        self.camSub = rospy.Subscriber(self.imageTopic, Image, self.camCallback)
        self.outPub = rospy.Publisher(config.visionFilterTopic, Image)     
        rospy.loginfo("Registered finish")          
        

    def searchComplete(self):
        rospy.loginfo("Sending centered request to mission planner")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=False,
                            centered=True,
                            task_complete_ctrl=controller(heading_setpoint=
                                                self.curHeading))

    def unregister(self):
        if self.camSub is not None:
            self.camSub.unregister()
        if self.compassSub is not None:
            self.compassSub.unregister()
        self.canPublish = False 
    
    def unregisterMission(self):
        if self.camSub is not None:
            self.camSub.unregister()
        self.canPublish = False 

    def camCallback(self, rosImg):
        if self.processingCount == self.processingRate:
            outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
            if self.canPublish and outImg is not None:
                try:
                    self.outPub.publish(Utils.cv2rosimg(outImg))
                except Exception, e:
                    pass
            self.processingCount = 0
        self.processingCount += 1
        '''
        if self.canPublish:
            try:
                rospy.loginfo("IN CAM")
                self.outPub.publish(rosImg)
            except Exception, e:
                pass
        '''
        # rospy.sleep(rospy.Duration(0.05))

    def depthCallback(self, data):
        self.depth = data.depth
        # rospy.loginfo("Depth callback {}".format(self.depth))
            
    def compassCallback(self, data):
        if not self.gotHeading:
            self.curHeading = data.yaw
            self.gotHeading = True
            rospy.loginfo("Heading: {}".format(self.curHeading))
    
    def userQuit(self, signal, frame):
        # if self.isAlone:
        #     self.unregister()
        # else:
        #     self.unregisterMission()
        self.isAborted = True
        self.isKilled = True
        rospy.signal_shutdown("Task manually killed")
        
    def abortMission(self):
        rospy.loginfo("Aborted :( Sorry Mission Planner...")
        if not self.isAlone:
            self.toMission(fail_request=True, task_complete_request=False,
                           task_complete_ctrl=controller(
                            heading_setpoint=self.curHeading))
            self.unregisterMission()
        else:
            self.unregister()
        self.canPublish = False
        self.isAborted = True
        self.isKilled = True
        
        rospy.loginfo("Aborted myself")
        rospy.signal_shutdown("Bye")
        
    def taskComplete(self):
        rospy.loginfo("Yay! Task Complete!")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=True,
                           task_complete_ctrl=controller(
                               heading_setpoint=self.curHeading))
            rospy.loginfo("Sent to mission")
            self.unregisterMission()
        else:
            self.unregister()

        self.canPublish = False
        self.isAborted = True
        self.isKilled = True
        # self.sendMovement(forward=0.0, sidemove=0.0)
        rospy.signal_shutdown("Bye")
    
    def sendMovement(self, forward=0.0, sidemove=0.0,
                     heading=None, depth=None,
                     timeout=0.5, blocking=False):
        
        depth = depth if depth else self.defaultDepth
        heading = heading if heading else self.curHeading
        
        goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=heading,
                              sidemove_setpoint=sidemove, depth_setpoint=depth)
        self.motionClient.send_goal(goal)
        rospy.loginfo("Moving.. f: %lf, sm: %lf, h: %lf, d: %lf", 
                      forward, sidemove, heading, depth)

        if blocking:
            self.motionClient.wait_for_result()
        else:
            self.motionClient.wait_for_result(timeout=rospy.Duration(timeout))
            