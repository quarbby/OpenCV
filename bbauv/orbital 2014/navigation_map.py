'''Draw navigation map'''

#!/usr/bin/env python

import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
from PyQt4 import QtCore, QtGui, QtSvg

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt

import rospy
import roslib
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import *

import sys
import os
import random
import math

class Navigation_Map(QWidget):
    
    def __init__(self):
        super(Navigation_Map, self).__init__()
        self.data = []
        self.points = []
        self.rate = 20
        self.earth_odom_sub = None
#         self.receiveData()
#         self.initSub()
        self.initUI()
    
    def initUI(self):
        map_label = QLabel("<b>Map of the World</b>")
        refresh_button = QPushButton("&Refresh")
        clear_button = QPushButton("&Clear")
        
        map_layout = QHBoxLayout()
        map_layout.addWidget(map_label)
        map_layout.addWidget(refresh_button)
        map_layout.addWidget(clear_button)
        map_layout.addStretch()
        
        refresh_button.clicked.connect(self.refreshBtnHandler)
        clear_button.clicked.connect(self.clearGraph)
        
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
#         self.toolbar = NavigationToolbar(self.canvas, self)
        self.refreshBtnHandler()
                
        main_layout = QVBoxLayout()
        main_layout.addLayout(map_layout)
#         main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas)
        
        self.initTimer(self.rate)
        self.setLayout(main_layout)
        self.show()
        rospy.loginfo("Navigation map up")
    
    def initSub(self):
        self.earth_odom_sub = rospy.Subscriber('/earth_odom', Odometry, self.earth_odom_callback)
    
    def unregisterSub(self):
        self.earth_odom_sub.unregister()
        
    def receiveData(self, x, y):
        if len(self.data) == 0:
            self.data.append((x, y))
        else:
            lastPoint = self.data[len(self.data)-1]
            distance = abs(math.sqrt( (x-lastPoint[0])**2 + (y-lastPoint[0])**2))
            if ( distance > 1.30 ):
                self.data.append((x,y))
#         if len(self.points) == 30:
#             self.data.append(self.points[0])
#             self.points = []
#         self.data.append((data.pose.pose.position.x, data.pose.pose.position.y))
    
    def earth_odom_callback(self, data):
        (x,y) = (data.pose.pose.position.x, data.pose.pose.position.y)
        if len(self.data) == 0:
            self.data.append((data.pose.pose.position.x, data.pose.pose.position.y))
        else:
            lastPoint = self.data[len(self.data)-1]
            distance = abs(math.sqrt( (x-lastPoint[0])**2 + (y-lastPoint[0])**2))
            if ( distance > 1.30 ):
                self.data.append((x,y))
#         if len(self.points) == 30:
#             self.data.append(self.points[0])
#             self.points = []
#         self.data.append((data.pose.pose.position.x, data.pose.pose.position.y))
    
    def initTimer(self, time):
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.refreshBtnHandler)
        self.timer.start(800.0 / time)
    
    def clearGraph(self):
        self.data = []
            
    def refreshBtnHandler(self):
#         for i in range(10):
#              self.data.append((random.random(), random.random()))
#         print self.data
# 
        axis = self.figure.add_subplot(111)

        # discards the old graph
        axis.hold(False)


        try:
            axis.plot([x[0] for x in self.data], [x[1] for x in self.data], '*-')
        except:
            pass
        
        self.canvas.draw()
