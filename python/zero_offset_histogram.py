#!/usr/bin/env python

# adapted from https://github.com/raultron/online_histogram
# Licensed as MIT License
# Copyright (c) 2016 Raul Acuna
# Copyright (c) 2022 Michael 'v4hn' Goerner

from PlotWindow import PlotWindow

import rospy

from std_msgs.msg import Int8, Float32
from std_srvs.srv import Empty as EmptyService, EmptyResponse

import sys, random
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication

import numpy
import numpy as np

from matplotlib.ticker import MaxNLocator
from scipy.stats import norm
from collections import deque


import math

class OnlineHist(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=1000
    self.values= deque(maxlen=self.window_size)  #numpy.zeros((self.window_size))
    self.index=0
    self.paused = False

    rospy.init_node('zero_offset_histogram')
    self.subscriber = rospy.Subscriber("~zero_offset", Float32, self.plotResults, queue_size = 1 )
    
    self.pauseButton.clicked.connect(self.pauseClicked)
    self.resetButton.clicked.connect(self.resetClicked)
    self.srv = rospy.Service('~reset', EmptyService, self.resetCalled)

  def pauseClicked(self):
    if self.paused:        
       self.paused = False
    else:
       self.paused = True
  
  def resetCalled(self, req):
    self.resetClicked()
    return EmptyResponse()

  def resetClicked(self):
    self.values.clear()
    self.axes.clear()
    self.canvas.draw()
    self.index=0       
    self.paused = False
    return


  def plotResults(self, data):       
    #self.axes.set_autoscaley_on(True)

    if self.index==self.window_size-1:
      self.index=0
    else:
      self.index=self.index+1
    self.values.append(data.data)

    if not self.paused:
        self.axes.clear()
        #b= np.unique(np.sort(np.round(self.values, decimals=6)))
        b= 100
        n, bins, patches = self.axes.hist(list(self.values), bins = b, facecolor='green', alpha=0.75, align='left')

        #I want to also fit the data to a Gaussian
        # best fit of data
        (mu, sigma) = norm.fit(list(self.values))
        # add a 'best fit' line
        y = norm.pdf( bins, mu, sigma) / norm.pdf(mu, mu, sigma) * np.max(n)
        l = self.axes.plot(bins, y, 'r--', linewidth=2)

        #self.axes.set_xticks(bins[:-1])
        self.axes.set_title(r'$\mathrm{Histogram\ of\ Range:}\ \mu=%.3f,\ \sigma=%.3f$' %(mu, sigma))
        self.axes.set_xlabel("Value")
        self.axes.set_ylabel("Frequency")
        output= "Data Size: "+str(len(self.values))
        min_x, max_x=self.axes.get_xlim()
        min_y, max_y=self.axes.get_ylim()
        #max_x*0.5,max_y*0.5,output,horizontalalignment='left',verticalalignment='center')        
        self.axes.annotate(output, (0.05,0.9), xycoords = 'axes fraction') 
        self.axes.yaxis.set_major_locator(MaxNLocator(integer=True))
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineHist()
    window.show()
    app.exec_()
