#!/usr/bin/env python

# adapted from https://github.com/raultron/online_histogram
# Licensed as MIT License
# Copyright (c) 2016 Raul Acuna
# Copyright (c) 2022-23 Michael 'v4hn' Goerner

from PlotWindow import PlotWindow

import rospy

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty as EmptyService, EmptyResponse

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication

import numpy as np

import seaborn as sns
import pandas as pd
from matplotlib.lines import Line2D

from scipy.stats import norm

class OnlinePlot(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)
    self.axes.grid(False)
    self.axes.get_yaxis().set_visible(False)

    self.paused = False

    rospy.init_node('zero_offset_histogram')
    self.subscriber = rospy.Subscriber("refine_startup_calibration/zero_offsets", Float32MultiArray, self.plotResults, queue_size = 1)
    
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
    self.axes.clear()
    self.canvas.draw()
    self.paused = False
    return

  def plotResults(self, data):       
    if not self.paused:
        self.axes.clear()
        df = pd.DataFrame(data.data, columns=['value'])
        (mu, sigma) = norm.fit(df['value'])
        df['sigma_distance'] = abs(df['value'] - mu)
        df['outlier'] = (df['sigma_distance'] > 2*sigma).astype('category').cat.rename_categories({True: 'outlier', False: 'inlier'})

        df['dummy'] = ''
        sns.stripplot(data= df, x= 'value', y= 'dummy', hue= 'outlier', jitter=True, ax= self.axes, cmap='coolwarm')
        self.axes.get_yaxis().set_visible(False)
        # self.axes.legend().set_visible(False)

        self.axes.axvline(x= mu, color='r')

        if sigma > 0:
          self.axes.axvline(x=mu + sigma, color='orange', linestyle='--')
          self.axes.axvline(x=mu - sigma, color='orange', linestyle='--')
        if sigma > 0:
          (mu2, sigma2) = norm.fit(df[df['outlier'] == 'inlier']['value'])
          self.axes.axvline(x=mu2, color='g')

        # set legend to explain red, orange and green lines
        legend_elements = [Line2D([0], [0], color='r', lw=1, label='mean'),
                   Line2D([0], [0], color='orange', lw=1, linestyle='--', label='sigma'),
                   Line2D([0], [0], color='g', lw=1, label='mean (inliers)')]
        self.axes.legend(handles=legend_elements, loc='upper right')

        self.axes.set_title(r'$\mathrm{StripPlot\ of\ Range:}\ \mu=%.3f,\ \sigma=%.3f$' % (mu, sigma))
        self.axes.set_xlabel("Value")
        output = "Data Size: " + str(len(df))

        self.axes.annotate(output, (0.05,0.9), xycoords = 'axes fraction') 
        # self.axes.yaxis.set_major_locator(MaxNLocator(integer=True))
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlinePlot()
    window.show()
    app.exec_()
