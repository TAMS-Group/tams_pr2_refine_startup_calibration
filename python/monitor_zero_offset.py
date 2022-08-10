#!/usr/bin/env python
# Copyright (c) 2022 Michael 'v4hn' Goerner

import rospy

from std_msgs.msg import Float32
from std_srvs.srv import Empty as EmptyService, EmptyResponse
from tams_pr2_refine_startup_calibration.srv import SetZeroOffset

from scipy.stats import norm

class MonitorOffset:
    def __init__(self):
        self.subscriber= rospy.Subscriber("zero_offset", Float32, self.offsetCB, queue_size = 10 )
        self.service= rospy.Service('set_zero_offset_from_mean', EmptyService, self.setZeroOffsetFromMean)
        self.offsets= []

    def offsetCB(self, msg):
        self.offsets.append(msg.data)

    def setZeroOffsetFromMean(self, req):
        if len(self.offsets) < 3:
            rospy.logerr("Cannot estimate reasonable mean from less than 3 samples")
            return EmptyResponse()
        (mu, sigma) = norm.fit(self.offsets)
        rospy.loginfo("Fit mean {} with sigma {}".format(mu, sigma))

        try:
            set_zero_offset = rospy.ServiceProxy('set_zero_offset', SetZeroOffset)
            set_zero_offset(mu)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to set zero offset: {}".format(e))
        return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('monitor_zero_offset')
    monitor = MonitorOffset()
    rospy.spin()
