#!/usr/bin/env python3
PKG = 'pyx4'
NAME = 'pyx4_test'

import sys 
import time
import unittest
import os
import csv

import numpy as np

import rospy
import rostest
import roslib.scriptutil as scriptutil
from pyx4.msg import pyx4_test as Pyx4_test_msg
from pyx4_base.definitions_pyx4 import TEST_COMP

class TestPeerSubscribeListener(unittest.TestCase):
    def __init__(self, *args):
        super(TestPeerSubscribeListener, self).__init__(*args)
        self.visited = []
        self.results = []
        
    def callback(self, data):
        self.results[data.test_type].append(data)
        if data.waypoint not in self.visited:
            self.visited.append(data.waypoint)

    def test_notify(self):
        rospy.Subscriber("pyx4_test/pyx4_test", Pyx4_test_msg, self.callback)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)

        for result in self.results:
            self.assertTrue(data.passed, msg=data.description)
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPeerSubscribeListener, sys.argv)
