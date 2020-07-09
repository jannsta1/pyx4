#!/usr/bin/env python
from __future__ import print_function

PKG = 'pyx4'
NAME = 'test_csv_mission'

import sys 
import time
import unittest
import os
import csv

import numpy as np

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import String
from pyx4.msg import pyx4_test as Pyx4_test_msg
from definitions_pyx4 import TEST_COMP

class TestPeerSubscribeListener(unittest.TestCase):
    def __init__(self, *args):
        super(TestPeerSubscribeListener, self).__init__(*args)
        self.results = {'wpts_pos': [], 'basic_checks': []}
        self.success = False
        
    def callback(self, data):
        self.results[data.test_type].append(data.passed)

    def test_notify(self):
        rospy.Subscriber("pyx4_test/pyx4_test", Pyx4_test_msg, self.callback)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)

        for result in self.results['wpts_pos']:
            self.assertTrue(result)
            self.assertIs(len(self.results['wpts_pos']), 12)
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPeerSubscribeListener, sys.argv)
