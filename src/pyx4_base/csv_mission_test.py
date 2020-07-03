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
from pyx4.msg import pyx4_state as Pyx4_msg
from definitions_pyx4 import TEST_COMP

class TestPeerSubscribeListener(unittest.TestCase):
    def __init__(self, *args):
        super(TestPeerSubscribeListener, self).__init__(*args)
        self.success = False
        self.wpts = TestPeerSubscribeListener._parse_comp_file()
        self.total_wpts = len(self.wpts)
        self.current_wpt = 0
        self.atol, self.rtol = 0, 10

    @staticmethod
    def _parse_comp_file():
        """ Read the test comparison file and return a dictionary of arrays,
        one for each waypoint.
        :param comp_file: a CSV file (label, x, y, z, yaw)
        :return {index: Array(x, y, z, yaw)}
        """
        with open(comp_file, 'r') as f:
            reader = csv.DictReader(f)
            return {i: [np.array([v for k, v in dic.iteritems()
                                 if k in ['x', 'y', 'z', 'yaw']])]
                    for i, dic in enumerate(reader)}
        
    def callback(self, data):
            
        if self.current_wpt < self.total_wpts:
            self.wpts[self.current_wpt].append(np.array([data.x,
                                                         data.y,
                                                         data.z,
                                                         data.yaw]))
            
        # if np.allclose(self.wpts[self.current_wpt][0],
        #                self.wpts[self.current_wpt][1],
        #                rtol=10):
        #     self.success = True
        self.current_wpt += 1

    def test_notify(self):
        rospy.Subscriber("pyx4_node/pyx4_state", Pyx4_msg, self.callback)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success, str(self.success))
        
if __name__ == '__main__':
    # TODO Make a script to import the csv test filles in
    # test_data and csv_mission_test
    # import argparse
    # parser = argparse.ArgumentParser(description="ROS test node")
    # parser.add_argument('--csv', type=str, default='basic_test.csv')
    # args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    comp_file = os.path.join(TEST_COMP, sys.argv[0])

    # if not os.path.isfile(comp_file):
    #     raise AttributeError("""file {} does not exist.
    #     Run test_data to create the test data for the selected mission.
    #     """.format(comp_file, mission_file))
    rostest.rosrun(PKG, NAME, TestPeerSubscribeListener, sys.argv)
