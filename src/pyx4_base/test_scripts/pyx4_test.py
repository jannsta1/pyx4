#!/usr/bin/env python3


PKG = 'pyx4'
NAME = 'pyx4_test'

import sys 
import time
import unittest
import os
import csv

import rospy
import rostest
from pyx4.msg import pyx4_test as Pyx4_test_msg
from pyx4_base_classes.definitions_pyx4 import MISSION_SPECS

class Pyx4Test(unittest.TestCase):
    def __init__(self, *args):
        super(Pyx4Test, self).__init__(*args)
        self.results = []
        self.visited = []
        
    def callback(self, data):
        """ Callback function from pyx4_test topic.
        Adds the result to the results list for assertion
        when the mission is completed.
        :param data: pyx4_test message
        """
        self.results.append(data)
        # If the waypoint is new, add it to the visited list
        if data.waypoint not in self.visited:
            self.visited.append(data.waypoint)
        

    def test_main(self):
        """ Main testing function. Called automatically by unittest.
        Subscribe to pyx4_test and waits for the mission to finish.
        Then, all the results that have been stored in self.results
        are asserted.
        """
        rospy.Subscriber("pyx4_test/pyx4_test", Pyx4_test_msg, self.callback)
        rospy.init_node(NAME, anonymous=True)
        while not rospy.is_shutdown() and time.time() < TIMEOUT:
            time.sleep(0.1)

        for result in self.results:
            self.assertTrue(result.passed, msg=result.description)

        # Testing that the amount of visited waypoints is
        # the same as in the mission
        self.assertEqual(len(self.visited), TOTAL_WAYPOINTS,
                         msg=""" VISITED WAYPOINTS NUMBER TEST: FAILED
                         There are {} waypoints in the mission,
                         but {} have been visited.
                         """.format(TOTAL_WAYPOINTS, len(self.visited)))
                         
        
if __name__ == '__main__':
    # Obtaining the total timeout and total waypoints from the mission file
    mission_file = os.path.join(MISSION_SPECS, sys.argv[1])
    with open(mission_file, 'r') as f:
        reader = csv.DictReader(f)
        # Get a list with all the rows
        wpt_list = [int(dic['timeout']) for dic in reader]
        # Nummber of rows - last one
        TOTAL_WAYPOINTS = len(wpt_list) - 1
        # Current time + 110% of mission time
        TIMEOUT = (sum(wpt_list)) * 1.1 + time.time()
        
    rostest.rosrun(PKG, NAME, Pyx4Test, sys.argv)
