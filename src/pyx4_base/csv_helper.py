#!/usr/bin/env python
from __future__ import print_function

PKG = 'pyx4'
NAME = 'pyx4_test'

import sys, time, os, csv
import multiprocessing as mp
import numpy as np
import rospy
from pyx4.msg import pyx4_state as Pyx4_msg
from pyx4.msg import pyx4_test as Pyx4_test_msg
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from definitions_pyx4 import TEST_COMP, MISSION_SPECS

class Pyx4Test():
    def __init__(self, mission_file, comp_file):
        self.success = False
        self.wpts = Pyx4Test._parse_comp_file(comp_file)
        self.total_wpts = len(self.wpts)
        self.current_wpt = 0
        self.current_pos = []
        self.current_target = None
        self.target_sample_num = 0
        self.atol, self.rtol = 0, 10
        self.pyx4_test_pub = rospy.Publisher(NAME + '/pyx4_test',
                                             Pyx4_test_msg, queue_size=10)

    @staticmethod
    def _parse_comp_file(comp_file):
        """ Read the test comparison file and return a dictionary of arrays,
        one for each waypoint.
        :param comp_file: a CSV file (label, x, y, z, yaw)
        :return {index: Array(x, y, z, yaw)}
        """
        with open(comp_file, 'r') as f:
            reader = csv.DictReader(f)
            return {i: np.array(map(float, [dic['x'],
                                             dic['y'],
                                             dic['z'],
                                             dic['yaw']]))
                    for i, dic in enumerate(reader)}

    def pyx4_callback(self, data):
        """ Compares the test data for the current waypoint to self.current_pos
        :param data: pyx4_state message
        """
        if self.current_wpt < self.total_wpts:
            pass_p = np.allclose(self.wpts[self.current_wpt],
                                 self.current_pos,
                                 rtol=2, atol=1)
            # rospy.loginfo("""wpts[current]: {}, current pos:
            # {}""".format(self.wpts[self.current_wpt],
            #              self.current_pos,))
        else:
            pass_p = False
            
        rospy.loginfo("{} pass: {}".format(data.state_label, pass_p))
        msg = Pyx4_test_msg()
        msg.test_type = 'wpt_pos'
        msg.passed = pass_p
        self.pyx4_test_pub.publish(msg)

        self.current_wpt += 1

    def local_position_callback(self, data):
        """ Gets the local position data from /mavros/local_position/pose and
        updates the attribute current.
        """
        pos = data.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z,
                                     data.pose.orientation.z])

    def position_target_callback(self, data):
        self.target_sample_num += 1
        self.current_target = data

    def main(self):
        """ Method to manage subscriptions:
        
        - pyx4_state topic: to know when a waypoint is reached.
          Callback: compare the local position in the test data for that
                    waypoint to the mavros/local_position data.
       
        - mavros/local_position: receive the local position
          Callback: update the attribute self.current_pos

        - mavros/setpoint_raw_local: receive the target setpoint
          Callback: compare the target setpoint to the type of each instruction
                    in the mission CSV.
        """
        # Subscribe to pyx4_state
        rospy.Subscriber("pyx4_node/pyx4_state", Pyx4_msg,
                         self.pyx4_callback)
        # Subscribe to mavros/local_position
        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                         self.local_position_callback)

        rospy.init_node(NAME, anonymous=True)
        # TODO: Set proper time
        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)
        
if __name__ == '__main__':
    #TODO Make a script to import the csv test filles in
    #test_data and csv_mission_test
    import argparse
    parser = argparse.ArgumentParser(description="ROS test node")
    parser.add_argument('--csv', type=str, default='basic_test.csv')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    # Mission and comparisson files have the same name by definition
    comp_file = os.path.join(TEST_COMP, args.csv)
    mission_file = os.path.join(MISSION_SPECS, args.csv)

    if not os.path.isfile(mission_file):
        raise AttributeError("""Mission file {} not found.
        """.format(mission_file))

    if not os.path.isfile(comp_file):
        raise AttributeError("""file {} does not exist.
        Run test_data to create the test data for the selected mission.
        """.format(comp_file))
    pyx4_test = Pyx4Test(mission_file, comp_file)
    pyx4_test.main()
