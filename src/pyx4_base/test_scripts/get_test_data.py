#!/usr/bin/env python
import rospy
import csv
import sys
import os
from pyx4.msg import pyx4_state as Pyx4_msg
from geometry_msgs.msg import PoseStamped
from pyx4_base.definitions_pyx4 import TEST_COMP

"""
ROS node to get the data to do the comparisons for the testing.

It subscribes to the pyx4_state topic, and adds the label and the
position for each waypoint to a CSV file.
"""

CSV_HEADER = ['label', 'x', 'y', 'z', 'yaw']

class TestData():
    def __init__(self, csv):
        self.current_pos = []
        self.csv = csv
        

    def pyx4_callback(self, data):
        """ Function triggered when a waypoint is reached.
        Checks the current local possition and adds it to the csv
        :param data: pyx4 state message
        """
        rospy.loginfo("""Waypoint {}:
        x: {},
        y: {},
        z: {},
        yaw: {}""".format(data.state_label, self.current_pos[0],
                           self.current_pos[1], self.current_pos[2],
                           self.current_pos[3]))
        
        # Open the CSV file in append mode to add the position
        # of the new waypoint.
        with open(self.csv, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',)
            writer.writerow([data.state_label] + self.current_pos)

    def local_position_callback(self, data):
        """ Gets the local position data from /mavros/local_position/pose and
        updates the attribute current.
        """
        pos = data.pose.position
        self.current_pos = [pos.x, pos.y, pos.z, data.pose.orientation.z]
        
    def get_data(self):
        rospy.init_node('test_data', anonymous=True)
        rospy.Subscriber("pyx4_node/pyx4_state", Pyx4_msg, self.pyx4_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                         self.local_position_callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Node to create test data")
    parser.add_argument('--comp', type=str, default='basic_test.csv')
    parser.add_argument('--overwrite', type=str, default='False')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # TODO Find a better way to pass the overwrite arg
    if args.overwrite == 'False': args.overwrite = False
    
    if os.path.isabs(args.comp):
        comp_file = args.comp
    else:
        comp_file = os.path.join(TEST_COMP, args.comp)

    # If the overwrite argument is false and the file already exists,
    # throw an error.
    # Overwriting an existing comparison file could be dangerous as it
    # could break the tests.
    if os.path.isfile(comp_file) and args.overwrite == False:
        raise AttributeError("""file {} already exists and
        overwrite is set to false""".format(comp_file))

    # Overwrite the file with the CSV header
    else:
        with open(comp_file, 'w') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',)
            writer.writerow(CSV_HEADER)

    test_data = TestData(comp_file)
    test_data.get_data()
