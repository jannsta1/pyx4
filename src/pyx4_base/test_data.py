#!/usr/bin/env python
import rospy
import csv
import sys
import os
from pyx4.msg import pyx4_state as Pyx4_msg
from definitions_pyx4 import TEST_COMP

"""
ROS node to get the data to do the comparisons for the testing.

It subscribes to the pyx4_state topic, and adds the label and the
position for each waypoint to a CSV file.
"""

CSV_HEADER = ['label', 'x', 'y', 'z', 'yaw']

def callback(data):
    row = [data.state_label, data.x, data.y, data.z, data.yaw]
    # Open the file in append mode to not overwrite
    rospy.loginfo(row)
    with open(comp_file, 'a') as csvfile:
        writer = csv.writer(csvfile, delimiter=',',)
        writer.writerow(row)

        
def get_test_data():
    rospy.init_node('test_data', anonymous=True)
    rospy.Subscriber("pyx4_node/pyx4_state", Pyx4_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Node to create test data")
    parser.add_argument('--csv', type=str, default='basic_test.csv')
    parser.add_argument('--overwrite', type=str, default='False')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # TODO Find a better way to pass the overwrite arg
    if args.overwrite == 'False': args.overwrite = False
    
    if os.path.isabs(args.csv):
        comp_file = args.csv
    else:
        comp_file = os.path.join(TEST_COMP, args.csv)

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

    get_test_data()
