#!/usr/bin/env python

"""
This file converts waypoints_local.py into a rosnode and instantiates a Pyx4 mission with this data.

The file is seperate to help with debugging (i.e. we can test the logic in waypoints_local.py without any ROS libraries

"""

import argparse
import os, sys
import rospy

from pyx4_src.generate_mission import Wpts_from_csv
from pyx4_src.definitions_pyx4 import MISSION_SPECS
from pyx4_src.pyx4 import Pyx4


if __name__ == '__main__':

    node_name = 'csv_mission'

    rospy.init_node(node_name, anonymous=True, log_level=rospy.DEBUG)
    parser = argparse.ArgumentParser(description="This node is a ROS side mavros based state machine.")
    parser.add_argument('--csv', type=str, default='big_square.csv')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if os.path.isabs(args.csv):
        mission_file = args.csv
    else:
        mission_file = os.path.join(MISSION_SPECS, args.csv)

    if os.path.isfile(mission_file):
        print ('running mission from csv file {}'.format(mission_file))
    else:
        raise AttributeError('File {} not found'.format(mission_file))

    flight_instructions = Wpts_from_csv(file_path=mission_file)

    pyx4 = Pyx4(flight_instructions=flight_instructions)
    pyx4.run()
