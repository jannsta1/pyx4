#!/usr/bin/env python3
import sys, unittest, time, os, csv
import rospy, rostest
import numpy as np
from pyx4.msg import pyx4_state as Pyx4_msg
from pyx4_base.definitions_pyx4 import TEST_COMP

class TestCSVMission(unittest.TestCase):
    
    def __init__(self, *args):
        super(TestCSVMission, self).__init__(*args)
        print(args)
        self.comp_wpts = TestCSVMission._parse_comp_file(comp_file)
        self.total_wpts = len(self.comp_wpts)
        self.current_wpt = 0
        self.success = True
        self.atol, self.rtol = 0, 10

    @staticmethod
    def _parse_comp_file(comp_file):
        """ Read the test comparison file and return a dictionary of arrays,
        one for each waypoint.
        :param comp_file: a CSV file (label, x, y, z, yaw)
        :return {index: Array(x, y, z, yaw)}
        """
        with open(comp_file, 'r') as f:
            reader = csv.DictReader(f)
            return {i: np.array([v for k, v in dic.items()
                                 if k in ['x', 'y', 'z', 'yaw']])
                    for i, dic in enumerate(reader)}
    
    def callback(self, data):
        """ Receive a waypoint data from the subscriber and compare it
        to the comparison data.
        Comparison done using numpy.allclose, where
          |a - b| <= (abs_tolerance + rel_tolerance * |b|)
          a,b in A, B.
        """
        pass_p = False
        wp_data = np.array([data.x, data.y, data.z, data.yaw])
        if self.current_wpt < self.total_wpts:
            pass_p = np.allclose(wp_data, self.comp_wpts[self.current_wpt])

        self.success *= pass_p
        self.current_wpt += 1
        
            
    def test_mission(self):
        rospy.init_node('csv_mission_test')
        rospy.Subscriber("pyx4_node/pyx4_state", Pyx4_msg, self.callback)
        timeout_t = time.time() + 10.0
        while (not rospy.is_shutdown() and
               not self.success and time.time() < timeout_t):
            time.sleep(0.1)
    
        self.assertTrue(self.success)

if __name__ == '__main__':
    # TODO Make a script to import the csv test filles in
    # test_data and csv_mission_test
    import argparse
    parser = argparse.ArgumentParser(description="ROS test node")
    parser.add_argument('--csv', type=str, default='basic_test.csv')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    comp_file = os.path.join(TEST_COMP, args.csv)

    if not os.path.isfile(comp_file):
        raise AttributeError("""file {} does not exist.
        Run test_data to create the test data for the selected mission.
        """.format(comp_file, mission_file))
    print(comp_file)
    rostest.rosrun('pyx4', 'csv_mission_test', TestCSVMission)




