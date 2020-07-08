#!/usr/bin/env python
from __future__ import print_function

PKG = 'pyx4'
NAME = 'pyx4_test'

import sys, time, os, csv
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
        self.targets = Pyx4Test._parse_mission_file(mission_file)
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

    @staticmethod
    def _parse_mission_file(mission_file):
        """ Read the test comparison file and return a dictionary of arrays,
        one for each waypoint.
        :param comp_file: a CSV file (label, x, y, z, yaw)
        :return {index: Array(x, y, z, yaw)}
        """
        with open(mission_file, 'r') as f:
            reader = csv.DictReader(f)
            return {i+1: {'xy_type': dic['xy_type'],
                          'x': dic['x_setpoint'],
                          'y': dic['y_setpoint'],
                          'z_type': dic['z_type'],
                          'z': dic['z_setpoint'],
                          'yaw_type': dic['yaw_type'],
                          'yaw': dic['yaw_setpoint']}
                    for i, dic in enumerate(reader)}

    def pyx4_callback(self, data):
        """ Compares the test data for the current waypoint to self.current_pos
        :param data: pyx4_state message
        """
        if self.current_wpt < self.total_wpts:
            pass_p = np.allclose(self.wpts[self.current_wpt],
                                 self.current_pos,
                                 rtol=2, atol=1)
            rospy.loginfo("""wpts[current]: {}, current pos:
            {}""".format(self.wpts[self.current_wpt],
                         self.current_pos,))
        else:
            pass_p = False
            
        rospy.loginfo("{} pass: {}".format(data.state_label, pass_p))
        msg = Pyx4_test_msg()
        msg.test_type = 'wpt_pos'
        msg.passed = pass_p
        self.pyx4_test_pub.publish(msg)

        self.setpoint_type_check(data, self.target_sample_num)
        self.current_wpt += 1

    def setpoint_type_check(self, cb_data, target_sample_num):
        # Last waypoint doesn't have target and
        # arming state is not needed because it is not in the mission CSV
        if (self.current_wpt < self.total_wpts - 1 and
            cb_data.state_label != 'arming - generic'):
            # Discard the first samples to avoid errors
            while self.target_sample_num <= target_sample_num + 35:
                pass

            data_target = self.targets[self.current_wpt]

            # If target for xy is position
            if data_target['xy_type'] == 'pos':
                # If the x target in the mission is different than launched
                if (int(data_target['x']) !=
                    round(self.current_target.position.x)):
                    passed = False
                    self.send_target_error_msg(passed, 'x',
                                               data_target['xy_type'],
                                               data_target['x'],
                                               'position',
                                               self.current_target.position.x)

                # If the y target in the mission is different than launched
                if (int(data_target['y']) !=
                    round(self.current_target.position.y)):
                    self.send_target_error_msg(passed, 'y',
                                               data_target['xy_type'],
                                               data_target['y'],
                                               'position',
                                               self.current_target.position.y)

            # If target for xy is velocity
            elif data_target['xy_type'] == 'vel':
                # If the x target in the mission is different than launched
                if (int(data_target['x']) !=
                    round(self.current_target.velocity.x)):
                    passed = False
                    self.send_target_error_msg(passed, 'x',
                                               data_target['xy_type'],
                                               data_target['x'],
                                               'velocity',
                                               self.current_target.velocity.x)
                                                
                # If the y target in the mission is different than launched
                if (int(data_target['y']) !=
                    round(self.current_target.velocity.y)):
                    passed = False
                    self.send_target_error_msg(passed, 'y',
                                               data_target['xy_type'],
                                               data_target['y'],
                                               'velocity',
                                               self.current_target.velocity.y)
            # If target for z is position
            if data_target['z_type'] == 'pos':
                # If the z target in the mission is different than launched
                if (int(data_target['z']) !=
                    round(self.current_target.position.z)):
                    passed = False
                    self.send_target_error_msg(passed, 'z',
                                               data_target['z_type'],
                                               data_target['z'],
                                               'position',
                                               self.current_target.position.z)
                    
            # If target for yaw is position
            if data_target['yaw_type'] == 'pos':
                # If the yaw target in the mission is different than launched
                if (int(data_target['yaw']) !=
                    round(self.current_target.yaw)):
                    passed = False
                    self.send_target_error_msg(passed, 'yaw',
                                               data_target['yaw_type'],
                                               data_target['yaw'],
                                               'position',
                                               self.current_target.yaw)

            if passed:
                msg = Pyx4_test_msg()
                msg.test_type = 'target'
                msg.passed = True
                msg.error = ''
                self.pyx4_test_pub.publish(msg)
                rospy.loginfo("""TARGET TEST:
                Waypoint {} passed the target test.
                """.format(self.current_wpt))
                                    
        
    def send_target_error_msg(self, passed, sp, expected_type, expected,
                              given_type, given):
        """ Function to publish the error message when the target is incorrect.
        :param sp: setpoint (xy, z, yaw)
        :param expected_type: expected setpoint type (pos, vel)
        :param expected: expected value
        :param given_type: pos, vel
        :param given: given value
        """
        msg = Pyx4_test_msg()
        msg.test_type = 'target'
        msg.passed = passed
        error_msg = """TARGET TEST:
        Waypoint {} did not pass the target test for setpoint {}:
        - Expected type {} with value {}, but got type {} with value {}
        """.format(self.current_wpt, sp, expected_type, expected,
                   given_type, given)
        
        self.pyx4_test_pub.publish(msg)
        rospy.loginfo(error_msg)

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

        # Subscribe to mavros/setpoint_raw/local
        rospy.Subscriber("mavros/setpoint_raw/target_local", PositionTarget,
                         self.position_target_callback)

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
