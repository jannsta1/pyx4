#!/usr/bin/env python2

import argparse
import os, sys
import rospy


from generate_mission import Wpts_from_csv
from definitions_pyx4 import MISSION_SPECS
from setpoint_bitmasks import MASK_XY_VEL__Z_POS__YAW_RATE
from mission_states import *
from pyx4_base import Pyx4_base
from geometry_msgs.msg import Twist

class Teleop_state(Generic_mission_state):
    """
    A mission state that takes the aircraft's current local position and
    holds it for a specified amount of time
    """

    def __init__(self,
                 flight_instruction_type='Teleoperation',
                 # waypoint state labels are mandatory
                 state_label='generic teleop',
                 timeout=60,
                 mavros_message_node=None,
                 parent_ref=None,
                 **kwargs
                 ):

        super(Teleop_state, self).__init__(
                                        flight_instruction_type=flight_instruction_type,
                                        timeout=timeout,
                                        mavros_message_node=mavros_message_node,
                                        parent_ref=parent_ref,
                                        state_label=state_label,
                                        **kwargs
                                        )  # sub and super class args


        self.type_mask = MASK_XY_VEL__Z_POS__YAW_RATE
        self.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.state_sub = rospy.Subscriber('/cmd_vel', Twist, self.teleop_node_cb)


    def precondition_check(self):
        ''' This function can be run by substates in order - this will be executed (in places of step)
        until self.preconditions_satisfied == True
        '''

        # might be worth verifying that the teleop node publisher is alive

        self.x_vel = 0.
        self.y_vel = 0.
        self.z_vel = 0.
        self.yaw_rate = 0.
        self.type_mask = MASK_XY_VEL__Z_POS__YAW_RATE
        self.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.preconditions_satisfied = True


    def step(self):

        # self.type_mask = MASK_XY_POS__Z_POS_YAW_POS
        rospy.loginfo_throttle(5, 'In teleop mode')

    def teleop_node_cb(self, data):
        self.x_vel = data.linear.x
        self.y_vel = data.linear.y
        self.z_vel = data.linear.z
        self.yaw_rate = data.angular.z


def generate_telop_mission(args):

    instructions = {}
    instruction_cnt = 0

    # Automatically provide the arming state
    instructions[instruction_cnt] = Arming_state(timeout=90)
    instruction_cnt += 1

    instructions[instruction_cnt] = Take_off_state()
    instruction_cnt += 1

    instructions[instruction_cnt] = Teleop_state()
    instruction_cnt += 1

    instructions[instruction_cnt] = Landing_state()
    instruction_cnt += 1

    return instructions


if __name__ == '__main__':

    node_name = 'teleop_mission'

    rospy.init_node(node_name, anonymous=True, log_level=rospy.DEBUG)
    parser = argparse.ArgumentParser(description="Teleoperation px4 quadcopter.")
    parser.add_argument('-t', '--timeout', type=float, default=30)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    flight_instructions = generate_telop_mission(args)

    pyx4 = Pyx4_base(flight_instructions=flight_instructions)
    pyx4.run()
