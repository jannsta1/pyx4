#!/usr/bin/env python2

from __future__ import division


import argparse

from mission_states import *
from pyx4_base import Pyx4_base


VALID_MISSIONS = ['hover', 'baggins', 'ortho', 'holo']
VALID_CONTROL_TYPES = ['vel', 'pos']

"""
This file is for the generation of simple missions that can be parametised with minimal standard parameters that can
therefore be used for multiple files

"""


def Parametised_mission(
                        mission_type='hover',
                        control_type='pos',
                        duration=10,
                        height=3.0,
                        heading=0.0,
                        x_tgt=1.0,
                        y_tgt=1.0,
                        z_tgt_rel=0.0,
                        radius=5.0,
                        ):
    '''
    Allows some parametisable missions that are commonly useful for test purposes

    :param file_path:
    :return:
    '''


    instructions = {}
    instruction_cnt = 0

    # set inverse target based on control mode
    if control_type == 'pos':
        x_back = 0.0
        y_back = 0.0
    else:
        x_back = float(-x_tgt)
        y_back = float(-y_tgt)

    ################# Common instructions -> arm & take offf
    instructions[instruction_cnt] = Arming_state(
        timeout=90
    )
    instruction_cnt += 1
    instructions[instruction_cnt] = Take_off_state(
        # instruction_type='hold',
        to_altitude_tgt=height,
        yaw_type='pos',
        heading_tgt_rad=heading,
        timeout=30,
    )
    instruction_cnt += 1

    ######################################################################################################
    ################################## Hover mission #####################################################
    if mission_type == 'hover':
        instructions[instruction_cnt] = Waypoint_state(
            state_label='Hovering',
            waypoint_type='pos',
            timeout=duration,
            xy_type='pos',
            x_setpoint=0,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        if mission_type == 'hover':
            instructions[instruction_cnt] = Hold_pos_state(
                state_label='Hovering',
                waypoint_type='pos',
                timeout=duration,
                # xy_type='pos',
                # x_setpoint=0,
                # y_setpoint=0,
                # z_type='pos',
                # z_setpoint=height,
                # yaw_type='pos',
                yaw_setpoint=heading,
                # coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
            )
            instruction_cnt = instruction_cnt + 1

    ######################################################################################################
    ######################## Baggins mission vel (there and back again)########################################
    elif mission_type == 'baggins':   # (there and back again)

        # there
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=x_tgt,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # and back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Coming back',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=float(x_back),
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        ######################################################################################################
        ######################## Tests gimbal in different ########################################
    elif mission_type == 'holo':  # (there and back again)

        # and back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type='vel',
            x_setpoint=0,
            y_setpoint=1,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=1.57,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # there
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type='vel',
            x_setpoint=1,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=0.0,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # there
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type='vel',
            x_setpoint=1,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=0.0,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # there
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type='vel',
            x_setpoint=0.7,
            y_setpoint=0.7,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=0.785,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1


    ######################################################################################################
    ######################## Orthogonal mission type - move in orthogonal directions #####################
    elif mission_type == 'ortho':   # (there and back again)

        # fwd
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=x_tgt,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # and back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Coming back',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=x_back,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # right
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=0,
            y_setpoint=y_tgt,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # and back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Coming back',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=0,
            y_setpoint=y_back,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # diagon alley
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=x_tgt,
            y_setpoint=y_tgt,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # and back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Coming back',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=x_back,
            y_setpoint=y_back,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1


        # now spin round on the spot
        # and back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='spinining around',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=0,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading + (np.pi),
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

        # now spin back again
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='spinining around',
            waypoint_type=control_type,
            xy_type=control_type,
            x_setpoint=0,
            y_setpoint=0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1



    ######################################################################################################
    ################################## Landing instruction ###############################################
    instructions[instruction_cnt] = Landing_state()
    instruction_cnt += 1

    return instructions

def is_valid_option(parser, arg, checklist):
    """
    Check if a string arg is a valid option according to a checklist

    Parameters
    ----------
    parser : argparse object
    arg : str
    checklist

    Returns
    -------
    arg
    """
    if not arg in checklist:
        parser.error("'{}' parametised mission type does not exist. Valid options are {}".format(arg, checklist))
    else:
        return arg


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Generates a mission from a simple argument list")

    # parser.add_argument('-m', '--mission_type', type=str, default='hover')
    parser.add_argument('-m', '--mission_type',
                        type=lambda x: is_valid_option(parser, x, VALID_MISSIONS),
                        help="Checks if mission is a valid option",
                        default='hover'
                        )

    parser.add_argument('-d', '--duration', type=float, default=10.0)
    parser.add_argument('-a', '--altitude', type=float, default=5.0)
    parser.add_argument('--vel_cap', type=bool, default=3.0)

    parser.add_argument('-c', '--control_type',
                        type=lambda x: is_valid_option(parser, x, VALID_CONTROL_TYPES),
                        help="Checks if control type is a valid option",
                        default='pos',
                        )

    parser.add_argument('-x', '--x_tgt', type=float, default=3.0)
    parser.add_argument('-y', '--y_tgt', type=float, default=3.0)
    parser.add_argument('-z', '--z_tgt_rel', type=float, default=3.0)
    parser.add_argument('-r', '--radius', type=float, default=3.0)
    parser.add_argument('--heading', type=float, default=0.0)

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


    # todo apply vel cap and warn if effected
    # todo - default duration shorter if control type is vel


    flight_instructions = Parametised_mission(
                                                mission_type=args.mission_type,
                                                control_type=args.control_type,
                                                duration=args.duration,
                                                height=args.altitude,
                                                heading=args.heading,
                                                x_tgt=args.x_tgt,
                                                y_tgt=args.y_tgt,
                                                z_tgt_rel=args.z_tgt_rel,
                                                radius=args.radius,
                                              )

    rospy.init_node('pyx4_parametised_node', anonymous=True, log_level=rospy.DEBUG)

    pyx4 = Pyx4_base(flight_instructions=flight_instructions)
    pyx4.run()

