#!/usr/bin/env python3

from mavros_msgs.msg import PositionTarget

from pyx4_base.pyx4_base_classes import Pyx4_base
from pyx4_base.mission_states import *

from warnings import warn

def  Survey_mission(
                        mission_type='hover',
                        control_type='pos',
                        duration=30,
                        height=3.0,
                        heading=0.0,
                        x_length=1.0,
                        y_length=10.0,
                        x_offset=1.0,
                        y_offset=1.0,
                        width_between_runs=3.0,
                        z_tgt_rel=0.0,
                        radius=5.0,
                        ):
    '''

    this mission type wasn't completed as there wasn't a good way of limiting the velocity of the waypoints while
    in offboard and position control modes


    Allows some user to input a simple rectanular area for surveying.

    NB. Px4/QGroundcontrol already has a nice tool for this - here I just wanted something quick and dirty to use in my
    ros setup that already has functins for triggering the camera

    The following parameters are provided: heading, x and y parameters ...

    :param file_path:
    :return:
    '''

    warn("this mission type wasn't completed as there wasn't a good way of limiting the velocity of the waypoints while"
         " in offboard and position control modes")

    instructions = {}
    instruction_cnt = 0

    # create test grid based on x & y length specifications
    x_initial = - np.ceil(y_length / 2.0)
    y_initial = 0.0

    qty_ys = (np.ceil(y_length / width_between_runs) * 2).astype(np.int)

    # generate x positions
    ys = np.arange(x_initial, -x_initial+width_between_runs, width_between_runs)
    ys_all = np.empty((ys.size + ys.size,), dtype=ys.dtype)
    ys_all[0::2] = ys
    ys_all[1::2] = ys

    # generate y positions
    reps = np.int(np.ceil(qty_ys/4.))
    xs_all = np.tile(np.array((y_initial, y_length, y_length, y_initial)), 10) [0:len(ys_all)]

    # append home to our target

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

    ################################### main loop
    for x_tgt, y_tgt in zip(xs_all, ys_all):
        print(('generating waypoint at {}, {} '.format(x_tgt, y_tgt)))
        instructions[instruction_cnt] = Waypoint_state(
            timeout=duration,
            state_label='Going out',
            waypoint_type='pos',
            xy_type='pos_with_vel',
            x_setpoint=x_tgt,
            y_setpoint=y_tgt,
            x_vel_setpoint=1.0,
            y_vel_setpoint=1.0,
            z_type='pos',
            z_setpoint=height,
            yaw_type='pos',
            yaw_setpoint=heading,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt = instruction_cnt + 1

    ################################### return to home
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

    ######################################################################################################
    ################################## Landing instruction ###############################################
    instructions[instruction_cnt] = Landing_state()
    instruction_cnt += 1

    print(xs_all)
    print(ys_all)

    return instructions
    # translate gid points based on the offset

    # rotate grid points based on heading

    # plot and record coordinates of way points



if __name__ == '__main__':

    # parser = argparse.ArgumentParser(description="Generates a mission from a simple argument list")
    #
    # # # parser.add_argument('-m', '--mission_type', type=str, default='hover')
    # # parser.add_argument('-m', '--mission_type',
    # #                     type=lambda x: is_valid_option(parser, x, VALID_MISSIONS),
    # #                     help="Checks if mission is a valid option",
    # #                     default='hover'
    # #                     )
    # #
    # # parser.add_argument('-d', '--duration', type=float, default=10.0)
    # # parser.add_argument('-a', '--altitude', type=float, default=5.0)
    # # parser.add_argument('--vel_cap', type=bool, default=3.0)
    # #
    # # parser.add_argument('-c', '--control_type',
    # #                     type=lambda x: is_valid_option(parser, x, VALID_CONTROL_TYPES),
    # #                     help="Checks if control type is a valid option",
    # #                     default='pos',
    # #                     )
    #
    # parser.add_argument('-x', '--x_tgt', type=float, default=3.0)
    # parser.add_argument('-y', '--y_tgt', type=float, default=3.0)
    # parser.add_argument('-z', '--z_tgt_rel', type=float, default=3.0)
    # parser.add_argument('-r', '--radius', type=float, default=3.0)
    # parser.add_argument('--heading', type=float, default=0.0)
    #
    # args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    #
    #
    # # todo apply vel cap and warn if effected
    # # todo - default duration shorter if control type is vel

    flight_instructions = Survey_mission(
                                                # mission_type=args.mission_type,
                                                # control_type=args.control_type,
                                                # duration=args.duration,
                                                # height=args.altitude,
                                                # heading=args.heading,
                                                # # x_tgt=args.x_tgt,
                                                # # y_tgt=args.y_tgt,
                                                # z_tgt_rel=args.z_tgt_rel,
                                                # radius=args.radius,
                                              )

    rospy.init_node('pyx4_survey_node', anonymous=True, log_level=rospy.DEBUG)

    pyx4 = Pyx4_base(flight_instructions=flight_instructions)
    pyx4.run()
    flight_instructions = Survey_mission()