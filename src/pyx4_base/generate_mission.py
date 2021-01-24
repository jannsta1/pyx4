#!/usr/bin/env python3

import csv
import os

from pyx4_base.definitions_pyx4 import MISSION_SPECS
from pyx4_base.mission_states import *


def Wpts_from_csv(file_path):
    '''
    Loads a csv file into our flight instruction data format

    We automatically assume that the first 2 states will be arm and takeoff

    The first instruction must be of type position (in all axis)

    mission instructions are re: centered around the takeoff location

    NB. If a simple routine with parametised inputs is required than a programable mission is a better option

    :param file_path:
    :return:
    '''

    instructions = {}
    instruction_cnt = 0
    first_row = True
    pos_tol = 0.2,  # tolerance for position waypoints todo - incorporate this properly

    # Automatically provide the arming state
    instructions[instruction_cnt] = Arming_state(
        timeout=90
    )
    instruction_cnt += 1

    with open(file_path, 'rb') as f:

        reader = csv.DictReader(f)
        for row in reader:

            # prepend mission with a takeoff at the height of the first waypoint
            if first_row:
                # evaluate first instuction and add take off command
                assert (row['z_type'] == 'pos'), 'the first instruction z axis must be of type pos'
                assert (row['xy_type'] == 'pos'), 'the first instruction xy axismust be of type pos'
                assert (row['yaw_type'] == 'pos'), 'the first instruction yaw axis must be of type pos'

                instructions[instruction_cnt] = Take_off_state(
                    to_altitude_tgt=np.float64(row['z_setpoint']),
                    yaw_type='pos',
                    heading_tgt_rad=np.float64(row['yaw_setpoint']),
                )
                instruction_cnt += 1
                first_row = False

            instructions[instruction_cnt] = Waypoint_state(
                state_label='waypoint_' + str(instruction_cnt),  # waypoint state labels are mandatory
                waypoint_type=row['instruction_type'],  # hold, pos, vel_xy, vel
                xy_type=row['xy_type'],
                x_setpoint=np.float64(row['x_setpoint']),
                y_setpoint=np.float64(row['y_setpoint']),
                z_type=row['z_type'],
                z_setpoint=np.float64(row['z_setpoint']),
                yaw_type=row['yaw_type'],
                yaw_setpoint=np.float64(row['yaw_setpoint']),
                coordinate_frame=row['coordinate_frame'],
                timeout=int(row['timeout'])
                )
            instruction_cnt = instruction_cnt + 1

    return instructions


if __name__ == '__main__':

    mission_file = os.path.join(MISSION_SPECS, 'big_square.csv')
    flight_instructions = Wpts_from_csv(file_path=mission_file)
