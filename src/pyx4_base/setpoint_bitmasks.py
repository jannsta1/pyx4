#!/usr/bin/env python3

"""
This module defines the setpoint bitmasks that are used for use with the setpoint_raw mavros message
"""

from mavros_msgs.msg import PositionTarget

pos_sp_bitmasks = {
            'px':PositionTarget.IGNORE_PX,
            'py':PositionTarget.IGNORE_PY,
            'pz':PositionTarget.IGNORE_PZ,
            'vx':PositionTarget.IGNORE_VX,
            'vy':PositionTarget.IGNORE_VY,
            'vz':PositionTarget.IGNORE_VZ,
            'yaw':PositionTarget.IGNORE_YAW,
            'yaw_rate':PositionTarget.IGNORE_YAW_RATE,
            'afx':PositionTarget.IGNORE_AFX,
            'afy':PositionTarget.IGNORE_AFY,
            'afz':PositionTarget.IGNORE_AFZ,
            'f':PositionTarget.FORCE,                # think this changes acceleration flags to force flags
            }

ignore_all_bitmask = sum(pos_sp_bitmasks.values())
MASK_ALL = ignore_all_bitmask
MASK_XY_POS__Z_POS_YAW_POS   = ignore_all_bitmask - pos_sp_bitmasks['px'] - pos_sp_bitmasks['py'] - pos_sp_bitmasks['pz'] - pos_sp_bitmasks['yaw']
MASK_XY_POS__Z_POS_YAW_RATE  = ignore_all_bitmask - pos_sp_bitmasks['px'] - pos_sp_bitmasks['py'] - pos_sp_bitmasks['pz'] - pos_sp_bitmasks['yaw_rate']
MASK_XY_VEL__Z_POS__YAW_POS  = ignore_all_bitmask - pos_sp_bitmasks['vx'] - pos_sp_bitmasks['vy'] - pos_sp_bitmasks['pz'] - pos_sp_bitmasks['vz'] - pos_sp_bitmasks['yaw']
MASK_XY_VEL__Z_POS__YAW_RATE = ignore_all_bitmask - pos_sp_bitmasks['vx'] - pos_sp_bitmasks['vy'] - pos_sp_bitmasks['pz'] - pos_sp_bitmasks['vz'] - pos_sp_bitmasks['yaw_rate']
MASK_XY_VEL__Z_VEL_YAW_POS   = ignore_all_bitmask - pos_sp_bitmasks['vx'] - pos_sp_bitmasks['vy'] - pos_sp_bitmasks['vz'] - pos_sp_bitmasks['yaw']
MASK_XY_VEL__Z_VEL_YAW_RATE  = ignore_all_bitmask - pos_sp_bitmasks['vx'] - pos_sp_bitmasks['vy'] - pos_sp_bitmasks['vz'] - pos_sp_bitmasks['yaw_rate']
MASK_XY_POS_XY_VEL_Z_POS_YAW_POS   = ignore_all_bitmask - pos_sp_bitmasks['px'] - pos_sp_bitmasks['py'] - pos_sp_bitmasks['vx'] - pos_sp_bitmasks['vy'] - pos_sp_bitmasks['pz'] - pos_sp_bitmasks['yaw']