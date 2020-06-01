#!/usr/bin/env python2
import numpy as np

from setpoint_bitmasks import *
from tf.transformations import euler_from_quaternion

def pose2yaw(this_pose):
    orientation_q = this_pose.pose.orientation
    (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    return yaw


def get_bitmask(xy_type, z_type, yaw_type):

    # all pos control
    if xy_type == 'pos' and z_type == 'pos' and yaw_type == 'pos':
        bitmask = MASK_XY_POS__Z_POS_YAW_POS
    elif xy_type == 'pos' and z_type == 'pos' and yaw_type == 'pos':
        bitmask = MASK_XY_POS__Z_POS_YAW_RATE

    # all pos control with limited vel (only available in horizontal motion at the moment
    elif xy_type == 'pos_with_vel':
        bitmask = MASK_XY_POS_XY_VEL_Z_POS_YAW_POS
        # todo, raise if this condition is met but an unsupported variant

    # xyz pos, yaw rate
    elif xy_type == 'pos' and z_type == 'pos' and yaw_type == 'vel':
        bitmask = MASK_XY_POS__Z_POS_YAW_RATE

    # xy velocity control
    elif xy_type == 'vel' and z_type == 'pos' and yaw_type == 'pos':
        bitmask = MASK_XY_VEL__Z_POS__YAW_POS
    elif xy_type == 'vel' and z_type == 'pos' and yaw_type == 'vel':
        bitmask = MASK_XY_VEL__Z_POS__YAW_RATE

    # xyz velocity control
    elif xy_type == 'vel' and z_type == 'vel' and yaw_type == 'pos':
        bitmask = MASK_XY_VEL__Z_VEL_YAW_POS
    elif xy_type == 'vel' and z_type == 'vel' and yaw_type == 'vel':
        bitmask = MASK_XY_VEL__Z_VEL_YAW_RATE
    else:
        raise TypeError(('this control combination is not yet implemented xy_type {} z_type {} yaw_type {}'.format(xy_type, z_type, yaw_type)))

    return np.uint16(bitmask)