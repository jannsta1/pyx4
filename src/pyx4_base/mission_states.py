#!/usr/bin/env python2
"""
This file contains commonly used mission states that that can be loaded into a mission by the commander.

All mission states should inherit from the 'Generic_mission_state'

"""

from __future__ import division

from copy import copy
import numpy as np
import rospy
import sys

from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import ExtendedState
from sensor_msgs.msg import NavSatFix

from definitions_pyx4 import VALID_WAYPOINT_TYPES, TAKE_OFF_PHASE
from setpoint_bitmasks import MASK_XY_POS__Z_POS_YAW_POS, MASK_XY_VEL__Z_VEL_YAW_POS, MASK_XY_VEL__Z_VEL_YAW_RATE
from utils import get_bitmask


class Generic_mission_state(object):
    """ Base class for flight behaviours.

    Pyx4 commander module handles timeout.

    """

    def __init__(self,
                 flight_instruction_type='generic_mission_state',
                 timeout=10,
                 mavros_message_node=None,
                 parent_ref=None,
                 # tol_heading_deg=1.0,
                 state_label="unspecified",   # use this to store information about the purpose of the state, e.g. "explore the world"
                 timeout_OK=True,  # this state should be set to false if timouts should be considered a mission failure
                 tol_distance=0.2,
                 tol_heading_deg=5,
                 **kwargs
                 ):

        self.flight_instruction_type = flight_instruction_type
        self._timeout = timeout
        self._setpoint_raw = PositionTarget()

        self.state_label = state_label
        self.timeout_OK = timeout_OK
        self.mission_state_busy = False    # this flag is used to indicate that the flight state is still processing an instruction and should not be polled again

        # Initialise mission state setpoint with the previous states setpoint
        self.update_sp_locals()

        # initialise local properties
        self._ros_message_node = mavros_message_node
        self._parent_ref = parent_ref
        self.stay_alive = True
        self.preconditions_satisfied = False
        self._prerun_complete = False
        self.tol_distance = tol_distance
        self.tol_heading_deg = tol_heading_deg
        self.tol_heading_rad = np.deg2rad(tol_heading_deg)


    def update_sp_locals(self):
        """
        Initialise the setpoint with current setpoint information until new instruction has succesfully initilaised
        """
        self.x = self._setpoint_raw.position.x
        self.y = self._setpoint_raw.position.y
        self.z =  self._setpoint_raw.position.z
        self.x_vel = self._setpoint_raw.velocity.x
        self.y_vel = self._setpoint_raw.velocity.y
        self.z_vel = self._setpoint_raw.velocity.z
        self.yaw = self._setpoint_raw.yaw
        self.yaw_rate = self._setpoint_raw.yaw_rate
        self.coordinate_frame = self._setpoint_raw.coordinate_frame
        self.type_mask = self._setpoint_raw.type_mask


    def pre_run(self, parent_ref, ros_message_node=None, current_sp_raw=PositionTarget()):
        """
        Do not overload this function!! (use precondition_check if customized initialisation functionality is required)

        provides mechanism for seemless transition from previous instruction / access to necessary parent class data

        """
        self._ros_message_node = ros_message_node
        self._parent_ref = parent_ref
        self._setpoint_raw = current_sp_raw
        self.update_sp_locals()
        self._prerun_complete = True
        rospy.loginfo('prerun complete')


    # this is deprecated as of 20.05.2020
    # def pre_run_inherited(self):
    #     ''' placeholder for inherited staes to add a prerun function  '''
    #     pass


    def precondition_check(self):
        ''' This function can be run by substates in order - this will be executed (in places of step)
        until self.preconditions_satisfied == True
        '''
        # start test with True then run a series of tests
        condition = True
        # if self.ros_message_node is None:
        #     rospy.logwarn('mavros node not yet -> flight instruction')
        #     condition = False
        self.preconditions_satisfied = condition


    @property
    def timeout(self):
        return self._timeout


    @property
    def ros_message_node(self):
        return self._ros_message_node


    @ros_message_node.setter
    def ros_message_node(self, new_ros_msg_node):
        self._ros_message_node = new_ros_msg_node


    @property
    def sp_raw(self):
        self._setpoint_raw.position.x = self.x
        self._setpoint_raw.position.y = self.y
        self._setpoint_raw.position.z = self.z
        self._setpoint_raw.velocity.x = self.x_vel
        self._setpoint_raw.velocity.y = self.y_vel
        self._setpoint_raw.velocity.z = self.z_vel
        self._setpoint_raw.yaw = self.yaw
        self._setpoint_raw.yaw_rate = self.yaw_rate
        self._setpoint_raw.coordinate_frame = self.coordinate_frame
        self._setpoint_raw.type_mask = self.type_mask  # 1027

        self._setpoint_raw.header.stamp = rospy.Time.now()

        return self._setpoint_raw


    @staticmethod
    def heading_error_rad(x, y):
        # todo - is there already a tf function for this?
        '''
        returns the heading error of the UAV in radians
        :param x:
        :param y:
        :return:
        '''
        return min((2 * np.pi) - abs(x - y), abs(x - y))


    @property
    def sp_error_yaw(self):
        # todo - is there already a tf function for this?
        return self.heading_error_rad(self._ros_message_node.yaw_local, self.yaw)


    @property
    def sp_error_xyz(self):
        # todo - is there already a tf function for this?
        a = np.array((self.x, self.y, self.z))

        b = np.array((self._ros_message_node.local_x, self._ros_message_node.local_y, self._ros_message_node.local_z))
        distance = np.linalg.norm(a - b).round(decimals=3)
        return distance


    @property
    def waypoint_reached(self):
        '''
        :return:
        '''

        # todo 1, parse the tolorences from mission and add ability to set a generic default for all waypoints of a mission
        # todo 2, add a timer so that the time that a waypoint has been held for can be measured

        return ((self.sp_error_xyz) < self.tol_distance and (self.sp_error_yaw < self.tol_heading_rad))


    def step(self):
        raise NotImplementedError("Subclasses should implement a step method")


    def validate_super_class(self):
        # todo - want to be able to check that the super class has all requirements for this method
        #  , perhaps this method returns a dictionary of things to check
        pass


    def __enter__(self):
        return self


    def __exit__(self):
        pass


class Waypoint_state(Generic_mission_state):
    """
    A mission state for specifying waypoints.

    The waypoint has different types:

    # hold, wpt, vel

    The axis: XY, Z and yaw can be individually specified in terms of:

    # control type: hold, wpt, vel
    # setpoint

    The state can be exited when:

    # All axis are pos control, and the setpoint has been reached within a specified euclidean tolerance
    # timeout is exceeded
    # external mission increment

    """

    def __init__(self,
                 state_label,                 # waypoint state labels are mandatory
                 waypoint_type,                         # hold, pos, vel_xy, vel
                 xy_type,
                 x_setpoint,
                 y_setpoint,
                 z_type,
                 z_setpoint,
                 yaw_type,
                 yaw_setpoint,
                 coordinate_frame,
                 x_vel_setpoint=None,
                 y_vel_setpoint=None,
                 pos_tol=0.2,                     # tolerance for position waypoints
                 tol_heading_deg=5,
                 flight_instruction_type='Waypoint',
                 timeout=30,
                 mavros_message_node=None,
                 update_status_rate=5.0,
                 to_altitude_tgt=2.0,
                 heading_tgt_rad=None,
                 parent_ref=None,
                 **kwargs
                 ):

        assert (waypoint_type in VALID_WAYPOINT_TYPES), \
            'unrecognised waypoint type {} valid instructions are {}'.format(waypoint_type, VALID_WAYPOINT_TYPES)
        self.waypoint_type = waypoint_type
        self.xy_type = xy_type
        self.x_setpoint = x_setpoint
        self.y_setpoint = y_setpoint
        self.x_vel_setpoint = x_vel_setpoint
        self.y_vel_setpoint = y_vel_setpoint
        self.z_type = z_type
        self.z_setpoint = z_setpoint
        self.yaw_type = yaw_type
        self.yaw_setpoint = yaw_setpoint
        # self.pos_tol = pos_tol
        # self.tol_heading_deg = tol_heading_deg
        self.coordinate_frame = coordinate_frame
        self.wpt_typemask = get_bitmask(self.xy_type, self.z_type, self.yaw_type)
        print ('generate bitmask {} for waypoint type {} with xy_typ: {} z_type {} and yaw type: {}'
               .format(self.wpt_typemask, waypoint_type, xy_type, z_type, yaw_type))
        self.update_status_rate = update_status_rate

        super(Waypoint_state, self).__init__(
                                        flight_instruction_type=flight_instruction_type,
                                        timeout=timeout,
                                        mavros_message_node=mavros_message_node,
                                        parent_ref=parent_ref,
                                        state_label=state_label,
                                        tol_distance=pos_tol,
                                        tol_heading_deg=tol_heading_deg,
                                        **kwargs
                                        )  # sub and super class args


    def precondition_check(self):
        ''' This function can be run by substates in order - this will be executed (in places of step)
        until self.preconditions_satisfied == True
        '''
        # start test with True then run a series of tests
        self.print_wpt()

        condition = True
        # todo - check we are armed and airborne?

        # if self._ros_message_node
        self.preconditions_satisfied = condition


    def print_wpt(self):
        """
        prints debug information about this waypoint class instance
        :return:
        """

        rospy.loginfo ( ('waypoint_type {}  xy_type {} z_type {} yaw type {} mask {}'
                         .format(self.waypoint_type, self.xy_type, self.z_type, self.yaw_type, self.type_mask)) )
        rospy.loginfo ( ('setpoint x: {}  y: {} z: {} yaw: {}'
                         .format(self.x_setpoint, self.y_setpoint, self.z_setpoint, self.yaw_setpoint)) )


    def step(self):

        # when both z_vel and z_pos targets the type mask is influenced by the other field. Thus we set the either vel
        # or pos setpoints to zero depending on which is unused.  X/Y/yaw don't seem to be affected in the same way but
        # we take the same approach here for commonality / in case it is just more difficult to spot
        # should handle the axis control
        if self.xy_type == 'pos':
            self.x = self.x_setpoint
            self.y = self.y_setpoint
            self.x_vel = 0.0
            self.y_vel = 0.0
        elif self.xy_type == 'pos_with_vel':
            self.x = self.x_setpoint
            self.y = self.y_setpoint
            rospy.logwarn_throttle(2, 'the velocity capping in this state doesnae work')
            rospy.logwarn_throttle(2, 'setting x_vel {} & y_vel {} type mask is {}'
                                   .format(self.x_vel, self.y_vel, self.wpt_typemask))
            self.x_vel = self.x_vel_setpoint
            self.y_vel = self.y_vel_setpoint

        else:
            self.x = 0.0
            self.y = 0.0
            self.x_vel = self.x_setpoint
            self.y_vel = self.y_setpoint

        if self.z_type == 'pos':
            self.z = self.z_setpoint
            self.z_vel = 0.0
        else:
            self.z = 0.0
            self.z_vel = self.z_setpoint

        if self.yaw_type == 'pos':
            self.yaw = self.yaw_setpoint
            self.yaw_rate = 0.0
        else:
            self.yaw = 0.0
            self.yaw_rate = self.yaw_setpoint

        self.coordinate_frame = self.coordinate_frame
        self.type_mask = self.wpt_typemask

        rospy.loginfo_throttle(self.update_status_rate, ('waypoint_type {}  xy_type {} z_type {} yaw type {} mask {}'
                               .format(self.waypoint_type, self.xy_type, self.z_type, self.yaw_type, self.type_mask)))

        rospy.loginfo_throttle(self.update_status_rate, (
            'setpoint x: {}  y: {} z: {} yaw: {}'.format(self.x_setpoint,
                                                         self.y_setpoint,
                                                         self.z_setpoint,
                                                         self.yaw_setpoint)))
        rospy.loginfo_throttle(self.update_status_rate, (
            'cuurent x: {}  y: {} z: {} yaw: {}'.format(self._ros_message_node.local_x,
                                                        self._ros_message_node.local_y,
                                                        self._ros_message_node.local_z,
                                                        self._ros_message_node.yaw_local)))
        rospy.loginfo_throttle(self.update_status_rate, (
            'delta x: {}  y: {} z: {} yaw: {}'.format(self._ros_message_node.local_x - self.x_setpoint,
                                                        self._ros_message_node.local_y - self.y_setpoint,
                                                        self._ros_message_node.local_z - self.z_setpoint,
                                                        self._ros_message_node.yaw_local - self.yaw_setpoint)))
        if self.waypoint_type == 'pos' or self.waypoint_type == 'pos_with_vel':
            if self.waypoint_reached:
                self.stay_alive = False
        elif self.waypoint_type == 'hold':
            if self.waypoint_reached:
                rospy.loginfo_throttle(self.update_status_rate, 'at target - just wiating for timeout')


        # # todo - add exit condition - e.g. e.g. altitude target reached


class Hold_pos_state(Generic_mission_state):
    """
    A mission state that takes the aircraft's current local position and holds it for a specified amount of time

    """

    def __init__(self,
                 flight_instruction_type='Hold_position',
                 state_label='generic pos hold',                 # waypoint state labels are mandatory
                 ## might be useful to have offsets to current positions or none hold yaw behaviour in future?
                 # x_setpoint_offset,
                 # y_setpoint_offset,
                 # z_setpoint_offset,
                 # yaw_type,
                 yaw_setpoint=None,
                 heading_tgt_rad=None,
                 timeout=30,
                 mavros_message_node=None,
                 parent_ref=None,
                 **kwargs
                 ):

        super(Hold_pos_state, self).__init__(
                                        flight_instruction_type=flight_instruction_type,
                                        timeout=timeout,
                                        mavros_message_node=mavros_message_node,
                                        parent_ref=parent_ref,
                                        state_label=state_label,
                                        **kwargs
                                        )  # sub and super class args

        # if required, the yaw setpoint is stored for when the setpoint is loaded at mission time
        if yaw_setpoint:
            self.yaw_setpoint_rqd = yaw_setpoint
        else:
            self.yaw_setpoint_rqd = None
            rospy.logwarn('No yaw setpoint provided - will use current heading')

        self.type_mask = MASK_XY_POS__Z_POS_YAW_POS
        self.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    def precondition_check(self):
        ''' This function can be run by substates in order - this will be executed (in places of step)
        until self.preconditions_satisfied == True
        '''
        # start test with True then run a series of tests
        self.x_setpoint = self._ros_message_node.local_x
        self.y_setpoint = self._ros_message_node.local_y
        self.z_setpoint = self._ros_message_node.local_z

        # set velocities to 0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0
        self.yaw_rate = 0.0

        if self.yaw_setpoint_rqd:
            rospy.loginfo('Using provided setpoint {}'.format(self.yaw_setpoint_rqd))
            self.yaw_setpoint = self.yaw_setpoint_rqd
        else:
            rospy.loginfo('No yaw setpoint provided - using current heading')
            self.yaw_setpoint = np.deg2rad(self._ros_message_node.yaw_local)

        rospy.loginfo(self.return_hpt_str())
        self.preconditions_satisfied = True


    def return_hpt_str(self):
        return 'hold at: x: {}  y: {} z: {} yaw: {}'.format(self.x_setpoint, self.y_setpoint, self.z_setpoint, self.yaw_setpoint)


    def step(self):

        # when both z_vel and z_pos targets the type mask is influenced by the other field. Thus we set the either vel
        # or pos setpoints to zero depending on which is unused.  X/Y/yaw don't seem to be affected in the same way but
        # we take the same approach here for commonality / in case it is just more difficult to spot
        # should handle the axis control
        self.x = self.x_setpoint
        self.y = self.y_setpoint
        self.z = self.z_setpoint
        self.yaw = self.yaw_setpoint
        # self.type_mask = MASK_XY_POS__Z_POS_YAW_POS
        rospy.loginfo_throttle(5, self.return_hpt_str())


class Take_off_state(Generic_mission_state):
    """
    A mission state that handles the task of taking off from the ground in offboard mode.

    This will only work if the vehicle has alrteady been armed (see "Arming_state")
    """

    def __init__(self,
                 flight_instruction_type='Take_off',
                 state_label='takeoff - generic',
                 timeout=15,
                 mavros_message_node=None,
                 to_altitude_tgt=2.0,
                 heading_tgt_rad=None,
                 parent_ref=None,
                 **kwargs
                 ):


        super(Take_off_state, self).__init__(
                                        flight_instruction_type=flight_instruction_type,
                                        state_label=state_label,
                                        timeout=timeout,
                                        mavros_message_node=mavros_message_node,
                                        parent_ref=parent_ref,
                                        **kwargs
                                        )  # sub and super class args

        # todo add ROBOT_STATE_ESTIMATION environmental variable - use this to load appropriate take off procedure
        self.heading_tgt_rad = heading_tgt_rad
        self.to_altitude_tgt = to_altitude_tgt

        # the same coordinate frame is used for both phase 1 and phase 2
        self.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.settled_at_setpoint = False
        self.timer_active = False
        self.take_off_phase = TAKE_OFF_PHASE.CLEAR_THE_GROUND

        self.take_off_vel = 1.5


    def precondition_check(self):

        # check we are landed
        # if self.extended_state_ref == mavutil.mavlink.enums['MAV_LANDED_STATE']:
        # todo - if we are already airborne then passthrough this state
        # todo - if in gps mode then take an average of readings for the start x,y and z data?

        if self._prerun_complete:
            try:
                # rospy.sleep(1.0)
                self.start_x = copy(self._ros_message_node.local_x)
                self.start_y = copy(self._ros_message_node.local_y)
                self.start_z = copy(self._ros_message_node.local_z)
                if self.heading_tgt_rad is None:
                    self.heading_tgt_rad = copy(self._ros_message_node.yaw_local)
                self.preconditions_satisfied = True
                rospy.loginfo('takeoff target: {} start z: {}'.format(self.to_altitude_tgt, self.start_z))

                self.vel_ramp_time_start = rospy.Time.now().to_sec()

            except Exception as e:

                rospy.logerr_throttle(1, 'some exception {} happened in 2323jj3'.format(e))
            # get current location


    def in_range_timer_cb(self, timer_event):
        self.settled_at_setpoint = True


    @property
    def tgt_hgt(self):
        return self.start_z + self.to_altitude_tgt


    @property
    def vel_ramp_tgt(self):

        vel = (self.vel_ramp_slope * (rospy.Time.now().to_sec() - self.vel_ramp_time_start)) + self.vel_ramp_start
        # rospy.loginfo(vel)

        return np.clip(vel, -2.0, 2.0)


    def step(self):


        self.mission_state_busy = True

        # phase 1 - get off the ground
        if self.take_off_phase == TAKE_OFF_PHASE.CLEAR_THE_GROUND:

            self._ros_message_node.setpoint_lock.acquire()
            self.type_mask = MASK_XY_VEL__Z_VEL_YAW_RATE
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.z_vel = self.take_off_vel #self.vel_ramp_tgt     #self.tgt_hgt
            self.z = self.tgt_hgt
            self.yaw_rate = 0.0  # self.heading_tgt_rad
            self._ros_message_node.setpoint_lock.release()

            # once we're off the ground then go to waypoint
            if self._ros_message_node.local_z > (0.8 * self.tgt_hgt):
                rospy.loginfo('Ground cleared, height is {}'.format(self._ros_message_node.local_z))
                self.take_off_phase = TAKE_OFF_PHASE.GO_TO_WPT

        else:
            # phase 2 - go to waypoint
            self._ros_message_node.setpoint_lock.acquire()
            self.x = self.start_x
            self.y = self.start_y
            self.z = self.tgt_hgt
            self.z_vel = 0   # if we don't set z_vel to 0 then this seems
            self.yaw = self.heading_tgt_rad  # self.heading_tgt_rad
            self.type_mask = MASK_XY_POS__Z_POS_YAW_POS
            self._ros_message_node.setpoint_lock.release()

            if self.waypoint_reached:
                if not self.timer_active:
                    self.timer_active = True
                    # should the state be held for 2s, or perhaps just wait 2s after take off?
                    self.in_range_timer = rospy.Timer(period=rospy.Duration(2), callback=self.in_range_timer_cb, oneshot=True)

            else:
                if self.timer_active:
                    self.in_range_timer.shutdown()
                    self.timer_active = False

            if self.settled_at_setpoint:
                rospy.loginfo(('takeoff conditions met, local position is {} {} {} yaw {}'.format(self.x, self.y, self.z, self.yaw)))
                self.stay_alive = False

        if not self._ros_message_node.state.armed:
            rospy.logerr("can't takeoff if not armed")
            # todo - implement quit mission flag
            self.stay_alive = False
            self._parent_ref.commander.mission_fail_state = True

        self.mission_state_busy = False



class Arming_state(Generic_mission_state):
    """
    A mission state that handles the task of taking off from the ground in offboard mode
    """

    def __init__(self,
                 flight_instruction_type='Arming',
                 state_label='arming - generic',
                 timeout=60,
                 timeout_OK=False,
                 mavros_message_node=None,
                 **kwargs
                 ):

        super(Arming_state, self).__init__(
                                        flight_instruction_type=flight_instruction_type,
                                        state_label=state_label,
                                        timeout=timeout,
                                        timeout_OK=timeout_OK,
                                        mavros_message_node=mavros_message_node,
                                        **kwargs
                                        )  # sub and super class args

        # self._setpoint_raw.type_mask = MASK_ALL
        self.z_vel = -0.5    # might help to stay on the ground?
        self.z = -0.5        # might help to stay on the ground?
        self.type_mask = MASK_XY_VEL__Z_VEL_YAW_RATE   # to match takeoff state
        self.coordinate_frame = PositionTarget.FRAME_LOCAL_NED


    def precondition_check(self):

        # check we are landed
        # if self.extended_state_ref == mavutil.mavlink.enums['MAV_LANDED_STATE']:
        # todo - if we are already airborne/armed then passthrough this state

        self.mission_state_busy = True
        self.checking_pre_conditions = True

        if self._prerun_complete:

            ready_to_arm = True

            if self._ros_message_node.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                rospy.logwarn('landed_state is currently {}'.format(self._ros_message_node.extended_state.landed_state))
                ready_to_arm = False

            # todo - this is depricated, remove once tested 13.8.19
            # try:
            #     rospy.wait_for_service('mavros/set_mode', 10)
            #     # self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
            # except:
            #     ready_to_arm = False
            #
            # try:
            #     rospy.wait_for_service('mavros/cmd/arming', 10)
            #     # self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            # except:
            #     ready_to_arm = False

            try:
                rospy.wait_for_message('mavros/global_position/raw/fix', NavSatFix ,30)
                # self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            except:
                rospy.logwarn('sat nav signal not present')
                ready_to_arm = False

            self.preconditions_satisfied = ready_to_arm
            if self.preconditions_satisfied:
                rospy.loginfo('start conditions met')

        else:
            rospy.logwarn_throttle(1, 'prerun not complete for arming state')

        self.mission_state_busy = False


    def step(self, required_mode='OFFBOARD'):
        """ """

        self.mission_state_busy = True

        if self._parent_ref.robot_type == 'SITL':
            if self._ros_message_node.state.armed:
                rospy.loginfo('Armed')
                self.stay_alive = False
            else:
                if self._ros_message_node.state.mode != required_mode:
                    try:
                        # self.setting_flight_mode = True
                        rospy.loginfo('attempting to go into offboard mode')
                        rospy.loginfo('mission_state busy 1 {}'.format(self.mission_state_busy))
                        res = self._ros_message_node.set_mode_srv(0, required_mode)  # 0 is custom mode
                        rospy.sleep(1)  # we wait a while for the state to be updated in the FCU
                        # rospy.loginfo('mission_state got response {} - still busy? {}'.format(res.mode_sent, self.mission_state_busy))
                        if not res.mode_sent:
                            rospy.logwarn("failed to send mode command")
                            # self.setting_flight_mode = False
                    except Exception as e:
                        rospy.logerr('failed to set mode to offboard because '.format(e))
                        # self.setting_flight_mode = False
                        self.mission_state_busy = False

                # then attempt to arm
                if self._ros_message_node.state.mode == required_mode:
                    pass
                    if not self._ros_message_node.state.armed:
                        try:
                            rospy.loginfo('attempting to arm')
                            res = self._ros_message_node.set_arming_srv(True)
                            rospy.sleep(1) # we wait a while for the state to be updated in the FCU
                            if not res.success:
                                rospy.logerr("failed to send arm command")
                                rospy.loginfo(res)
                            else:
                                rospy.loginfo("Arm command sent")

                        except Exception as e:
                            rospy.logerr('failed to arm vehicle because '.format(e))
                            self.arming_in_progress = False
                            self.mission_state_busy = False

        elif self._parent_ref.robot_type == 'REAL':

            if self._ros_message_node.state.mode == required_mode and self._ros_message_node.state.armed:
                rospy.loginfo("Real flight controller is offboard and armed - here we go!")
                self.stay_alive = False

            else:
                rospy.loginfo_throttle(5, "Real flight controller is in mode {} and armed status is {}"
                                       .format(self._ros_message_node.state.mode, self._ros_message_node.state.armed))
        else:
            rospy.logerr('unknown robot type {}'.format(self._parent_ref.robot_type))

        self.mission_state_busy = False


class Landing_state(Generic_mission_state):
    """
    A mission state that handles the task of landing by lowering altitude in the current location at a safe rate
    """
    def __init__(self,
                 flight_instruction_type='Landing',
                 state_label='landing - generic',
                 timeout=30,
                 mavros_message_node=None,
                 heading_tgt_rad=None,
                 **kwargs
                 ):

        self.heading_tgt_rad = heading_tgt_rad

        super(Landing_state, self).__init__(
                                            flight_instruction_type=flight_instruction_type,
                                            state_label=state_label,
                                            timeout=timeout,
                                            mavros_message_node=mavros_message_node,
                                            **kwargs
                                        )  # sub and super class args

    def precondition_check(self):

        if self._prerun_complete:
            try:
                self.start_x = copy(self._ros_message_node.local_x)
                self.start_y = copy(self._ros_message_node.local_y)
                self.start_z = copy(self._ros_message_node.local_z)
                if self.heading_tgt_rad is None:
                    self.heading_tgt_rad = copy(self._ros_message_node.yaw_local)
                self.preconditions_satisfied = True
            except Exception as e:

                rospy.logerr_throttle(1, 'some exception {} happened in 2323jj3'.format(e))
            # get current location


    def step(self):

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = -0.5
        self.yaw = self.heading_tgt_rad  # self.heading_tgt_rad
        self.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.type_mask = MASK_XY_VEL__Z_VEL_YAW_POS         # since MASK_XY_POS__Z_VEL_YAW_POS doesn't seem to work

        if self._ros_message_node.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            rospy.logwarn('Landed state satisfied')
            self.stay_alive = False


class Idle_state(Generic_mission_state):

    def __init__(self,
                 flight_instruction_type='Idle',
                 state_label='Idle_state',
                 timeout=30,
                 mavros_message_node=None,
                 heading_tgt_rad=None,
                 **kwargs
                 ):

        super(Idle_state, self).__init__(
            flight_instruction_type=flight_instruction_type,
            state_label=state_label,
            timeout=timeout,
            mavros_message_node=mavros_message_node,
            **kwargs
        )  # sub and super class args

    def step(self):

        # rospy.logwarn_throttle(3, 'Mission complete - waiting for termination')
        pass


class Post_run_state(Generic_mission_state):
    """
    A mission state that shuts down ROS processes once the mission has been completed

    todo: this state is not working reliably
    """

    def __init__(self,
                 flight_instruction_type='Post run',
                 state_label='post run - generic',
                 timeout=30,
                 mavros_message_node=None,
                 heading_tgt_rad=None,
                 **kwargs
                 ):

        self.heading_tgt_rad = heading_tgt_rad

        super(Post_run_state, self).__init__(
                                            flight_instruction_type=flight_instruction_type,
                                            state_label=state_label,
                                            timeout=timeout,
                                            mavros_message_node=mavros_message_node,
                                            **kwargs
                                        )  # sub and super class args

        self.ready_to_shutdown = False


    def precondition_check(self):

        if self._prerun_complete:
            # If we are armed and landed
            if (not self._ros_message_node.state.armed) and \
                    ( self._ros_message_node.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND):
                rospy.logwarn('on ground and disarmed - shutting ros down now')

                self.ros_shutdown_timer = rospy.Timer(rospy.Duration(5), self.ros_shutdown_timer_cb, oneshot=True)
                self.preconditions_satisfied = True


    def ros_shutdown_timer_cb(self, timer_event):
        self.ready_to_shutdown = True


    def step(self):

        rospy.logwarn_throttle(3, 'Mission complete - waiting for termination')

        if self.ready_to_shutdown:
            rospy.logwarn_throttle(3, 'Shutting down conditions met - killing ROS now')
            self.stay_alive = False
            rospy.signal_shutdown("killing from {} state ".format(self.flight_instruction_type))
            sys.exit(0)