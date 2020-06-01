#!/usr/bin/env python2

from __future__ import division

from enum import Enum
from threading import Thread
from pymavlink import mavutil

import rospy
import mavros
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, OpticalFlowRad
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from geometry_msgs.msg import PoseStamped
# from waypoints_local import *
# from waypoints_local import *

from sensor_msgs.msg import NavSatFix, Image, CameraInfo, Range
from tf.transformations import euler_from_quaternion
import os
import sys
import argparse

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

from definitions_pyx4 import *
from mission_states import *

# enums # todo - would named tuples be better?


# non class specific functions
def merge_two_dicts(x, y):
    z = x.copy()  # start with x's keys and values
    z.update(y)  # modifies z with y's keys and values & returns None
    return z

# todo - get global poisition of drone on startup
# todo - maybe its better to replace the flight_instructions dictionary with a collection of classes
class Pyx4_deprecated(object):
    def __init__(self,
                 mission_file_path,
                 mavros_ns="",
                 # used if mavros is requied to belong to a higher order namespace (e.g. multi-drone simulation
                 main_loop_freq=30,
                 sem=State_estimation_method.MOCAP,  # state estimation method
                 tol_heading_deg=3,
                 camera_pose_topic_name="/bee/camera_pose",
                 ):

        self.node_alive = True

        self.mavros_ns = mavros_ns
        self.main_loop_freq = main_loop_freq
        self.sem = sem
        self.hardware_type = Hardware.UNKNOWN
        self.tol_heading_deg = tol_heading_deg
        self.mission_file_path = mission_file_path

        self.camera_pose_topic_name = camera_pose_topic_name       # todo - is this required?

        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.optic_flow_raw = OpticalFlowRad()
        self.optic_flow_range = Range()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None
        self.mocap_pose = PoseStamped()
        self.camera_pose = PoseStamped()
        self.camera_yaw = None




        if isinstance(self.mission_file_path, (basestring, str)):
            # Get our mission
            file_path = self.mission_file_path # "/home/jan/catkin_packages/dfdrone/data/missions/basic_wpts.csv"
            # todo add offset x,y,z functionality
            self.flight_instructions = Wpts_from_csv(self.mission_file_path)
        elif isinstance(self.mission_file_path, (dict)):
            # if isinstance(self.mission_file_path[0], flight_instruction):
            self.flight_instructions = self.mission_file_path

        self.validate_flight_instructions()
        self.mission_idx = 0
        self.flight_instruction = self.flight_instructions[self.mission_idx]
        # if self.flight_instruction.instruction_type == 'pos':
        self.sp_raw = self.flight_instruction.sp_raw
        self.mission_count = len(self.flight_instructions)

        # create a watchdog thread that checks topics are being received at the expected rates
        self.watchdog_thread = Thread(target=self.watchdog, args=())
        self.watchdog_thread.daemon = True
        self.watchdog_thread.start()

        # create a thread that publishes the setpoint
        self.sp_pub_thread = Thread(target=self.sp_pub, args=())
        self.sp_pub_thread.daemon = True
        self.sp_pub_thread.start()

        # create a thread that checks the proximity of the quad to a waypoint
        # self.wp_check_thread = Thread(target=self.wp_check, args=())
        # self.wp_check_thread.daemon = True
        # self.wp_check_thread.start()



    #
    # Properties
    #
    @staticmethod
    def pose2yaw(this_pose):
        orientation_q = this_pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    # @property
    # def yaw2headingXY():

    @staticmethod
    def quat2yaw(this_quat):
        '''
        converts from a quaternion (developed using message type: posestamped.pose.orientation) to a yaw
        we assume that the UAV is level enough for this to work
        :param this_quat: 
        :return: 
        '''
        (_, _, yaw) = euler_from_quaternion([this_quat.x, this_quat.y, this_quat.z, this_quat.w])
        return yaw

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
    def yaw(self):
        return self.quat2yaw(self.local_position.pose.orientation)

    @property
    def sp_error_yaw(self):
        # todo - is there already a tf function for this?
        return self.heading_error_rad(self.yaw, self.sp_raw.yaw)

    @property
    def sp_error_xyz(self):
        # todo - is there already a tf function for this?
        a = np.array((self.flight_instruction.sp_raw.position.x, self.flight_instruction.sp_raw.position.y,
                      self.flight_instruction.sp_raw.position.z))
        b = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y,
                      self.local_position.pose.position.z))
        distance = np.linalg.norm(a - b).round(decimals=3)
        return distance

    @property
    def waypoint_reached(self, tol_distance=0.2):
        '''
        
        :return: 
        '''

        # todo 1, parse the tolorences from mission and add ability to set a generic default for all waypoints of a mission
        # todo 2, add a timer so that the time that a waypoint has been held for can be measured

        return ((self.sp_error_xyz) < tol_distance and (self.sp_error_yaw < np.deg2rad(self.tol_heading_deg)))

    @property
    def end_of_flight_instructions(self):
        '''
        checks if we are in the last command in our flight_instructions list
        :return: 
        '''
        return (self.mission_idx >= self.mission_count)

    # def heading_error_rad(self, x, y):
    #     '''
    #     returns the heading error of the UAV in radians
    #     :param x:
    #     :param y:
    #     :return:
    #     '''
    #
    #     return min((2 * np.pi) - abs(x - y), abs(x - y))

    def validate_flight_instructions(self):
        '''
        cycles through current flight instructions dictionary and validates them
        :return: 
        '''

        for key in self.flight_instructions:

            self.validate_flight_instruction(instruction = self.flight_instructions[key])

    def validate_flight_instruction(self, instruction):
        '''
        checks whether a flight instruction has a corresponding method and (ideally) any associated mandatory information
        :param flight_idx: 
        :return: 
        '''
        instruction_valid = False
        if instruction.instruction_type == 'wpt' or instruction.instruction_type == 'hold':
            instruction_valid = True
            # todo - check for nans etc

        # if self.flight_instructions[flight_idx].instruction_type
        if not instruction_valid:
            if hasattr(self.__class__, instruction.instruction_type):

                if callable(getattr(self.__class__, instruction.instruction_type)):
                    instruction_valid = True
            else:
                rospy.logerr(('instruction error - no method in {} called {}: '.format(self.__class__.__name__, instruction.instruction_type)))

        if not instruction_valid:

            raise (('this instruction type {} will break this module, aborting'.format(instruction.instruction_type)))

    def insert_new_instruction(self, instruction_idx, instruction2insert):
        '''
        attempts to find problems with instructions, idea is to do this prior to the mission rather than during 
        :param instruction_idx: 
        :param instruction2insert: 
        :return: 
        '''

        # todo - add "insert_instruction_before", which inserts an instruction before a particular type of instruction
        self.validate_flight_instruction(instruction2insert)
        # ensure greater than current mission idx:
        #todo - can we add a lock to make sure that mission_idx isn't incremented while this method runs?
        if instruction_idx > self.mission_idx:
            # pass through dictionary in reverse order to ensure space is available
            total = len(self.flight_instructions)
            for idx in np.arange(total, instruction_idx, -1): # todo - check against corner cases, e.g. instruction_idx = 0
                self.flight_instructions[idx] = self.flight_instructions[idx-1]
            self.flight_instructions[instruction_idx] = instruction2insert
        else:
            rospy.logerr('appending here will have no effect')

    def replace_instruction_by_idx(self, instruction_idx, instruction2insert):
        '''
        overites flight instruction at idx
        :param instruction_idx:
        :param instruction2insert:
        :return:
        '''

        # todo - a function that replaces an instruction with a string name
        self.validate_flight_instruction(instruction2insert)
        assert (len(self.flight_instructions) >= instruction_idx), 'requested replace instruction idx greater than mission idxs'
        self.flight_instructions[instruction_idx] = instruction2insert


    def print_mission_details(self):
        '''
        cycles through current flight instructions dictionary and prints them
        :return: 
        '''
        print('printing mission waypoints')
        for key, vals in enumerate(self.flight_instructions):
            self.flight_instructions[key].print_wpt_info()
        print('End of mission waypoints')

    def increment_mission(self):
        '''
        Moves our mission idx to the next value
        :return: 
        '''
        # todo - add check to see if the number is valid? - maybe set flag when last instruction recahed?
        if not self.end_of_flight_instructions:
            # todo - add mission names to this line: i.e. incrementing mission idx from XX (state XX) to ... XX (state XX)
            rospy.logdebug(('incrementing mission idx from ' + str(self.mission_idx) + ' to ' + str(self.mission_idx + 1)))
            self.mission_idx = self.mission_idx + 1
        else:
            rospy.logerr('At final mission idx {} blocking increment'.format(self.mission_idx))


    ###########################################
    # ROS callback functions
    ###########################################
    def altitude_callback(self, data):
        self.altitude = data

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))
        self.extended_state = data

    def global_position_callback(self, data):
        self.global_position = data

    def optic_flow_raw_callback(self, data):
        self.optic_flow_raw = data

    def optic_flow_range_callback(self, data):
        self.optic_flow_range = data

    def home_position_callback(self, data):
        self.home_position = data

    def local_position_callback(self, data):
        self.local_position = data
        self.local_x = self.local_position.pose.position.x
        self.local_y = self.local_position.pose.position.y
        self.local_z = self.local_position.pose.position.z
        # self.local_pos = np.array((self.local_x, self.local_y, self.local_z))

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))
        self.mission_wp = data

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                    'MAV_STATE'][data.system_status].name))
        self.state = data

    def mocap_pos_callback(self, data):
        self.mocap_pose = data

    def cam_pose_cb(self, this_pose):
        self.camera_pose = this_pose
        self.camera_yaw = self.pose2yaw(this_pose=self.camera_pose)

    def start_waypoint_timeout(self):
        try:
            self.wpt_timer = rospy.Timer(rospy.Duration(self.flight_instruction.timeout), self.waypoint_timeout)
        except AttributeError:
            rospy.logwarn('no timeout specified for this waypoint')

    def stop_waypoint_timeout(self):
        self.wpt_timer.shutdown()  # shutdown previous timer to make sure this doesn't cause an early timeout
        self.waypoint_timeout_flag = False

    def waypoint_timeout(self, unused_but_required):
        '''
        we set this flag to true if a setpoint has not been reached after the specified duration (self.timeout_time)
        :return: 
        '''
        self.waypoint_timeout_flag = True
        rospy.loginfo('waypoint timer ending')

    # def get_sp_xyz(self):
    #
    #     return np.array((self.sp_raw.position.x, self.sp_raw.position.y, self.sp_raw.position.z))
    #
    # def get_sp_yaw(self, pos_setpoint_raw):
    #     return pos_setpoint_raw.yaw

    # def get_type_mask(self, pos_setpoint_raw):
    #     return pos_setpoint_raw.type_mask

    # def waypoint_status_string(self, mask_value, distance_to_target=8888, heading_error_rad=666):
    #     '''
    #     function used to generate string that reports status relating to reaching the waypoint
    #     :param mask_value:
    #     :param distance_to_target:
    #     :param heading_error_rad:
    #     :return:
    #     '''
    #
    #     status_str = 'Waypoint is: ' + str([self.sp_x, self.sp_y, self.sp_z])
    #
    #     # check if using XY position coordinates
    #     if mask_value & PositionTarget.IGNORE_PY == 0:
    #         status_str = status_str + 'Distance from waypoint: ' + str(distance_to_target) + 'm'
    #
    #     if mask_value & PositionTarget.IGNORE_YAW == 0:
    #         status_str = status_str + ' Heading error: ' + str(np.rad2deg(heading_error_rad).round(2)) + ' degrees'
    #
    #     status_str = str(status_str) + ' Position: ' + str(self.local_pos.round(2))
    #
    #     return status_str
    def take_off(self):
        """ This method is started in a thread by the invoke_instruction method.  """
        # todo - need to add locks to variables used in threads !!!!!
        rospy.logwarn('TAKE OFFFFFFFFFFFFFFFFFFFFFFFFFFFF !!!!!!')

        to = Take_off_state(
                            mission_idx_ref=self.mission_idx,
                            flight_instruction_ref=self.flight_instruction,
                            extended_state_ref=self.extended_state,
                            local_position_x_ref=self.local_x,
                            local_position_y_ref=self.local_y,
                            local_position_z_ref=self.local_z,
                            current_yaw_ref=self.yaw,
                            current_sp_raw=self.sp_raw,  # todo - is this required?
                            # mission_vel_tgt=self.mission_xy_tgt_m_s,
                            # mission_z_tgt=self.mission_z_tgt_m,
                            )
        to.main_loop()
        if to.our_mission_idx == self.mission_idx:
            self.increment_mission()
        rospy.logwarn('TAKE OFF !!!!! FINISHED')


    def land(self):

        # todo - need to add locks to variables used in threads !!!!!
        rospy.logwarn('LAAAAAAAAAAAANDING !!!!!')
        rospy.logwarn((id(self.extended_state), id(self.flight_instruction.sp_raw.velocity.x)))
        la = Landing_state(
                            mission_idx_ref=self.mission_idx,
                            flight_instruction_ref=self.flight_instruction,
                            extended_state_ref=self.extended_state,
                            local_position_x_ref=self.local_x,
                            local_position_y_ref=self.local_y,
                            local_position_z_ref=self.local_z,
                            current_yaw_ref=self.yaw,
                            current_sp_raw=self.sp_raw,  # todo - is this required?
                            )
        la.main_loop()
        if la.our_mission_idx == self.mission_idx:
            self.increment_mission()
        rospy.logwarn('LAAAAAAAAAAAANDING !!!!! FINISHED')

    def invoke_instruction(self):
        # if its not a waypoint ('wpt') state then we expect self.sp_raw to be determined elsewhere
        self.flight_instruction = self.flight_instructions[self.mission_idx]
        rospy.loginfo(('Starting mission instruction {} with timeout {} '.format(
            self.flight_instruction.instruction_type, self.flight_instruction.timeout)))
        self.mission_idx_previous = self.mission_idx

        if self.flight_instruction.instruction_type == 'wpt' or self.flight_instruction.instruction_type == 'hold':
            self.sp_raw = self.flight_instruction.sp_raw
            if self.flight_instruction.instruction_type == 'wpt':
                rospy.loginfo(('New setpoint is x:{:.04g} y:{:.04g} z:{:.04g} yaw:{:.04g}'.format(
                    self.sp_raw.position.x, self.sp_raw.position.y, self.sp_raw.position.z, self.sp_raw.yaw)))
            else:
                rospy.loginfo(('New setpoint is x:{:.04g} y:{:.04g} z:{:.04g} yaw:{:.04g}'.format(
                    self.sp_raw.velocity.x, self.sp_raw.velocity.y, self.sp_raw.velocity.z, self.sp_raw.yaw)))
        else:
            print('Starting class specific flight instruction: {}'.format(self.flight_instruction.instruction_type))
            self.this_instruction_thread = Thread(target=getattr(self, self.flight_instruction.instruction_type), args=())
            self.this_instruction_thread.daemon = True
            self.this_instruction_thread.start()


        # set timeout if its there
        self.stop_waypoint_timeout()
        self.start_waypoint_timeout()

    def sp_pub(self):
        '''
        A thread that ensures our waypoint is sent perdiodically
        :return: 
        '''
        # initialise waypoint timer
        self.wpt_timer = rospy.Timer(rospy.Duration(8888), self.waypoint_timeout)
        self.waypoint_timeout_flag = False
        rate = rospy.Rate(100)
        self.mission_idx_previous = self.mission_idx
        last_condition_met = False

        # invoke our first instruction
        self.invoke_instruction()

        # poll for mission increments, if mission increment detected, manage transition to next state accordingly
        while not rospy.is_shutdown() and self.node_alive:

            if not self.end_of_flight_instructions:
                # if our mission index is incremented - handled here if wpt, hold or timeout, elsewhere if another mission type
                # usually this means the pyx4_base class has been inherited by another class
                if self.mission_idx > self.mission_idx_previous:
                    self.invoke_instruction()

            else:
                rospy.loginfo_throttle(5, 'Mission finished, sending final instruction until shutdown')

            # SEND SETPOINT - we always try this
            try:
                self.flight_instruction.sp_raw.header.stamp = rospy.Time.now()
                self.local_pos_pub_raw.publish(self.flight_instruction.sp_raw)
            except Exception as e:
                rospy.logerr_throttle(1, ('couldnt publish the setpoint message because: ', e))

            # if our waypoint is reached then increment mission instruction
            if self.flight_instruction.instruction_type == 'wpt':
                # if we are close enough to the setpoint then increment mission_idx
                # todo - replace this check with a function or property (so that this can be called from elsewhere
                # todo (cont) - perhaps split into distance and heading. Also, add a timer to say how long the condition has been satisfied
                if self.waypoint_reached:
                    if not self.end_of_flight_instructions:
                        rospy.loginfo(('waypoint reached ', self.sp_error_xyz))
                        self.increment_mission()
                    else:
                        last_condition_met = True

            # Check our timeout condition
            # if self.flight_instruction.instruction_type == 'hold' or self.flight_instruction.instruction_type == 'wpt':
            if self.waypoint_timeout_flag:
                if not self.end_of_flight_instructions:
                    rospy.logwarn(('time out of state ', self.flight_instruction.instruction_type, self.flight_instruction.state_label))
                    self.increment_mission()
                else:
                    last_condition_met = True
                self.stop_waypoint_timeout()

            if self.end_of_flight_instructions:
                if last_condition_met:
                    rospy.logwarn_throttle(10, ('Final state satisfied - waiting for shutdown'))

                    if self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                        if self.state.armed != State.armed:
                            self.shut_node_down()

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def watchdog(self):

        # todo: consider monitoriing the following topics
        # gps_topics = ['global_pos', 'home_pos', 'mission_wp']
        # vision_topics = ['vision_pose']
        # main_topics = ['alt', 'ext_state', 'local_pos', 'state']

        #
        rate = rospy.Rate(1.1)  # todo - different rates for different messages?

        self.wd_initialised = False
        self.wd_fault_detected = False
        self.fault_this_loop = False

        time_last_msg = rospy.get_time()
        while not rospy.is_shutdown() and self.node_alive:

            time_this_run = rospy.get_time()
            time_delta = time_this_run - time_last_msg
            time_last_msg = time_this_run

            #
            self.fault_this_loop = False
            try:
                if (self.state.header.stamp.secs + 2.5) < rospy.get_time():
                    rospy.logwarn(('not receiving state message with time interval ', time_delta))
                    self.fault_this_loop = True
            except:
                self.fault_this_loop = True

            try:
                if (self.local_position.header.stamp.secs + 2.0) < rospy.get_time():
                    rospy.logwarn(('not receiving local position message with time interval ', time_delta))
                    self.fault_this_loop = True
            except:
                self.fault_this_loop = True

            if self.sem is State_estimation_method.MOCAP:
                try:
                    if (self.mocap_pose.header.stamp.secs + 2.0) < rospy.get_time():
                        rospy.logwarn(('not receiving mocap message with time interval ', time_delta))
                        self.fault_this_loop = True
                except:
                    self.fault_this_loop = True

            if self.sem is State_estimation_method.GPS:
                try:
                    if (self.global_position.header.stamp.secs + 2.0) < rospy.get_time():
                        rospy.logwarn(('not receiving global position message with time interval ', time_delta))
                        self.fault_this_loop = True
                except:
                    self.fault_this_loop = True

            if self.sem is State_estimation_method.OPTIC_FLOW:
                try:
                    if (self.optic_flow_raw.header.stamp.secs + 2.0) < rospy.get_time():
                        rospy.logwarn((self.optic_flow_raw.header.stamp.secs + 2.0), rospy.get_time())
                        rospy.logwarn(('not receiving optic flow message with time interval ', time_delta))
                        self.fault_this_loop = True
                except:
                    self.fault_this_loop = True

            if self.wd_initialised and self.fault_this_loop:
                self.wd_fault_detected = True

            # watchdog is initised when all conditions are met
            if not self.wd_initialised and not self.fault_this_loop:
                rospy.loginfo('enabling watchdog')
                self.wd_initialised = True

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                                 desired_landed_state].name, index))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        rospy.loginfo(
            "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
                format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                           desired_landed_state].name, mavutil.mavlink.enums[
                           'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                       index, timeout))

    def check_if_sitl(self, timeout=10, loop_freq=1):
        # get fcu_url before continuing
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            try:
                # todo - use the rosparam proxy instead
                fcu_url = rospy.get_param(self.mavros_ns + 'mavros/fcu_url')

                if ('localhost' in fcu_url) or ('127.0.0.1' in fcu_url):
                    self.hardware_type = Hardware.SITL
                else:
                    self.hardware_type = Hardware.PIXHAWK

                rospy.loginfo("hardware type confirmed {0} | seconds: {1} of {2}".
                              format(self.hardware_type, i / loop_freq, timeout))

                break
            except:
                rospy.logwarn(('FCU parameter not yet set, will try again in ' + str(
                    ()) + ' seconds, searching under namespace: ', self.mavros_ns))
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def set_mode(self, mode, timeout):

        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        # rospy.logerr("failed to send mode command")
                        rospy.logwarn("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

                # self.assertTrue(mode_set, (
                #     "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
                #         format(mode, old_mode, timeout)))

    def preflight_checks(self):
        '''

        :return: 
        '''

        # self.wait_for_topics(timeout=20)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)
        self.check_if_sitl(timeout=10)

        # self.check_state_estimation()

    def set_arm(self, arm, timeout):

        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        return arm_set

        # self.assertTrue(arm_set, (
        #     "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
        #         format(arm, old_arm, timeout)))

    ################################################
    # Mission method
    ################################################
    def run(self):

        # todo - what rate should this run at?
        # todo - perhaps we can broadcast sp here?
        """Test offboard attitude control"""

        # # make sure the simulation is ready to start the mission
        self.preflight_checks()
        rate = rospy.Rate(self.main_loop_freq)

        # arm quadcopter
        self.set_mode('OFFBOARD', 50)
        start_decision = self.set_arm(True, 5)

        if start_decision:
            while not rospy.is_shutdown() and self.node_alive:
                if self.wd_fault_detected:
                    rospy.logerr(("Watchdog test Failed, exiting "))
                    # todo - set some default behaviour here? e.g. send home position
                    break
                elif self.wd_initialised:
                    rospy.loginfo_throttle(30, 'pyx4_base program heart beat')
                    try:
                        rate.sleep()
                    except rospy.ROSException as e:
                        self.fail(e)
                        # self.set_mode("AUTO.LAND", 5)
                        # self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,  90, 0)
                        # self.set_arm(False, 5)
                else:
                    rospy.loginfo_throttle(2, ("waiting for watchdog to initialise"))
        else:
            rospy.logerr('couldnt arm quad, aborting mission')

    def shut_node_down(self):
        self.node_alive = False
        rospy.logwarn('waiting 2s and shutting down')
        # rospy.sleep(2.0)
        sys.exit()


    def fail(self, e=None):
        rospy.logwarn(('PYX4 error is :', e))


def var2mission_filepath(var_in, path_in=MISSION_SPECS):

    # check if has .csv extension
    if not var_in.lower().endswith(('.csv')):
        var_in = var_in + '.csv'
        print var_in, '1'

    if not os.path.isabs(var_in):
        var_in = os.path.join(path_in, var_in)
        print var_in, '2'

    if os.path.isfile(var_in):
        print var_in, '3'
        return var_in
    else:
        rospy.logerr(('coundt find file {}'.format(var_in)))


if __name__ == '__main__':

    rospy.init_node('test_node', anonymous=True, log_level=rospy.INFO)
    parser = argparse.ArgumentParser(description="This node is a ROS side mavros based state machine.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace",
                        default='mavros')  # JS - changed as mavros.DEFAULT_NAMESPACE doesn't seem to exist anymore - todo - is there a new enum for this?
    parser.add_argument('-f', '--mission-file', help="which mission .csv file we will use to outline our mission", default='basic_wpts')
    parser.add_argument('-s', '--state-estimation-type', help="which sensing modality is used for state estimation", default='GPS')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # todo - add arg parse to facilitate inputting different missions from the command line
    # pyx4_base = Pyx4_deprecated(mission_file_path=EXAMPLE_MISSION, sem=State_estimation_method.GPS)
    pyx4 = Pyx4_deprecated(mission_file_path=var2mission_filepath(args.mission_file),
                           sem=State_estimation_method[args.state_estimation_type.upper()])
    pyx4.run()

    #
    # lf = Learning_flight(lf_imsave_folder=LF_SAVE_IMAGES_FOLDER)
    # lf.run()

