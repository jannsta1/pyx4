#!/usr/bin/env python2

from copy import copy
import rospy
import sys

class Commander(object):
    """
    A module to control the transition between different mission states.

    This is currently achieved by progressing through an ordered list of instructions. The commanders main function is
    therefore to safely keep track of which instruction is active.

    Flight states are exited:

     1) once the current flight instruction requests this
     2) once the current flight instruction's timeout duration has expired


    """

    def __init__(self,
                 flight_instructions,             # an indexed dictionary containing all the flight instructions
                 mavros_interface_node,           # an instance of a mavros interface node - our link to the FCU
                 ros_rate=50,                     # Frequency of our run method
                 commander_parent_ref=None,       # A reference to the calling node
                 start_authorised=True,
                 ):

        self.mavros_interface_node = mavros_interface_node
        self._ros_rate = ros_rate
        self.commander_parent_ref = commander_parent_ref
        self.start_authorised = start_authorised

        self._node_alive = True
        self.mission_fail_state = False
        self._flight_instructions = flight_instructions

        # self.validate_flight_instructions()
        self._mission_idx = 0

        self._flight_instruction = self._flight_instructions[self._mission_idx]
        # initialise waypoint timer
        self.wpt_timer = rospy.Timer(rospy.Duration(self._flight_instruction.timeout), self.waypoint_timeout_callback)
        self.waypoint_timeout_flag = False
        self.load_flight_instruction(increment_mission=False)


    @property
    def sp_raw(self):
        """
        Returns the current raw setpoint value
        :return:
        """
        return self._flight_instruction.sp_raw


    @property
    def mission_count(self):
        """
        How many instructions in out instruction list?
        :return:
       """
        return len(self._flight_instructions)


    @property
    def end_of_flight_instructions(self):
        '''
        checks if we are in the last command in our flight_instructions list
        :return:
        '''
        return (self.mission_idx >= (self.mission_count - 1))


    @property
    def mission_idx(self):
        return self._mission_idx


    @mission_idx.setter
    def mission_idx(self, idx):
        self._mission_idx = idx


    def load_flight_instruction(self, increment_mission=True):

        rospy.loginfo(('Attempting to load flight instruction'))
        if not self.end_of_flight_instructions:

            # todo - race condition with timeout being set while mission incremented, could this result in  a dble inc?
            self.stop_waypoint_timeout()

            current_setpoint_raw = copy(self._flight_instruction.sp_raw)
            if increment_mission:
                self.mission_idx += 1

            self._flight_instruction = self._flight_instructions[self.mission_idx]

            # give status update
            rospy.loginfo(('Starting mission instruction {} with idx {} and timeout {} '.format(
                self._flight_instruction.flight_instruction_type, self.mission_idx,  self._flight_instruction.timeout)))
            # rospy.loginfo(('Mission state sp: \n \n  {} '.format(self._flight_instruction.sp_raw)))
            rospy.loginfo(('State label: {}'.format(self._flight_instruction.state_label)))

            # start new flight instruction
            self._flight_instruction.pre_run(parent_ref=self.commander_parent_ref,
                                             ros_message_node=self.mavros_interface_node,
                                             current_sp_raw=current_setpoint_raw)

            self.start_waypoint_timeout()

            # shutdown old instruction
            if increment_mission:
                try:
                    self._flight_instructions[self.mission_idx - 1].node_alive = False
                except AttributeError as e:
                    rospy.logwarn(e)

            # publish mission in pyx4_state ROS msg
            self.commander_parent_ref.pyx4_state_msg.flight_state = self._flight_instruction.flight_instruction_type
            self.commander_parent_ref.pyx4_state_msg.state_label = self._flight_instruction.state_label
            self.commander_parent_ref.publish_pyx4_state()

        else:
            rospy.logerr('cant increment mission, at final instruction')


    def run(self):

        rate = rospy.Rate(self._ros_rate)


        while not rospy.is_shutdown() and self._node_alive:

            # check if we need to increment our mission:
            # + instruction timeout?
            # + instruction completed?
            # +

            if not self.mission_fail_state:

                if not self.end_of_flight_instructions:
                    # if our mission index is incremented - handled here if wpt, hold or timeout, elsewhere if another mission type
                    # usually this means the pyx4_base class has been inherited by another class

                    # rospy.logwarn_throttle(1, ('commander heartbeat. In state {} which is mission idx {}'.format(self._flight_instruction.flight_instruction_type, self.mission_idx)))
                    # if self.mission_idx > self.mission_idx_previous:
                    if self._flight_instruction.stay_alive == False:
                        self.load_flight_instruction(increment_mission=True)

                    # if time out then increment mission
                    if self.waypoint_timeout_flag:

                        # todo - requirement here is to shut nodes down if they didn't succeed - sometimes this could be
                        # due to timing out but sometimes timing out is OK
                        if not self._flight_instruction.timeout_OK:
                            rospy.logerr("couldn't initialise state {}".format(self._flight_instruction.flight_instruction_type))
                            self.shut_node_down()

                        rospy.logwarn(('time out of state ' + str(self._flight_instruction.state_label) +
                                       ', of type ' + str(self._flight_instruction.flight_instruction_type)))

                        self.load_flight_instruction(increment_mission=True)

                else:
                    rospy.loginfo_throttle(5, '[CMD] Mission finished, sending final instruction until shutdown')
                    if self._flight_instruction.stay_alive:
                        rospy.logwarn_throttle(10, ('[CMD] Final state satisfied - waiting for shutdown'))
                        # if self.mavros_interface_node.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                        # if self.mavros_interface_node.state.armed != State.armed:
                    else:
                        self.shut_node_down()

                # if the mission_state has finished its previous iteration, then start next iteration
                if not self._flight_instruction.mission_state_busy:

                    if self._flight_instruction.preconditions_satisfied:
                        self._flight_instruction.step()
                    else:
                        if self._flight_instruction._prerun_complete:
                            rospy.loginfo('running precondition_check ')
                            self._flight_instruction.precondition_check()
                        else:
                            rospy.logerr('prerun not completeted for {}'.format(self._flight_instruction.flight_instruction_type))

                else:
                    rospy.logdebug('mission state is busy - not polling')

                # prevent garbage in console output when thread is killed
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass

            else:
                rospy.logerr('mission fail state reported - exit offboard mode')
                # todo - might be better to try and send zero setpoints in this state
                self.shut_node_down()

        self.shut_node_down()


    def start_waypoint_timeout(self):
        try:
            self.wpt_timer = rospy.Timer(rospy.Duration(self._flight_instruction.timeout), self.waypoint_timeout_callback)
        except AttributeError:
            rospy.logwarn('no timeout specified for this waypoint')


    def stop_waypoint_timeout(self):
        self.wpt_timer.shutdown()  # shutdown previous timer to make sure this doesn't cause an early timeout
        self.waypoint_timeout_flag = False


    def waypoint_timeout_callback(self, unused_but_required):
        '''
        we set this flag to true if a setpoint has not been reached after the specified duration (self.timeout_time)
        :return:
        '''
        self.waypoint_timeout_flag = True
        rospy.loginfo('timer ending')


    def shut_node_down(self):
        self._node_alive = False
        rospy.logwarn('Commander - shutting down')
        # rospy.sleep(2.0)
        sys.exit()

