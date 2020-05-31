#!/usr/bin/env python2
"""
This module continuously provides mavros with a setpoint at the required rate so that onboard computer doesn't loose
sync with the flight control unit
"""
from __future__ import division

import rospy

def setpoint_publisher(mavros_interface_node, commander_class_instance, ros_rate=100):
    """
    This method continuously publishes the setpoint state - must run at a deterministic rate to prevent offboard mode
    from exiting (offboard mode exits if a new instruction is not received at a minimum of 2 hz)

    """
    rate = rospy.Rate(ros_rate)
    while not rospy.is_shutdown():
        try:

            # todo - add this thread lock to mission_states
            mavros_interface_node.setpoint_lock.acquire()
            mavros_interface_node.local_pos_pub_raw.publish(commander_class_instance.sp_raw)
            mavros_interface_node.setpoint_lock.release()

        except Exception as e:
            rospy.logerr_throttle(1, ('couldnt publish the setpoint message because: ', e))

        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            pass