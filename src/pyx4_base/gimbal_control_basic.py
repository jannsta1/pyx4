#!/usr/bin/env python3

import argparse
import sys
import rospy
import numpy as np
from mavros_msgs.msg import ActuatorControl
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class Gimbal_controller():
    """
    An experimental module for sending gimbal commands. It has been designed for use with the PX4_SITL
    gimbal_controller plugin but could also work with hardware if the correct physical connections are made


    """

    def __init__(self,
                 roll_degs,
                 pitch_degs,
                 yaw_degs,
                 ):

        self.message_pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=10)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)

        self.actuator_control_message = ActuatorControl()
        self.seq = 0
        self.local_position = PoseStamped()
        self.heading = 0.0
        self.height = 0.0

        self.roll_rads = np.deg2rad(roll_degs)
        self.pitch_rads = -0.5 #np.derad(-.0) #np.deg2rad(pitch_degs)
        self.yaw_rads = 0.0 # np.deg2rad(0.0) #np.deg2rad(yaw_degs)


    @property
    def inputs(self):
        # todo - better way to handle height - currently this is needed or simulation crashes but hopefully there is a fix for this - lift camera up?
        if self.height < 0.5:
            return np.zeros((8))
        else:
            return np.array((self.roll_rads, self.pitch_rads, self.yaw_rads, 0.0, 0.0, 0.0, 0.0, 0.0))

    def local_position_callback(self, data):
        self.local_position = data
        self.height = self.local_position.pose.position.z
        self.heading = self.quat2yaw(data.pose.orientation)

    def quat2yaw(self, this_quat):
        '''
        converts from a quaternion (developed using message type: posestamped.pose.orientation) to a yaw
        we assume that the UAV is level enough for this to work
        :param this_quat:
        :return:
        '''
        (_, _, yaw) = euler_from_quaternion([this_quat.x, this_quat.y, this_quat.z, this_quat.w])
        return yaw

    def run(self):

        r =rospy.Rate(1)
                        #   r    p    y?


        while not rospy.is_shutdown():

            self.actuator_control_message.header.stamp = rospy.Time.now()
            self.actuator_control_message.header.seq = self.seq
            self.actuator_control_message.group_mix = 2 # ActuatorControl.PX4_MIX_PAYLOAD
            # if self.seq % 50 == 0:
            #     # inputs = np.roll(inputs, 1)
            #     rospy.loginfo(self.inputs)
            # rospy.loginfo_throttle(3, self.inputs)
            self.actuator_control_message.controls = self.inputs

            self.message_pub.publish(self.actuator_control_message)

            self.seq = self.seq + 1
            r.sleep()


if __name__ == '__main__':

    rospy.init_node('gimbal_test', anonymous=True, log_level=rospy.INFO)

    parser = argparse.ArgumentParser(description="Gimbal node parser.")

    parser.add_argument('-r', '--roll', help="target height for drone during mission", type=float, default=0.0)
    parser.add_argument('-p', '--pitch', help="target height for drone during mission", type=float, default=0.0)
    parser.add_argument('-y', '--yaw', help="target height for drone during mission", type=float, default=0.0)

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    gt = Gimbal_controller(roll_degs=args.roll, pitch_degs=args.pitch, yaw_degs=args.yaw)
    gt.run()