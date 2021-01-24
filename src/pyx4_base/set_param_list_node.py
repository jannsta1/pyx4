#!/usr/bin/env python3

"""
This module is used to set a list of PX4 parameters

"""


from mavros_msgs.msg import ParamValue

from pyx4_base.mavros_interface import *
from pyx4_base.definitions_pyx4 import *


class Px4_param_setter(object):
    def __init__(self,
                 rospy_rate=0.2,
                 mavros_ns='',
                 state_estimation_mode=State_estimation_method.GPS,
                 enforce_height_mode_flag=False,
                 height_mode_req=0,
                 enforce_sem_mode_flag=False,
                 params_dict=None,
                 ):

        self.node_alive = True

        self.params_dict = params_dict


        self.mavros_ns = mavros_ns
        self._run_rate=rospy_rate
        self.state_estimation_mode = state_estimation_mode
        self.enforce_height_mode_flag = enforce_height_mode_flag
        self.height_mode_req = height_mode_req

        # start mavros interface thread
        self.mavros_interface = Mavros_interface(
                                                state_estimation_mode=self.state_estimation_mode,
                                                enforce_height_mode_flag=self.enforce_height_mode_flag,
                                                height_mode_req=self.height_mode_req
                                                )
        self.mavros_interface_thread = Thread(target=self.mavros_interface.run, args=())
        self.mavros_interface_thread.daemon = True
        self.mavros_interface_thread.start()

        wait_for_watchdog_rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.mavros_interface.wd_initialised:
            rospy.loginfo('Waiting for mavros interface to inilialise')   # todo - report what is preventing watchdog from completing
            wait_for_watchdog_rate.sleep()
        rospy.loginfo('Mavros interface initialised')

        self.watchdog_timer = rospy.Timer(rospy.Duration(200), self.watchdog_timeout_cb)

    def watchdog_timeout_cb(self):
        rospy.logerr('watchdog timeout triggered')
        self.node_alive = False
        sys.exit()

    def run(self):
        """ Keep nodes alive until exit conditions present """

        rate = rospy.Rate(self._run_rate)

        while not rospy.is_shutdown() and self.mavros_interface._node_alive and self.node_alive:

            # first check which items need to be changed
            self.incorrect_params_dict = {}
            for param, val in self.params_dict.items():
                # rospy.loginfo((param, val))

                param_read_attempts = 0
                while param_read_attempts < 5:
                    res = self.mavros_interface.get_param_srv(param)
                    print(res)
                    if res.success:
                        # todo - if res == false then: try again? or just add to list
                        eeprom_val = res.value.integer
                        if val != eeprom_val:
                            self.incorrect_params_dict[param] = ParamValue(val, val)
                            rospy.loginfo(
                                'param {} is currently set to {} not {} - adding to dictionary'.format(param, eeprom_val, val))
                        else:
                            rospy.loginfo('param {} is already set to {} not adding to dictionary'.format(param, val))
                        break
                    else:
                        rospy.logwarn("Couldn't read param {} on attempt {} - trying again".format(param, param_read_attempts))
                        rospy.sleep(1)

                    param_read_attempts += 1

            # if no items need to be changed then exit loop
            if len(self.incorrect_params_dict) == 0:
                rospy.loginfo('All params correct - finishing up')
                break
            else:
                for param, val in self.incorrect_params_dict.items():
                    rospy.loginfo(('attempting to change {} to {}'.format(param, val.integer)))

                    param_write_attempts = 0
                    while param_write_attempts < 5:
                        res = self.mavros_interface.set_param_srv(param, val)
                        if res.success:
                            rospy.loginfo('result for {} is: {}'.format(param, res.success))
                            break
                        else:
                            rospy.logerr("Couldn't read param {} on attempt {} - trying again".format(param, param_write_attempts))
                            rospy.sleep(1)
                        param_write_attempts += 1

            rate.sleep()
        rospy.loginfo('Finished setting params')


if __name__ == '__main__':


    rospy.init_node('pyx4_param_setter_node', anonymous=True, log_level=rospy.DEBUG)


    # todo - need a way to seperate this from cx_model
    args = cx_argparse(sys_argv_in=sys.argv)

    # params_dict = {'EKF2_HGT_MODE': 2}
    params_dict = {'EKF2_HGT_MODE': args.height_mode_req}

    # params_list = []
    # ParamValue('EKF2_HGT_MODE', integer=1)


    pyx4 = Px4_param_setter(
                   enforce_height_mode_flag=False,
                    params_dict=params_dict,
                   )
    pyx4.run()