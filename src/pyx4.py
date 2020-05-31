#!/usr/bin/env python2

from __future__ import division

from setpoint_publisher import *
from mavros_interface import *
from definitions_pyx4 import *
from mission_states import *
from threading import Thread
from commander import *

from lfdrone.msg import pyx4_state as Pyx4_msg

# todo - re-add state estimation options
# todo - create a message that is published when the flight state changes
class Pyx4(object):
    """
    A class for controlling a px4 flight control unit via ROS (using the mavros package).

    NB. Mavros must be started elsewhere for this node to initialise properly

    """
    def __init__(self,
                 flight_instructions,
                 node_name='pyx4_node',
                 rospy_rate=2,
                 mavros_ns='',
                 state_estimation_mode=State_estimation_method.GPS,
                 enforce_height_mode_flag=False,
                 height_mode_req=0,
                 enforce_sem_mode_flag=False,
                 start_authorised=True,
                 ):

        self.node_alive = True

        self.mavros_ns = mavros_ns
        self._run_rate=rospy_rate
        self.state_estimation_mode = state_estimation_mode
        self.enforce_height_mode_flag = enforce_height_mode_flag
        self.height_mode_req = height_mode_req
        self.start_authorised = start_authorised

        # create pyx4 state publisher
        self.pyx4_state_msg_pub = rospy.Publisher(node_name + '/pyx4_state', Pyx4_msg, queue_size=5)
        self.pyx4_state_msg = Pyx4_msg()
        self.pyx4_state_msg.flight_state = 'Not_set'
        self.pyx4_state_msg.state_label = 'Not_set'

        # get robot type - it is essential that this environmental variable is written so that we know is its a robot
        # or a simulation
        try:
            self.robot_type = os.environ['ROBOT_TYPE']
        except Exception as e:
            rospy.logerr("couldn't find mandatory environmenatal variable: 'ROBOT_TYPE' - has this been set?")
            self.shut_node_down()

        # start mavros interface thread
        self.mavros_interface = Mavros_interface(
                                                state_estimation_mode=self.state_estimation_mode,
                                                enforce_height_mode_flag=self.enforce_height_mode_flag,
                                                height_mode_req=self.height_mode_req
                                                )
        self.mavros_interface_thread = Thread(target=self.mavros_interface.run, args=())
        self.mavros_interface_thread.daemon = True
        self.mavros_interface_thread.start()
        rospy.loginfo('Pyx4_deprecated -> Mavros interface thread initialised')

        # initialise the watchdog and wait until
        wait_for_watchdog_rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.mavros_interface.wd_initialised:
            rospy.loginfo('[pyx4_v2]: Waiting for the pyx4 watchdog to inilialise')
            # todo - report what is preventing watchdog from completing
            wait_for_watchdog_rate.sleep()
        rospy.loginfo('Pyx4_deprecated watchdog initialised')

        # start commander thread
        # todo - add commander.mission_failed state - so that calling methods know when to give up
        self.commander = Commander(
                                    flight_instructions=flight_instructions,      # a list of all the flight instructions
                                    mavros_interface_node=self.mavros_interface,
                                    commander_parent_ref=self,
                                    start_authorised=start_authorised,
                                   )
        self.commander_thread = Thread(target=self.commander.run, args=())
        self.commander_thread.daemon = True
        if start_authorised:
            self.commander_thread.start()
            rospy.loginfo('commander initialised')

        # start setpoint publisher thread
        self.sp_pub_thread = Thread(target=setpoint_publisher, args=(self.mavros_interface, self.commander))
        self.sp_pub_thread.daemon = True
        if start_authorised:
            self.sp_pub_thread.start()
            rospy.loginfo('sp pub initialised')


    def publish_pyx4_state(self):

        self.pyx4_state_msg.header.stamp = rospy.Time.now()
        self.pyx4_state_msg_pub.publish(self.pyx4_state_msg)

    def do_delayed_start(self):
        """
        provides a mechanism for pyx4 to initialise (and start mavros) without starting the commander threads
        handy if other preconditions need to be waited for before starting the mission

        """
        self.commander_thread.start()
        rospy.loginfo('commander initialised')
        self.sp_pub_thread.start()
        rospy.loginfo('sp pub initialised')

    def run(self):
        """ Keep nodes alive until exit conditions present """

        rate = rospy.Rate(self._run_rate)
        while not rospy.is_shutdown() and self.commander._node_alive and self.mavros_interface._node_alive:
            try:
                rate.sleep()
            except:
                self.shut_node_down()

        # todo - return true if mission succesful and false if not - how to check this if the commander shuts down?


    def shut_node_down(self, shutdown_message='shutting down'):

        self.node_alive = False
        rospy.signal_shutdown(shutdown_message)
        sys.exit(1)


    def fail(self, e=None):
        rospy.logwarn(('Pyx4_deprecated v2 error is :', e))


if __name__ == '__main__':

    node_name = 'pyx4_node'

    rospy.init_node(node_name, anonymous=True, log_level=rospy.DEBUG)
    flight_instructions = {0: Arming_state(timeout=90.0,),  1: Take_off_state(), 2: Landing_state()}

    parser = argparse.ArgumentParser(description="This node is a ROS side mavros based state machine.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace",
                        default='mavros')  # JS - changed as mavros.DEFAULT_NAMESPACE doesn't seem to exist anymore - todo - is there a new enum for this?
    # parser.add_argument('-f', '--mission-file', help="which mission .csv file we will use to outline our mission",
    #                     default='basic_wpts')
    # parser.add_argument('-s', '--state-estimation-type', help="which sensing modality is used for state estimation",
    #                     default='GPS')
    parser.add_argument('--enforce_hgt_mode_flag', type=bool, default=False)
    parser.add_argument('--height_mode_req', type=int, default=0)
    parser.add_argument('--state_estimation', type=int, default=0)


    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])



    # flight_instructions = {0: Take_off_state()}

    # mission_file = os.path.join(MISSION_SPECS, 'big_square.csv')
    # flight_instructions = Wpts_from_csv(file_path=mission_file)


    pyx4 = Pyx4(flight_instructions=flight_instructions,
                enforce_height_mode_flag=args.enforce_hgt_mode_flag,
                height_mode_req=args.height_mode_req,
                )
    pyx4.run()