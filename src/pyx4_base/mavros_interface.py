#!/usr/bin/env python3

import numpy as np
from threading import Thread, Lock
import rospy
import sys

from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, OpticalFlowRad, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, WaypointClear, WaypointPush, CommandTOL, CommandHome
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Range
from std_msgs.msg import Float64, Float32
from tf.transformations import euler_from_quaternion

from definitions_pyx4 import *

from definitions_pyx4 import MAV_VTOL_STATE, LANDED_STATE, MAV_STATE


class Mavros_interface(object):

    # todo - add mavros ns to topics
    def __init__(self,
                 ros_rate=10,   # slow as nothing happens in the main loop
                 state_estimation_mode=State_estimation_method.GPS,
                 enforce_height_mode_flag=False,
                 height_mode_req=0,
                 ):

        self.sem = state_estimation_mode
        self._node_alive = True
        self.ros_rate = ros_rate

        self.wd_initialised = False
        self.wd_fault_detected = False
        self.fault_this_loop = False

        # initialise data containers
        self.altitude = Altitude()
        self.altitude_bottom_clearance = Float32()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.optic_flow_raw = OpticalFlowRad()
        self.optic_flow_range = Range()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        # self.gt_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mocap_pose = PoseStamped()
        self.camera_pose = PoseStamped()
        self.camera_yaw = None
        self.local_x = Float64().data
        self.local_y = Float64().data
        self.local_z = Float64().data
        self.gt_x = Float64().data
        self.gt_y = Float64().data
        self.gt_z = Float64().data
        # todo - we can probably live with just XX_vel_bod data?:
        self.x_vel = Float64().data
        self.y_vel = Float64().data
        self.gt_x_vel = Float64().data
        self.gt_y_vel = Float64().data
        self.vel_ts = Float64().data
        self.xy_vel = Float64().data
        self.x_vel_bod = Float64().data
        self.y_vel_bod = Float64().data
        self.xy_vel_bod = Float64().data
        self.body_yaw_rate = Float64().data

        self.global_compass_hdg_deg = Float64().data

        self.enforce_height_mode_flag = enforce_height_mode_flag
        self.height_mode_req = height_mode_req

        # threading locks
        self.setpoint_lock = Lock()  # used for setting lock in our setpoint publisher so that commands aren't mixed

        # ROS services
        service_timeout = 10
        rospy.loginfo("Searching for mavros services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('/mavros/cmd/set_home')
            # rospy.wait_for_service('mavros/fcu_url', service_timeout)   # todo - check how this is used in px4
            self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
            self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
            self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
            self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            self.cmd_home_srv = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
            rospy.loginfo("Required ROS services are up")
        except rospy.ROSException:
            self.shut_node_down(extended_msg="failed to connect to Mavros services - was the mavros node started?")

        # make sure we have information about our connection with FCU
        rospy.loginfo("Get our fcu string")
        try:
            self.fcu_url = rospy.get_param('mavros/fcu_url')
        except Exception as e:
            print (e)
            self.shut_node_down(extended_msg="cant find fcu url")

        # ensure that our height mode is as we expect it to be (if required)
        rospy.loginfo('check height_mode {}'.format(self.enforce_height_mode_flag))
        if self.enforce_height_mode_flag:
            # todo - allow multiple attempts at this
                # res = self.mavros_interface.get_param_srv(param)
            param_read_attempts = 0
            try:

                while param_read_attempts < 5:
                    res = self.get_param_srv('EKF2_HGT_MODE')
                    if res.success:
                        self.height_mode = res.value.integer
                        if self.height_mode == self.height_mode_req:
                            rospy.loginfo('height mode {} as expected'.format(self.height_mode))
                            break
                        else:
                            raise Exception ("height mode is {} - (expected heightmode is {}) change parameter with QGround control and try again".format(self.height_mode, self.height_mode_req))
                            break
                    else:
                        rospy.logerr( "Couldn't read EKF2_HGT_MODE param on attempt {} - trying again".format(param_read_attempts))
                    param_read_attempts += 1
                    rospy.sleep(2)

            except Exception as e:
                rospy.logerr(
                    "Couldn't read EKF2_HGT_MODE - shutting down".format(param_read_attempts))
                self.shut_node_down(extended_msg= "height_mode error - traceback is {}".format(e))

        # todo: ensure that our state estimation parameters are as available (this requires the state estimation
        #  topic name so can't test this until we do something with mocap again) Actually, we can incorporate this into
        #  the watchdog
        # if state_estimation_mode == State_estimation_method.MOCAP:

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',ExtendedState,self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',NavSatFix, self.global_position_callback)
        self.optic_flow_raw_sub = rospy.Subscriber('mavros/px4flow/raw/optical_flow_raw',OpticalFlowRad, self.optic_flow_raw_callback)
        self.optic_flow_range_sub = rospy.Subscriber('mavros/px4flow/ground_distance',Range,self.optic_flow_range_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.mocap_pos_sub = rospy.Subscriber('mavros/vision_pose/pose', PoseStamped, self.mocap_pos_callback)
        # self.camera_pose_sub = rospy.Subscriber(self.camera_pose_topic_name, PoseStamped, self.cam_pose_cb)

        # todo - add check for this signal to watchdog - or remap /mavros/local_position/velocity -> /mavros/local_position/velocity_local
        self.velocity_local_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.vel_callback)
        self.velocity_body_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.vel_bod_callback)
        self.compass_sub = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.compass_hdg_callback)
        self.ground_truth_sub = rospy.Subscriber('/body_ground_truth', Odometry, self.gt_position_callback)

        ## Ros publishers
        self.local_pos_pub_raw = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        # ROS topics - this must come after our ROS subscribers
        topics_timeout = 30
        rospy.loginfo("waiting for ROS topics")
        try:
            # check that essential messages are being subscribed to regularly
            for _ in np.arange(2):
                rospy.wait_for_message('mavros/local_position/pose', PoseStamped, topics_timeout)
                rospy.wait_for_message('mavros/extended_state', ExtendedState, topics_timeout)
        except rospy.ROSException:
            self.shut_node_down(extended_msg="Required ros topics not published")

        rospy.loginfo("ROS topics are up")

        # create a watchdog thread that checks topics are being received at the expected rates
        self.watchdog_thread = Thread(target=self.watchdog, args=())
        self.watchdog_thread.daemon = True
        self.watchdog_thread.start()


    def run(self):

        rate = rospy.Rate(self.ros_rate)
        while not rospy.is_shutdown() and self._node_alive:
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logwarn(('Mavros interface error is :', e))


    def shut_node_down(self, extended_msg=''):
        self._node_alive = False
        rospy.logerr('mavros interface node is shutting down ' + extended_msg)
        sys.exit()

    ###########################################
    # Frequently used properties
    ###########################################
    @property
    def yaw_local(self):
        orientation_q = self.local_position.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw


    ###########################################
    # ROS callback functions
    ###########################################
    def altitude_callback(self, data):
        self.altitude = data
        self.altitude_bottom_clearance = data.bottom_clearance

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                MAV_VTOL_STATE(self.extended_state.vtol_state).name, MAV_VTOL_STATE(data.vtol_state).name))


        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                LANDED_STATE(self.extended_state.landed_state).name,  LANDED_STATE(data.landed_state).name))

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


    def gt_position_callback(self, odom):
        self.gt_x = odom.pose.pose.position.x
        self.gt_y = odom.pose.pose.position.y
        self.gt_z = odom.pose.pose.position.z

        self.gt_x_vel = odom.twist.twist.linear.x
        self.gt_y_vel = odom.twist.twist.linear.y


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
                MAV_STATE(self.state.system_status).name, MAV_STATE(data.system_status).name))
        self.state = data


    def mocap_pos_callback(self, data):
        self.mocap_pose = data


    def vel_callback(self, data):
        ## Coordinate frame for local pos (note this is relative to a fixed frame of reference and is not in the body
        # frame) to the take off point,  appears to be: X: forward, Y: Left, Z: up
        self.vel_ts = data.header.stamp.to_sec()
        self.x_vel = data.twist.linear.x
        self.y_vel = data.twist.linear.y
        self.xy_vel = np.linalg.norm((data.twist.linear.x, data.twist.linear.y))

    def vel_bod_callback(self, data):
        ## Coordinate frame for local pos (note this is relative to a fixed frame of reference and is not in the body
        # frame) to the take off point,  appears to be: X: forward, Y: Left, Z: up
        self.x_vel_bod = data.twist.linear.x
        self.y_vel_bod = data.twist.linear.y
        self.xy_vel_bod = np.linalg.norm((data.twist.linear.x, data.twist.linear.y))
        self.body_yaw_rate = data.twist.angular.z


    def compass_hdg_callback(self, data):
        self.global_compass_hdg_deg = data.data

    # def cam_pose_cb(self, this_pose):
    #     self.camera_pose = this_pose
    #     self.camera_yaw = self.pose2yaw(this_pose=self.camera_pose)


    def watchdog(self):
        """
        We ensure that data is A) present and B) once the watchdog is initialised, we ensure that data is coming in
        periodically.
        """
        # todo: consider monitoriing the following topics
        # gps_topics = ['global_pos', 'home_pos', 'mission_wp']
        # vision_topics = ['vision_pose']
        # main_topics = ['alt', 'ext_state', 'local_pos', 'state']

        #
        rate = rospy.Rate(0.5)
        time_last_msg = rospy.get_time()
        while not rospy.is_shutdown() and self._node_alive:

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

            ################################################### Check our state estimation status
            if self.sem is State_estimation_method.UNKNOWN:
                #
                pass

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


if __name__ == '__main__':

    rospy.init_node('mavros_interface_node', anonymous=True, log_level=rospy.DEBUG)

    # todo - add sem here if this will be used as a ros node
    mri = Mavros_interface()
    mri.run()