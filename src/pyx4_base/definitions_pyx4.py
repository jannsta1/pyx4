import os
from enum import Enum

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(ROOT_DIR, 'data')
MISSION_SPECS = os.path.join(DATA_DIR, 'mission_specs')

EXAMPLE_MISSION = os.path.join(MISSION_SPECS, 'basic_wpts.csv')



VALID_WAYPOINT_TYPES = ['hold', 'pos', 'pos_with_vel', 'vel_xy', 'vel']

class Hardware(Enum):
    UNKNOWN = 0
    SITL = 1
    HITL = 2  # not currently used
    PIXHAWK = 3


class State_estimation_method(Enum):
    UNKNOWN = 0  # sometimes used to bypass state estimation checks - e.g. if we want to test something without the need to fly
    MOCAP = 1
    OPTIC_FLOW = 2
    GPS = 3

    # methods for argparse compatibility
    def __str__(self):
        return self.name.upper()

    def __repr__(self):
        return str(self)


class TAKE_OFF_PHASE(Enum):
    CLEAR_THE_GROUND = 0
    GO_TO_WPT = 1


class MAV_VTOL_STATE(Enum):
    MAV_VTOL_STATE_UNDEFINED = 0
    MAV_VTOL_STATE_TRANSITION_TO_FW = 1
    MAV_VTOL_STATE_TRANSITION_TO_MC = 2
    MAV_VTOL_STATE_MC = 3
    MAV_VTOL_STATE_FW = 4


class LANDED_STATE(Enum):
    LANDED_STATE_UNDEFINED = 0
    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2
    LANDED_STATE_TAKEOFF = 3
    LANDED_STATE_LANDING = 4


class MAV_STATE(Enum):
        MANUAL = 0
        ALTCTL = 1
        POSCTL = 2
        AUTO_MISSION = 3
        AUTO_LOITER = 4
        AUTO_RTL = 5
        ACRO = 6
        OFFBOARD = 7
        STAB = 8
        RATTITUDE = 9
        AUTO_TAKEOFF = 10
        AUTO_LAND = 11
        AUTO_FOLLOW_TARGET = 12
        MAX = 13