import os
from enum import Enum

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(ROOT_DIR, 'data')
MISSION_SPECS = os.path.join(DATA_DIR, 'mission_specs')

EXAMPLE_MISSION = os.path.join(MISSION_SPECS, 'basic_wpts.csv')

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


VALID_WAYPOINT_TYPES = ['hold', 'pos', 'pos_with_vel', 'vel_xy', 'vel']


class TAKE_OFF_PHASE(Enum):
    CLEAR_THE_GROUND = 0
    GO_TO_WPT = 1