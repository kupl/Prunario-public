import os
from typing import List

import carla


PATH_PROJECT_ROOT: str = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..'))
FORMAT_LOGGING_DIR: str = "%Y-%m-%d-%H:%M:%S"

NAME_FILE_SCENARIO: str = 'scenario.json'
NAME_FILE_STATE: str = 'state.json'
NAME_FILE_GROUND_TRUTH: str = 'manual_truth.txt'
NAME_FIGURE_DIR: str = 'figures'

TIME_THRESHOLD_STALLING: int = 60
MISSION_THRESHOLD: float = 0.95
DISTANCE_THRESHOLD_DEST: float = 10.0
DISTANCE_THRESHOLD_START: float = 2.0
DISTANCE_THRESHOLD_NEAR_START: float = 2.0
DISTANCE_THRESHOLD_AT: float = 2.0
PITCH_THRESHOLD: float = 5.0 #4.0 #3.0
STEERING_THRESHOLD: float = 5 / 90.0   # 1 degrees
SURROUNDINGS_DIST_THRESHOLD: float = 20

TAGS_STATIC_OBJECTS = [
    'Fences',
    'Poles',
    'TrafficSigns',
    'Vegetation',
    'GuardRail',
    'TrafficLight',
    'Static',
    'Dynamic',
    'Buildings',
    'Walls',
    'Vehicles',
    'Pedestrians'
]

TYPE_NPC_CAR: List[str] = [
    "vehicle.audi.a2",
    "vehicle.ford.mustang",
    "vehicle.mercedes.coupe",
    "vehicle.micro.microlino"
]
TYPE_NPC_TRUCK: List[str] = [
    "vehicle.carlamotors.carlacola",
]
TYPE_NPC_VAN: List[str] = [
    "vehicle.volkswagen.t2",
]
TYPE_NPC_BICYCLE: List[str] = [
    "vehicle.gazelle.omafiets",
]
TYPE_NPC_MOTORCYCLE: List[str] = [
    "vehicle.yamaha.yzf"
]


CORNERING_INTERVAL: int = 5
CORNERING_THERESHOLD: float = 0.999999

CORNER_THRESHOLD = STEERING_THRESHOLD

THRESHOLD_DISTANCE_NEAR: float = 20.0  # 13.0
THRESHOLD_ANGLE_AHEAD: float = 30.0
THRESHOLD_ANGLE_BEHIND: float = (180 - THRESHOLD_ANGLE_AHEAD)
THRESHOLD_ANGLE_REVERSE: float = 90.0
THRESHOLD_ANGLE_VISIBLE: float = 30.0
THRESHOLD_ANGLE_NEAR: float = 59.0
THRESHOLD_ANGLE_CURRENT_LANE: float = 100.0

# NUM_MINIMUM_CLUSTERS: int = 15
NUM_MAXIMUM_RULES: int = 5

DESC_STATE_LOADING: str = 'Loading states...'
DESC_FEATURE_EXTRACTION: str = 'Extracting features from states...'
DESC_RULE_SEARCH: str = 'Searching for the best set of rules for clustering...'

NUM_PROCESS: int = 20

INTERVAL: int = 2

LABEL_POSITIVE_EXAMPLES: str = 'POS'

INITIAL_VAILD_LANE_ID: int = -1

FILE_NAME_LOGGING_FEATURE: str = 'feature.txt'
FILE_NAME_LOGGING_FEATURE_RAW: str = 'feature_raw.txt'
FILE_NAME_LOGGING_REASON: str = 'reason.txt'
THRESHOLD_DISTANCE_RELATED: float = 50.0

TIME_REINITIALIZE: int = 10

EXACT_MATCH: str = 'exact_match'
RATIO_MATCH: str = 'ratio_match'
DBSCAN_MATCH: str = 'dbscan'

THRESHOLD_RATIO_MATCH: float = 0.5
