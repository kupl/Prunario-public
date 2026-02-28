from pathlib import Path
from typing import Union

from fuzz.commons.constants.simulation import *


class Config:

    VERSION_CARLA: str = '0.9.15'
    HOST_CARLA: str = HOST_CARLA
    PORT_CARLA: int = PORT_CARLA
    PORT_TM: int = PORT_TM
    NAME_CONTAINER_ADS: str = 'autoware_universe'
    NAME_CONTAINER_ADS_PLANNER: str = 'autoware_universe_planner_0'
    NAME_CONTAINER_CARLA: str = f'carla-{NAME_USER}-{CARLA_VERSION}'
    NAME_CONTAINER_BRIDGE: str = 'carla-apollo-15'

    PATH_SIMULATION: Path = PROJECT_ROOT / 'shared' / NAME_CONTAINER_ADS
    PATH_SIMULATION_LOG: Path = PATH_SIMULATION / 'logs'
    PATH_LOCK: Path = PATH_SIMULATION_LOG / 'lock'
    PATH_ID_FUZZER: Path = PATH_SIMULATION_LOG / 'other' # For other fuzzers
    PATH_DATA: Path = PROJECT_ROOT / 'data'
    PATH_CARLA_AUTOWARE: Path = PATH_SIMULATION / 'carla_autoware'
    PATH_APOLLO: Path = Path(__file__).resolve().parent / 'apollo'
    IP_APOLLO: str = 'localhost'
    PORT_APOLLO: int = 8899

    BP_EGO: str = 'vehicle.toyota.prius'

    TIMEOUT_STALLING: int = 20