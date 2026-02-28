import os
from pathlib import Path
from typing import Optional


FPS: int = 20
# Project root path
PROJECT_ROOT: Path = Path(__file__).parent.parent.parent.parent.resolve()
# User name
NAME_USER: Optional[str] = os.getenv("USER")
# Display
DISPLAY: Optional[str] = os.getenv("DISPLAY")
# Carla version
CARLA_VERSION: str = '0.9.15'
# Carla host ip
HOST_CARLA: str = 'localhost'
# Carla port
PORT_CARLA: int = 2000
# Carla traffic manager port
PORT_TM: int = 8000
# Carla timeout
TIMEOUT_CARLA: int = 5

# Name of seed file
NAME_SEED: str = 'scenario.json'
# Name of state file
NAME_STATE: str = 'state.json'
# Name of front video
NAME_VIDEO_FRONT: str = 'front.mp4'
# Name of top video
NAME_VIDEO_TOP: str = 'top.mp4'
# Name of misbehavior file
NAME_FILE_VIOLATION: str = 'violation.txt'
# Name of bag file (For Autoware)
NAME_FILE_BAG: str = 'bag'
NAME_FILE_BAG_COMPRESSED: str = 'bag_compressed'
# Name of launch log file
NAME_FILE_LAUNCH_LOG: str = 'launch.log'
# Name of record file (For Apollo)
NAME_FILE_RECORD: str = '*.record.*.*'

# Simulation Results
SUCC: int = 0
NEXT_SEED: int = 1
TRY_AGAIN: int = 2
EXIT: int = 3


def set_port(port: int, id_gpu: int) -> None:
    global PORT_CARLA
    PORT_CARLA = port + id_gpu*3
