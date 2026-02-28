import json
from pathlib import Path
import time
from typing import Any, Dict, List


class State:

    def __init__(self, path_scenario: Path, _map: str) -> None:
        # Start time of the simulation
        self.time_start: float = time.time()
        # Path to save
        self.log_dir: Path = path_scenario
        # Current map
        self.map: str = _map
        # State
        self.state: List[Dict[str, Any]] = []
        # Bounding boxes
        self.bounding_boxes: Dict[str, Dict[str, Dict[str, float]]] = {}
        # Violation informations
        self.result: Dict[str, Any] = {
            'collision': False,
            'collision_with': None,
            'stalling': False,
            'speeding': False,
            'spawn_timeout': False,
            'start_timeout': False,
            'normal': False,
            'timeout': False
        }
        # End time of the simulation
        self.time_end: float = 0.0

    def __str__(self) -> str:
        return str(self.__dict__)

    def update(self, state_t: Dict[str, Any]) -> None:
        self.state.append(state_t)

    def any_violation(self) -> bool:
        return any(self.result.values())

    def dump(self) -> None:
        if not (self.log_dir / 'state.json').is_file():
            self.time_end = time.time()
            with (self.log_dir / 'state.json').open('w') as f:
                del self.log_dir
                json.dump(self.__dict__, f)
