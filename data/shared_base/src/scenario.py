from copy import deepcopy
from pathlib import Path
import json
from typing import Any, Dict, List


class Scenario:

    def __init__(self, path_scenario: Path) -> None:
        self.path_scenario: Path = path_scenario
        js = json.load((self.path_scenario / 'scenario.json').open())
        # # of mutation
        self.num_mutation: int = js['num_mutation'] if 'num_mutation' in js else 0
        # Map
        self.map: str = js['map']
        # Mission
        self.mission: Dict[str, Dict[str, float]] = js['mission']
        # NPCs
        self.npcs: Dict[str, List[Dict[str, Any]]] = js['npcs']
        # Weather
        self.weather: Dict[str, float] = js['weather']
        # Time
        self.time: Dict[str, float] = js['time']
        # Puddles
        self.puddles: List[Dict[str, float]] = js['puddles']
        # Applied mutation operators
        self.hist_mops: Dict[str, List[str]] = js['hist_mops']

    def __str__(self) -> str:
        return json.dumps({
            'map': self.map,
            'mission': self.mission,
            'npcs': self.npcs,
            'weather': self.weather,
            'time': self.time,
            'puddles': self.puddles,
            'hist_mops': self.hist_mops
        })

    def dump(self,
             p: Path,
             name: str = 'scenario.json') -> None:
        _p = deepcopy(self.path_scenario)
        del self.path_scenario
        with (p / name).open('w') as f:
            json.dump(self.__dict__, f, indent=4)
        self.path_scenario = _p
