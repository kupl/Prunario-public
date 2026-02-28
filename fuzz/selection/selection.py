from __future__ import annotations
from copy import deepcopy
import json
from pathlib import Path
from typing import Dict, List, Tuple, Union

from fuzz.commons.constants import *
from fuzz.data import Scenario


class SeedPool:

    def __init__(self,
                 path_log: Path) -> None:
                #  scenario_seed: Scenario,
                #  score_seed: float = 0.0) -> None:
        self.path_log: Path = path_log
        # self.elements: List[Tuple[Scenario, float]] = [
        #     (scenario_seed, score_seed)]
        self.elements: List[Tuple[Scenario, float]] = []
        self.seed_curr: str = ''
        self.score_curr: float = 0.0
        self.pool: Dict[str, float] = dict()
        self.__log: Dict[int, Dict[str, float]] = dict()
        self.num_pop: int = 0
        if (path_log / 'pool.json').is_file():
            with (path_log / 'pool.json').open('r') as f:
                res = json.load(f)
            self.seed_curr = res['seed_curr']
            self.score_curr = res['score_curr']
            self.__log = res['log']
            self.num_pop = res['num_pop']
            elements = []
            for path_scenario, score in res['pool'].items():
                scenario = Scenario(Path(path_scenario))
                elements.append((scenario, score))
            elements.sort(key=lambda x: x[1])
            self.elements = elements

    def is_empty(self) -> bool:
        return len(self.elements) == 0

    def push(self,
             scenario: Scenario,
             score: float) -> None:
        # if unconditional or is_interesting(score):
        self.elements.append((scenario, score))
        self.elements.sort(key=lambda x: x[1])

    def pop(self) -> Tuple[Scenario, float]:
        if not self.is_empty():
            # Log the current pool before pop
            self.__log[self.num_pop] = deepcopy(
                {str(scenario.path_scenario): score for scenario, score in self.elements})
            scenario, score = self.elements.pop()
            self.seed_curr = str(scenario.path_scenario)
            self.score_curr = score
            self.num_pop += 1
            return scenario, score
        else:
            raise IndexError("Seed pool is empty")

    def peek(self) -> Tuple[Scenario, Union[int, float]]:
        if not self.is_empty():
            scenario, score = self.elements[-1]
            return scenario, score
        else:
            raise IndexError("Seed pool is empty")

    def dump(self) -> None:
        res = dict()
        res['seed_curr'] = self.seed_curr
        res['num_pop'] = self.num_pop
        res['score_curr'] = self.score_curr
        res['pool'] = {str(scenario.path_scenario): score for scenario, score in self.elements}
        res['log'] = self.__log
        with (self.path_log / 'pool.json').open('w') as f:
            json.dump(res, f, indent=4)

    def clear(self) -> None:
        self.elements = []

    def __len__(self) -> int:
        return len(self.elements)

    def __str__(self) -> str:
        return '[' + ', '.join([f'({_scen.path_scenario.name}, {score:.3f})' for _scen, score in self.elements]) + ']'
