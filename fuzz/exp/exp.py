import json
import math
from pathlib import Path
from typing import Set, Tuple

from fuzz.behavior import Sequence
from fuzz.commons.constants import *
from fuzz.data import Scenario
from fuzz.simulation.utils import distance_2d, get_carla_location


class Exp:

    def __init__(self, path_log: Path):
        self.path_log: Path = path_log
        self.__experienced: Set[Tuple[Path, Sequence, Sequence]] = set()
        # For RQ2
        self.__scenarios: Dict[Path, Scenario] = dict()
        if (self.path_log / 'exp.json').is_file():
            with (self.path_log / 'exp.json').open() as f:
                exp = json.load(f)
            # Load behavior-feature sequence from each path
            for path_mb in exp:
                path_mb = Path(path_mb)
                s = Scenario(path_mb)
                bs = Sequence.load(path_mb, NAME_FILE_BEHAVIOR)
                if (path_mb / NAME_FILE_BEHAVIOR_HAT).is_file():
                    # For pruning
                    bs_hat = Sequence.load(path_mb, NAME_FILE_BEHAVIOR_HAT)
                else:
                    # For baseline
                    bs_hat = bs
                self.__experienced.add((path_mb, bs, bs_hat))
                self.__scenarios[path_mb] = s

    def redundant(self, 
                  bs: Sequence) -> bool:
        if len(self.__experienced) <= 0:
            return False
        else:
            cond1 = any(bs.redundant(bs_exp) for _, bs_exp, _ in self.__experienced)
            return cond1

    def ddredundant(self,
                    bs_hat: Sequence) -> bool:
        if len(self.__experienced) <= 0:
            return False
        else:
            cond1 = any(bs_hat.ddredundant(bs_exp) for _, bs_exp, _ in self.__experienced)
            return cond1


    def append(self,
               path_exp: Path,
               s: Scenario,
               bs: Sequence,
               bs_hat: Sequence) -> None:
        self.__experienced.add((path_exp, bs, bs_hat))
        self.__scenarios[path_exp] = s

    def dump(self) -> None:
        with (self.path_log / 'exp.json').open('w') as f:
            json.dump([str(path_exp)
                      for path_exp, _, _ in self.__experienced], f, indent=4)

    def get_experienced(self) -> Set[Tuple[Path, Sequence, Sequence]]:
        return self.__experienced

    def __len__(self) -> int:
        return len(self.__experienced)

    def ddredundant_autofuzz(self,
                             s: Scenario) -> bool:
        if len(self.__experienced) <= 0:
            return False
        else:
            for _s in self.__scenarios.values():
                if s.ddredundant_autofuzz(_s):
                    return True
            return False


from itertools import zip_longest

def vectorize_with_padding(s1: Scenario, s2: Scenario) -> Tuple[List[float], List[float]]:
    res_s1 = []
    res_s2 = []

    # Mission (No padding)
    res_s1.extend(s1.mission['start'].values())
    res_s1.extend(s1.mission['dest'].values())
    res_s2.extend(s2.mission['start'].values())
    res_s2.extend(s2.mission['dest'].values())

    # NPCs
    for target in ['vehicles', 'pedestrians']:
        for _n1, _n2 in zip_longest(s1.npcs[target], s2.npcs[target]):
            # s1
            # res_s1.append(i)
            if _n1 is None:
                res_s1.extend([0.0] * 15)
            else:
                if _n1['type_motion'] == STATIC:
                    res_s1.append(0.0)
                    res_s1.append(0.0) # speed placeholder
                elif _n1['type_motion'] == LINEAR:
                    res_s1.append(1.0)
                    res_s1.append(_n1['speed']) # speed
                else:   # Dynamic
                    res_s1.append(2.0)
                    if target == 'vehicles':
                        res_s1.append(0.0) # speed placeholder
                    else:
                        res_s1.append(_n1['speed'])
                if target == 'vehicles':
                    res_s1.append(TYPES_VEHICLE.index(_n1['bp']))
                else:
                    res_s1.append(TYPES_PEDESTRIAN.index(_n1['bp']))
                res_s1.extend(_n1['start'].values())
                res_s1.extend(_n1['dest'].values())
            # s2
            # res_s2.append(i)
            if _n2 is None:
                res_s2.extend([0.0] * 15)
            else:
                if _n2['type_motion'] == STATIC:
                    res_s2.append(0.0)
                    res_s2.append(0.0) # speed placeholder
                elif _n2['type_motion'] == LINEAR:
                    res_s2.append(1.0)
                    res_s2.append(_n2['speed']) # speed
                else:   # Dynamic
                    res_s2.append(2.0)
                    if target == 'vehicles':
                        res_s2.append(0.0) # speed placeholder
                    else:
                        res_s2.append(_n2['speed'])
                if target == 'vehicles':
                    res_s2.append(TYPES_VEHICLE.index(_n2['bp']))
                else:
                    res_s2.append(TYPES_PEDESTRIAN.index(_n2['bp']))
                res_s2.extend(_n2['start'].values())
                res_s2.extend(_n2['dest'].values())
    assert len(res_s1) == len(res_s2), print(s1.npcs, s2.npcs)
    # Weather
    res_s1.extend(s1.weather.values())
    res_s2.extend(s2.weather.values())
    # Time
    res_s1.extend(s1.time.values())
    res_s2.extend(s2.time.values())
    # Puddles
    for _p1, _p2 in zip_longest(s1.puddles, s2.puddles):
        if _p1 is None:
            res_s1.extend([0.0] * 7)
        else:
            res_s1.extend(_p1.values())
        if _p2 is None:
            res_s2.extend([0.0] * 7)
        else:
            res_s2.extend(_p2.values())
    assert len(res_s1) == len(res_s2)#, print(s1, s2)
    return res_s1, res_s2
