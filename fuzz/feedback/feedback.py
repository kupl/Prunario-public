import json
import math
from typing import Any, Dict, Tuple, Union

from fuzz.carla import cc
from fuzz.carla.utils import *
from fuzz.commons.constants import *
from fuzz.data.record import Record, State

import numpy as np


class Feedback:

    def __init__(self,
                 path_mb: Path,
                 state: State) -> None:
        d_ttc_min = min_TTC(state)
        d_max_lane = max_distance_from_the_lane_center(state)
        accel = fast_accel(state)
        self.feedbacks: Dict[str, Any] = {
            'min_ttc': d_ttc_min,
            'dist_max_from_lane_center': d_max_lane,
            'fast_accel': accel,
        }
        self.score: float = sum(self.feedbacks.values())
        
    def dump(self, path: Union[str, Path]) -> None:
        if isinstance(path, str):
            path = Path(path)
        with (path / 'feedback.json').open('w') as f:
            json.dump(self.feedbacks, f, indent=4)
        with (path / 'score.txt').open('w') as f:
            f.write(str(self.score))


def min_TTC(state: State) -> float:
    """
    Calculate the minimum Time-to-Collision met during the driving.
    Range: (0, 100]

    :returns: The minimum Time-to-Collision met during the driving.
    :rtype: float
    """
    ttc_min = 100
    for s in state.state:
        p_ego = get_carla_transform(s['point'])
        speed_ego = s['speed']
        for npc in s['vehicles'] + s['pedestrians']:
            p_npc = get_carla_transform(npc['point'])
            # Check distance between two:
            if p_ego.location.distance(p_npc.location) > 30.0:
                continue
            speed_npc = npc['speed']
            try:
                ttc = compute_ttc(
                    _p_i=p_ego,
                    speed_i=speed_ego,
                    bp_i='vehicle.toyota.prius',
                    _p_j=p_npc,
                    speed_j=speed_npc,
                    bp_j=npc['type']
                )
            except RuntimeError:
                ttc = 100
            if ttc < ttc_min:
                ttc_min = ttc
    if ttc_min <= 0:    # For zero division
        ttc_min = 0.00000001
    return 1 / ttc_min


def max_distance_from_the_lane_center(state: State) -> float: #Tuple[float, float]:
    """
    Calculate the maximum distance from the lane center of the ego vehicle during the driving.
    Range: [0, 1]

    :returns: The maximum distance from the lane center of the ego vehicle during the driving.
    :rtype: float
    """
    d_max = 0.0
    for s in state.state:
        p_ego = s['point']
        p_nearest = cc.get_the_nearest_wp(p_ego)
        width = p_nearest.lane_width / 2
        loc = p_nearest.transform.location
        d = get_carla_location(p_ego).distance(loc)
        _d = d / width
        if _d > d_max:
            d_max = _d
    return d_max #, d_max


def fast_accel(state: State) -> float: #Tuple[float, float]:
    """
    Calculate the feedback score of the fast acceleration.
    Range: [0, 1]

    :returns: The feedback score of the fast acceleration.
    :rtype: float
    """
    # Filter first 5 seconds, which makes false positive
    speeds = [s['speed'] for s in state.state][5*FPS:]
    if len(speeds) > 2:
        diffs_speed = np.diff(speeds)
        v = abs(max(diffs_speed))
    else:
        v = 0.0
    return v / 5 #, v


def get_feedback(path_mb: Path,
                 state: State) -> Feedback:
    """
    Calculate the feedback from the record.

    :param record: The record to calculate the feedback score.

    :returns: Feedback scores.
    :rtype: Dict[str, float]
    """
    feedback = Feedback(path_mb, state)
    return feedback
