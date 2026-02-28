from collections import defaultdict
from pathlib import Path
from typing import Dict

from fuzz.ads import ADS
from fuzz.behavior import *
from fuzz.carla import cc
from fuzz.commons.constants import *
from fuzz.commons.exceptions import DDSimulateFail, ExtractionException
from fuzz.commons.utils import get_logger
from fuzz.data import *
from fuzz.model import Model
from fuzz.simulation.planner_v import PlannerV
from fuzz.simulation.utils import distance_2d

import numpy as np
from tqdm import tqdm


def run_abs_real(scenario: Scenario,
                 models: Dict[str, Model],
                 ads: ADS,
                 start_delay: float,
                 path_run: Union[None, Path]=None) -> Dict[Union[int, str], Sequence]: #Sequence:
    info = dict()
    info['ego'] = dict()
    planner_v = PlannerV()
    # Get waypoints of the ego vehicle (DDEgo)
    try:
        points = [wp for wp in planner_v.plan_trajectory_ego(
            scenario.mission,
            models['ego'],
            traj_exp=scenario.traj_exp, # Provide ego vehicle's expected trajectory
            start_delay=start_delay,
            path_run=path_run
        )]
    except DDSimulateFail as e:
        get_logger().info("DdEgo failed.")
        raise DDSimulateFail
    yaws = [wp.transform.rotation.yaw % 360 for wp in points]
    info['ego']['type'] = ads.bp
    info['ego']['type_motion'] = 'ego'
    info['ego']['point'] = [p.transform for p in points]
    info['ego']['dyaw'] = np.append([0.0], np.diff(yaws))
    info['ego']['speed'] = [p.speed for p in points]
    # Get waypoints and steers of the NPCs
    try:
        _tmp = dict()
        for npc in scenario.npcs['vehicles'] + scenario.npcs['pedestrians']:
            try:
                points_v = [wp for wp in planner_v.plan_trajectory(
                    npc=npc,
                    model=models['npc'],
                    type_motion=npc['type_motion'],
                    # Only for the linear vehicle
                    speed_linear=npc['speed'] if 'speed' in npc else -1.0,
                    duration_max=len(points),    # Align static NPCs
                    path_run=path_run
                )]
            except ExtractionException as e:
                continue
            for p_v in points_v:
                if npc['id'] not in _tmp:
                    _tmp[npc['id']] = dict()
                    _tmp[npc['id']]['type'] = npc['bp']
                    _tmp[npc['id']]['type_motion'] = npc['type_motion']
                    _tmp[npc['id']]['point'] = []
                    _tmp[npc['id']]['speed'] = []
                _tmp[npc['id']]['point'].append(p_v.transform)
                _tmp[npc['id']]['speed'].append(p_v.speed)
        for _id, _info in _tmp.items():
            info[_id] = dict()
            points_npc = _info['point']
            yaws_npc = [wp.rotation.yaw % 360 for wp in points_npc]
            info[_id]['type'] = _info['type']
            info[_id]['type_motion'] = _info['type_motion']
            info[_id]['point'] = points_npc
            info[_id]['dyaw'] = np.append([0.0], np.diff(yaws_npc))
            info[_id]['speed'] = _info['speed']
            # Align the length of the waypoints
            len_npc = len(points_npc)
            if len(points) > len_npc:
                info[_id]['point'] += [info[_id]['point'][-1]] * (len(points) - len_npc)
                info[_id]['dyaw'] = np.append(info[_id]['dyaw'], [info[_id]['dyaw'][-1]] * (len(points) - len_npc))
                info[_id]['speed'] += [info[_id]['speed'][-1]] * (len(points) - len_npc)
            else:
                info[_id]['point'] = info[_id]['point'][:len(points)]
                info[_id]['dyaw'] = info[_id]['dyaw'][:len(points)]
                info[_id]['speed'] = info[_id]['speed'][:len(points)]
    except DDSimulateFail as e:
        get_logger().info("DdNPC failed.")
        raise DDSimulateFail
    bss = dict()
    min_idx_inter = len(points)
    for _id, _info in info.items():
        if _info['type_motion'] not in [DYNAMIC, 'ego'] or 'walker' in _info['type']:
            continue
        # Collect others' information
        if _id == 'ego':
            mission = scenario.mission
        else:
            _npc = [v for v in scenario.npcs['vehicles'] + scenario.npcs['pedestrians'] if v['id'] == _id].pop()
            mission = _npc
        info_others = {k: v for k, v in info.items() if k != _id}
        bs, idx_inter_first = get_driving_pattern_sequence(
            mission=mission,
            obj=_info, 
            obj_other=info_others, 
            violation='',   # Don't know violation in prediction
            is_abs=True
        )
        min_idx_inter = min(min_idx_inter, idx_inter_first)
        bss[_id] = bs
    # If any interaction occurs, trim the sequence
    if any([bs.any_interaction() for bs in bss.values()]):
        # Trim ego's sequence
        bs_final = Sequence()
        for _f in bss['ego']:
            bs_final.append(_f)
            if _f.start < min_idx_inter <= _f.end:
                _f.end = min_idx_inter
                break
        bss['ego'] = bs_final
    return bss
