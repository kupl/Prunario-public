from typing import Dict, List, Set, Tuple, Union

from fuzz.behavior import *
from fuzz.commons.exceptions import ExtractionException
from fuzz.data import Record, Scenario
from fuzz.simulation.utils import distance_2d

import carla
import numpy as np


class Extractor:

    def __init__(self,
                 mission: Dict[str, Dict[str, float]],
                 traj_exp: List[carla.Transform]) -> None:
        self.mission: Dict[str, Dict[str, float]] = mission
        self.traj_exp: List[carla.Transform] = traj_exp
        self.dist_driven_all: float = 0.0
        self.dist_all: float = sum([self.traj_exp[i].location.distance(
            self.traj_exp[i+1].location) for i in range(len(self.traj_exp)-1)])
        self.wp_prev: carla.Transform = get_carla_transform(mission['start'])

        self.idx_lane_curr_h: int = 0
        self.shapes_h: List[Element] = []
        self.dists_lane_h: List[float] = []
        self.dist_driven_h: float = 0.0
        self.dist_driven_lane_curr_h: float = 0.0
        self.wps_changes_h: List[Tuple[carla.Transform, carla.Transform]] = []
        
        self.idx_lane_curr_v: int = 0
        self.shapes_v: List[Element] = []
        self.dists_lane_v: List[float] = []
        self.dist_driven_v: float = 0.0
        self.dist_driven_lane_curr_v: float = 0.0
        self.wps_changes_v: List[Tuple[carla.Transform, carla.Transform]] = []
        
        self.ctx_speed: List[float] = [0.0] * 30

        self.__setup()

    def __cons(self, shapes: List[Element]) -> List[Element]:
        shapes_tmp = []
        p_prev = None
        cnt = 1
        for i, f in enumerate(shapes):
            if p_prev is None:
                p_prev = f
                shapes_tmp.append(f)
                continue
            if p_prev == f:
                cnt += 1
            else:
                if cnt < K:
                    shapes_tmp = shapes_tmp[:-cnt]
                cnt = 1
            shapes_tmp.append(f)
            p_prev = f
        if cnt < K:
            shapes_tmp = shapes_tmp[:-cnt]
        return shapes_tmp

    def __dedup(self, shapes_tmp: List[Element]) -> Tuple[List[Element], List[int]]:
        shapes = []
        cuts = []
        for i, f in enumerate(shapes_tmp):
            if len(shapes) <= 0 or f != shapes[-1]:
                if i > 0:
                    cuts.append(i)
                shapes.append(f)
        return shapes, cuts

    def __segmentation(self,
                       cuts: List[int],
                       target: str) -> None:
        if target == 'horizontal':
            suffix = '_h'
        else:   # 'vertical'
            suffix = '_v'
        wp_prev = self.traj_exp[0]
        _dist = 0.0
        for i in range(len(self.traj_exp)):
            if i < len(self.traj_exp)-1 and self.traj_exp[i] == self.traj_exp[i+1]:
                continue
            if i in cuts:
                getattr(self, f'wps_changes{suffix}').append((wp_prev, self.traj_exp[i]))
                wp_prev = self.traj_exp[i]
                getattr(self, f'dists_lane{suffix}').append(_dist)
                _dist = 0.0
            _dist += self.traj_exp[i].location.distance(
                self.traj_exp[i+1].location) if i < len(self.traj_exp)-1 else 0.5
        getattr(self, f'dists_lane{suffix}').append(_dist)
        getattr(self, f'wps_changes{suffix}').append((wp_prev, self.traj_exp[-1]))


    # Setup road information for feature extraction
    def __setup(self) -> None:
        if len(self.traj_exp) <= 1:
            raise ExtractionException
        __yaws = [wp.rotation.yaw % 360 for wp in self.traj_exp]
        __dyaws = np.append([0.0], np.diff(__yaws))
        _hors: List[Element] = [check_h(__dyaws[i]) for i in range(len(self.traj_exp))]
        _hors_tmp = self.__cons(_hors)  # []
        _verts: List[Element] = [check_v(self.traj_exp[i].rotation.pitch) 
                                 for i in range(len(self.traj_exp))]
        _verts_tmp = self.__cons(_verts)  # []
        self.shapes_h, cuts_h = self.__dedup(_hors_tmp)
        self.shapes_v, cuts_v = self.__dedup(_verts_tmp)
        self.__segmentation(cuts_h, 'horizontal')
        self.__segmentation(cuts_v, 'vertical')


    def get_speed_feature(self,
                          wp: Union[Dict[str, float], carla.Transform],
                          speed: float) -> Dict[str, Any]:
        if isinstance(wp, dict):
            wp = get_carla_transform(wp)
        if self.wp_prev is None:
            # For the first frame
            self.wp_prev = wp
        dist_delta = wp.location.distance(self.wp_prev.location)
        self.dist_driven_all += dist_delta
        for suffix in ['_h', '_v']:
            idx_lane_curr = getattr(self, f'idx_lane_curr{suffix}')
            dist_lane_curr = getattr(self, f'dists_lane{suffix}')[idx_lane_curr]
            setattr(self, f'dist_driven{suffix}', getattr(self, f'dist_driven{suffix}') + dist_delta)
            setattr(self, f'dist_driven_lane_curr{suffix}', getattr(self, f'dist_driven_lane_curr{suffix}') + dist_delta)
            if getattr(self, f'dist_driven_lane_curr{suffix}') > dist_lane_curr:
                setattr(self, f'dist_driven_lane_curr{suffix}', dist_lane_curr)
            dist_remain_lane_curr = dist_lane_curr - getattr(self, f'dist_driven_lane_curr{suffix}')
            if dist_remain_lane_curr < 0.0:
                dist_remain_lane_curr = 0.0
            if wp.location.distance(getattr(self, f'wps_changes{suffix}')[idx_lane_curr][1].location) < 1:
                if idx_lane_curr + 1 < len(getattr(self, f'wps_changes{suffix}')):
                    setattr(self, f'idx_lane_curr{suffix}', idx_lane_curr + 1)
                    setattr(self, f'dist_driven_lane_curr{suffix}', 0.0)
                    dist_lane_curr = getattr(self, f'dists_lane{suffix}')[idx_lane_curr + 1]
                    dist_remain_lane_curr = dist_lane_curr
                else:
                    raise ExtractionException

        # Feature 1: The average speed (in km/h) over at most the past 30 frames
        _f1 = sum(self.ctx_speed) / len(self.ctx_speed)
        # Feature 2: The travel distance between start and cur
        _f2 = self.dist_driven_all
        # Feature 3: The remaining distance between cur and end
        _f3 = max(0.0, self.dist_all - self.dist_driven_all)
        # Feature 4: The travel distance between h_start and cur
        _f4 = self.dist_driven_lane_curr_h
        # Feature 5: The remaining distance between cur and h_end
        _f5 = max(0.0, self.dists_lane_h[self.idx_lane_curr_h] - self.dist_driven_lane_curr_h)
        # Feature 6: The travel distance between v_start and cur
        _f6 = self.dist_driven_lane_curr_v
        # Feature 7: The remaining distance between cur and v_end
        _f7 = max(0.0, self.dists_lane_v[self.idx_lane_curr_v] - self.dist_driven_lane_curr_v)
        self.wp_prev = wp
        return {
            'speed_avg': _f1,
            'driven_all': _f2,
            'remain_all': _f3,
            'driven_h': _f4,
            'remain_h': _f5,
            'driven_v': _f6,
            'remain_v': _f7,
        }


def extract_speed_features(scenario: Scenario,
                           record: Record,
                           until: Dict[Union[int, str], int]) -> Dict[str, Dict[str, Set[Tuple[Tuple[str, float], float]]]]:
    dataset = dict()
    for target in ['ego'] + ['npc']:
        dataset[target] = set()
    traj_exp = scenario.traj_exp
    assert len(traj_exp) > 0
    # Ego Vehicle
    _points = [s['point'] for s in record.state.state][:until['ego']]
    if len(_points) < 2:
        return dataset
    _speeds = [s['speed'] for s in record.state.state][:until['ego']]

    features = []
    speeds = []
    _points = [get_carla_transform(p) for p in _points]
    ext = Extractor(mission=scenario.mission, traj_exp=traj_exp)
    for i, (p, s) in enumerate(zip(_points[:-1], _speeds[:-1])):
        try:
            f = ext.get_speed_feature(p, s)
        except ExtractionException as _:
            # Extraction ended.
            break
        if f not in features:
            features.append(f)
            speeds.append(_speeds[i+1])
            # speeds.append(speeds_diff[i])
    for _f, _s in zip(features, speeds):
        dataset['ego'].add((tuple(_f.values()), _s))
    # NPCs
    for npc in scenario.npcs['vehicles']:
        # Dynamic NPCs only
        if npc['type_motion'] == DYNAMIC and npc['id'] in until:
            _points = [_v['point']
                       for s in record.state.state for _v in s['vehicles'] if _v['id'] == npc['id']][:until[npc['id']]]
            _speeds = [_v['speed']
                       for s in record.state.state for _v in s['vehicles'] if _v['id'] == npc['id']][:until[npc['id']]]
            try:
                traj_exp = cc.plan_trajectory(npc)
                ext = Extractor(mission=npc, traj_exp=traj_exp)
            except ExtractionException as _:
                break
            features_v = []
            speeds_v = []
            for i, (p, s) in enumerate(zip(_points[:-1], _speeds[:-1])):
                try:
                    f = ext.get_speed_feature(p, s)
                except ExtractionException as e:
                    break
                if f not in features_v and s > 1.0:
                    features_v.append(f)
                    speeds_v.append(_speeds[i+1])
            for _f, _s in zip(features_v, speeds_v):
                dataset['npc'].add((tuple(_f.values()), _s))
    return dataset
