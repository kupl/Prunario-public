from __future__ import annotations
from collections import defaultdict
from copy import deepcopy
import json
import math
from typing import Any, Iterator, List, Tuple, Union

from fuzz.carla import cc
from fuzz.carla.utils import get_carla_location, get_carla_rotation, get_carla_vector3D, get_carla_transform, compute_ttc, RoadOption, transform_to_dict
from fuzz.commons.constants import *
from fuzz.data.record import Record, Scenario
from fuzz.simulation.utils import distance_2d

import numpy as np
import scipy


class Element:

    def redundant(self, other: Element) -> bool:
        if not isinstance(other, Element):
            raise TypeError(
                f"Cannot check subsumption relation of {self.__class__.__name__} and {other.__class__.__name__}.")

        if other.__class__.__name__ == self.__class__.__name__:
            return True
        elif 'top' in other.__class__.__name__.lower():
            return True
        else:
            return False

    def __str__(self) -> str:
        return self.__class__.__name__

    def __eq__(self, other: Element) -> bool:
        return str(self) == str(other)


### Horizontal Trajectory Shape ###
class ShapeH(Element):

    def __hash__(self) -> int:
        return hash(str(self))

    def __repr__(self) -> str:
        return self.__str__()


class HStraight(ShapeH):

    def __hash__(self) -> int:
        return hash(str(self))

    def __str__(self) -> str:
        return "↑"
        # return "→"


class HLeft(ShapeH):

    def __hash__(self) -> int:
        return hash(str(self))

    def __str__(self) -> str:
        return "↰"


class HRight(ShapeH):

    def __hash__(self) -> int:
        return hash(str(self))

    def __str__(self) -> str:
        return "↱"


### Vertical Trajectory Shape ###
class ShapeV(Element):

    def __repr__(self) -> str:
        return self.__str__()


class VUp(ShapeV):

    def __hash__(self) -> int:
        return hash(str(self))

    def __str__(self) -> str:
        return "↗"


class VDown(ShapeV):

    def __hash__(self) -> int:
        return hash(str(self))

    def __str__(self) -> str:
        return "↘"


class VFlat(ShapeV):

    def __hash__(self) -> int:
        return hash(str(self))

    def __str__(self) -> str:
        return "→"

### Interaction ###


class Interaction(Element):

    def __repr__(self) -> str:
        return self.__str__()


class INo(Interaction):

    def __str__(self) -> str:
        return "-"


class IYes(Interaction):

    def __str__(self) -> str:
        return "+"


class IYesVehicle(IYes):

    def __str__(self) -> str:
        return "VYes"


class IYesDynamicVehicle(IYesVehicle):

    def __str__(self) -> str:
        return "VD"


class IYesEgoVehicle(IYesVehicle):

    def __str__(self) -> str:
        return "VE"


class IYesLinearVehicle(IYesVehicle):

    def __str__(self) -> str:
        return "VL"


class IYesStaticVehicle(IYesVehicle):

    def __str__(self) -> str:
        return "VS"


class IYesPedestrian(IYes):

    def __str__(self) -> str:
        return "PYes"


class IYesDynamicPedestrian(IYesPedestrian):

    def __str__(self) -> str:
        return "ND"


class IYesLinearPedestrian(IYesPedestrian):

    def __str__(self) -> str:
        return "NL"


class IYesStaticPedestrian(IYesPedestrian):

    def __str__(self) -> str:
        return "NS"


class ITop(Interaction):

    def __str__(self) -> str:
        return "T"

### Interaction ###


### DrivingPattern ###
class DrivingPattern:

    def __init__(self) -> None:
        self.start: int = 0
        self.end: int = 0

    def __hash__(self) -> int:
        return hash(str(self))

    def redundant(self, _: DrivingPattern) -> bool:
        raise NotImplementedError

    def __str__(self) -> str:
        raise NotImplementedError

    def __eq__(self, _: DrivingPattern) -> bool:
        raise NotImplementedError


class Stop(DrivingPattern):

    def __init__(self) -> None:
        super().__init__()

    def __hash__(self) -> int:
        return hash(str(self))

    def redundant(self, other: DrivingPattern) -> bool:
        return isinstance(other, Stop)

    def __str__(self) -> str:
        return "STOP"

    def __eq__(self, other: DrivingPattern) -> bool:
        return isinstance(other, Stop)


class START(DrivingPattern):

    def __init__(self) -> None:
        super().__init__()

    def __hash__(self) -> int:
        return hash(str(self))

    def redundant(self, other: DrivingPattern) -> bool:
        return isinstance(other, START)

    def __str__(self) -> str:
        return "START"

    def __eq__(self, other: DrivingPattern) -> bool:
        return isinstance(other, START)


class DEST(DrivingPattern):

    def __init__(self) -> None:
        super().__init__()

    def __hash__(self) -> int:
        return hash(str(self))

    def redundant(self, other: DrivingPattern) -> bool:
        return isinstance(other, DEST)

    def __str__(self) -> str:
        return "DEST"

    def __eq__(self, other: DrivingPattern) -> bool:
        return isinstance(other, DEST)


class Move(DrivingPattern):

    def __init__(self,
                 shape_h: ShapeH,
                 shape_v: ShapeV,
                 interaction: Interaction) -> None:
        super().__init__()
        self.shape_h: ShapeH = shape_h
        self.shape_v: ShapeV = shape_v
        self.interaction: Interaction = interaction

    def redundant(self, other: DrivingPattern) -> bool:
        if isinstance(other, Move):
            return self.shape_h.redundant(other.shape_h) and \
                self.shape_v.redundant(other.shape_v) and \
                self.interaction.redundant(other.interaction)
        else:
            return False

    def __str__(self) -> str:
        return f"({str(self.shape_h)},{str(self.shape_v)},{str(self.interaction)})"

    def __eq__(self, other: DrivingPattern) -> bool:
        if isinstance(other, Move):
            return self.shape_h == other.shape_h and \
                self.shape_v == other.shape_v and \
                self.interaction == other.interaction
        else:
            return False

    def __hash__(self) -> int:
        return hash(str(self))

### DrivingPattern ###


### Behavior-feature Sequence ###
class Sequence:

    def __init__(self) -> None:
        self.features: List[DrivingPattern] = []
        self.violation: str = ''    # Initially bot
        self.details: list[list[int]] = []

    def __hash__(self) -> int:
        return hash(str(self))

    def __iter__(self) -> Iterator[DrivingPattern]:
        return iter(self.features)

    def append(self, feature: DrivingPattern) -> None:
        self.features.append(feature)

    def get_idx_first_interaction(self) -> int:
        for i, f in enumerate(self.features):
            # print(f)
            if isinstance(f, Move):
                if f.interaction != INo():
                    return i
            elif isinstance(f, Stop):
                return i
        return -1

    def slice_to_first_interaction(self) -> Sequence:
        _bs = deepcopy(self)
        idx = self.get_idx_first_interaction()
        if idx == -1:
            return _bs  # No interactions
        _bs.features = _bs.features[:idx+1]
        return _bs

    def any_interaction(self) -> bool:
        _inter = any([f.interaction != INo()
                     for f in self.features if isinstance(f, Move)])
        return _inter  # or _stop

    def any_stop(self) -> bool:
        _stop = any([f == Stop() for f in self.features])
        return _stop

    def redundant(self, other: Sequence) -> bool:
        if len(self) <= 0:
            return False
        if len(self) > len(other):
            return False
        # 1. Check violation
        if self.violation != other.violation:
            return False
        # 2. Check driving pattern sequence
        if self.features == other.features:
            return True

        return False

    def ddredundant(self, other: Sequence) -> bool:
        if len(self) <= 0:
            return False
        if len(self) > len(other):
            return False
        if self.features == other.features[:len(self)]:
            return True
        return False

    def ddredundant_traj_shape(self, other: Sequence) -> bool:
        if len(self) <= 0:
            return False

        _bs_other = deepcopy(other)
        _bs_other_traj_shape = Sequence()
        for _f in _bs_other:
            # Ignore Stop, while keeping START, DEST and Move
            if not isinstance(_f, Stop):
                # Consider trajectory shape only
                if isinstance(_f, Move):
                    _f.interaction = INo()
                # Deduplication
                if len(_bs_other_traj_shape) <= 0:
                    # First feature
                    _bs_other_traj_shape.append(_f)
                elif _bs_other_traj_shape[-1] != _f:
                    # Append only when feature changes
                    _bs_other_traj_shape[-1].end = _f.start
                    _bs_other_traj_shape.append(_f)
        _bs_other_traj_shape[-1].end = len(other)

        # Check if self trajectory shape is a prefix of other trajectory shape
        if self.features == _bs_other_traj_shape.features[:len(self)]:
            return True
        return False

    def __len__(self) -> int:
        return len(self.features)

    def __getitem__(self, idx: Union[int, slice]) -> DrivingPattern:
        return self.features[idx]

    def __str__(self) -> str:
        return '·'.join([str(f) for f in self.features])

    def __repr__(self) -> str:
        return self.__str__()

    def __eq__(self, other: Sequence) -> bool:
        if len(self) != len(other):
            return False
        for f1, f2 in zip(self.features, other.features):
            if f1 != f2:
                return False
        return True

    def insert(self, idx: int, feature: DrivingPattern) -> None:
        self.features.insert(idx, feature)

    def dump(self,
             path: Path,
             name: str = NAME_FILE_BEHAVIOR) -> None:
        with (path / name).open('w') as f:
            json.dump({
                'driving_pattern_sequence': str(self),
                'violation': self.violation,
                'debug': {f"[{f.start/FPS:.2f},{f.end/FPS:.2f}]": str(f) for f in self.features},
                'details': [(f.start, f.end) for f in self.features]
            }, f, indent=4, ensure_ascii=False)

    @classmethod
    def load(cls, path: Path, name: str = NAME_FILE_BEHAVIOR) -> Sequence:
        with (path / name).open('r') as f:
            data = json.load(f)
        bs = Sequence.from_str(data['driving_pattern_sequence'])
        for i, (start, end) in enumerate(data['details']):
            bs[i].start = start
            bs[i].end = end
        bs.violation = data['violation']
        bs.details = data['details']
        return bs

    @classmethod
    def from_str(cls, s: str) -> Sequence:
        bs = Sequence()
        for f in s.split('·'):
            if f == str(Stop()):
                bs.append(Stop())
            elif f == str(START()):
                bs.append(START())
            elif f == str(DEST()):
                bs.append(DEST())
            else:
                h, v, i = map(lambda _s: _s.strip(),
                              f.strip()[1:-1].split(','))
                if h == str(HStraight()):
                    h = HStraight()
                elif h == str(HLeft()):
                    h = HLeft()
                elif h == str(HRight()):
                    h = HRight()
                else:
                    raise ValueError(f"Unknown horizontal shape {h}")
                if v == str(VUp()):
                    v = VUp()
                elif v == str(VDown()):
                    v = VDown()
                elif v == str(VFlat()):
                    v = VFlat()
                else:
                    raise ValueError(f"Unknown vertical shape {v}")
                if i == str(IYes()):
                    i = IYes()
                elif i == str(IYesVehicle()):
                    i = IYesVehicle()
                elif i == str(IYesDynamicVehicle()):
                    i = IYesDynamicVehicle()
                elif i == str(IYesLinearVehicle()):
                    i = IYesLinearVehicle()
                elif i == str(IYesStaticVehicle()):
                    i = IYesStaticVehicle()
                elif i == str(IYesPedestrian()):
                    i = IYesPedestrian()
                elif i == str(IYesDynamicPedestrian()):
                    i = IYesDynamicPedestrian()
                elif i == str(IYesLinearPedestrian()):
                    i = IYesLinearPedestrian()
                elif i == str(IYesStaticPedestrian()):
                    i = IYesStaticPedestrian()
                elif i == str(IYesEgoVehicle()):
                    i = IYesEgoVehicle()
                elif i == str(INo()):
                    i = INo()
                elif i == str(ITop()):
                    i = ITop()
                else:
                    raise ValueError(f"Unknown interaction {i}")
                bs.append(
                    Move(
                        shape_h=h,
                        shape_v=v,
                        interaction=i
                    )
                )
        return bs
### Behavior-feature Sequence ###


def check_h(dyaw: float) -> ShapeH:
    if -MAX_YAW < dyaw < -MIN_YAW:
        h = HLeft()
    elif MIN_YAW < dyaw < MAX_YAW:
        h = HRight()
    else:
        h = HStraight()
    return h


def check_v(pitch_curr: float) -> ShapeV:
    pitch_curr = (pitch_curr - 180.0) % 360.0 - 180.0
    if pitch_curr > PITCH_THRESHOLD:
        v = VUp()
    elif pitch_curr < -PITCH_THRESHOLD:
        v = VDown()
    else:
        v = VFlat()
    return v


def get_driving_pattern_sequence(mission: Dict[str, Dict[str, float]],
                                 obj: Dict[str, Any],
                                 obj_other: Dict[str, Dict[str, Any]],
                                 violation: str,
                                 is_abs: bool = False) -> Tuple[Sequence, int]:
    bs = Sequence()
    dest = get_carla_location(mission['dest'])
    cnt_inter = 0
    f_prev = None
    if obj['type_motion'] == 'ego':
        bs.violation = violation
    # import time
    for i, (p, dyaw, speed) in enumerate(zip(obj['point'], obj['dyaw'], obj['speed'])):
        if distance_2d(p.location, obj['point'][0].location) < 0.01:  # 1.0:
            # If the object is near the starting point
            f = START()
        elif distance_2d(p.location, dest) < 1.0:
            # If the object is near the destination
            f = DEST()
        elif i < 30 or speed < THRESHOLD_STOP:
            f = Stop()
        else:
            # 1. Check Horizontal Trajectory Shape
            if speed < THRESHOLD_STOP:
                f = Stop()
            else:
                h_obj = check_h(dyaw)
                # 2. Check Vertical Trajectory Shape
                v_obj = check_v(p.rotation.pitch)
                # 3. Check Interaction
                inter = INo()
                inters_curr = []
                for _id in obj_other:
                    # For other techniques
                    if len(obj_other[_id]['point']) <= i:
                        continue
                    # cc.mark_point(obj_other[_id]['point'][i])
                    p_other = obj_other[_id]['point'][i]
                    speed_other = obj_other[_id]['speed'][i]
                    type_other = obj_other[_id]['type']
                    type_motion_other = obj_other[_id]['type_motion']
                    _dist = p.location.distance(p_other.location)
                    if _dist < 30.0 / 3.6 * THRESHOLD_INTER:
                        # If z axis is too far, ignore it
                        ttc = compute_ttc(
                            _p_i=p,
                            speed_i=speed,
                            bp_i=obj['type'],
                            _p_j=p_other,
                            speed_j=speed_other,
                            bp_j=type_other
                        )
                        # print(i, f, ttc)
                        if ttc < THRESHOLD_INTER:
                            # Check if the object is moving or not
                            if all([_spd < THRESHOLD_STOP for _spd in obj_other[_id]['speed'][i-K:i]]):
                                _mode = 'static'
                            else:
                                _mode = 'dynamic'
                            if 'vehicle' in type_other:
                                if _mode == 'dynamic':
                                    type_inter = IYesDynamicVehicle()
                                elif _mode == 'static':
                                    type_inter = IYesStaticVehicle()
                                else:
                                    type_inter = IYesVehicle()
                            elif 'walker' in type_other:
                                if _mode == 'dynamic':
                                    type_inter = IYesDynamicPedestrian()
                                elif _mode == 'static':
                                    type_inter = IYesStaticPedestrian()
                                else:
                                    type_inter = IYesPedestrian()
                            else:
                                type_inter = IYes()
                            inters_curr.append((_id, type_inter, _dist))
                if len(inters_curr) > 0:
                    inters_curr.sort(key=lambda x: x[2])
                    inter = inters_curr[0][1]
                f = Move(h_obj, v_obj, inter)
                if is_abs:
                    if f.interaction != INo() and f == f_prev:
                        cnt_inter += 1
                    else:
                        cnt_inter = 0
        f.start = i
        f.end = i + 1
        bs.append(f)
        f_prev = f
        if is_abs and cnt_inter >= (K - 1):  # At least 0.5 seconds
            break
    # Get the first index of interaction or stop
    idxs_inter_first = {
        'inter': len(bs),
        'stop': len(bs)
    }
    cnt = {
        'inter': 0,
        'stop': 0
    }
    inter_prev = INo()
    for i, f in enumerate(bs):
        if isinstance(f, START) or isinstance(f, DEST):
            continue
        elif isinstance(f, Stop):
            cnt['inter'] = 0
            cnt['stop'] += 1
            if cnt['stop'] >= K and idxs_inter_first['stop'] == len(bs) and i > 5 * FPS:
                idxs_inter_first['stop'] = i
        else:
            cnt['stop'] = 0
            if f.interaction != INo() and f.interaction == inter_prev:
                cnt['inter'] += 1
                if cnt['inter'] >= K and idxs_inter_first['inter'] == len(bs):
                    idxs_inter_first['inter'] = i
            else:
                cnt['inter'] = 0
            inter_prev = f.interaction
        if any([idxs_inter_first[k] != len(bs) for k in idxs_inter_first]):
            break
    idx_inter_first = min([v for v in idxs_inter_first.values()])
    return postprocess(bs), idx_inter_first  # bs


def get_driving_pattern_sequence_traj(mission: Dict[str, Dict[str, float]],
                                      obj: Dict[str, Any]) -> Sequence:
    bs = Sequence()
    dest = get_carla_location(mission['dest'])
    bs.violation = ''  # No violation for trajectory shape
    for i, (p, dyaw, speed) in enumerate(zip(obj['point'], obj['dyaw'], obj['speed'])):
        if distance_2d(p.location, obj['point'][0].location) < 0.01:  # 1.0:
            # If the object is near the starting point
            f = START()
        elif distance_2d(p.location, dest) < 1.0:
            # If the object is near the destination
            f = DEST()
        else:
            # 1. Check Horizontal Trajectory Shape
            h_obj = check_h(dyaw)
            # 2. Check Vertical Trajectory Shape
            v_obj = check_v(p.rotation.pitch)
            # 3. Check Interaction
            f = Move(h_obj, v_obj, INo())
        f.start = i
        f.end = i + 1
        bs.append(f)
    return postprocess(bs)


def get_driving_pattern_sequences(mission_ego: Dict[str, Dict[str, float]],
                                  record: Record,
                                  ego_only: bool = False) -> Tuple[Dict[Union[int, str], Sequence], Dict[Union[int, str], int]]:
    info = dict()
    # Preprocess record
    info['ego'] = dict()
    points = [get_carla_transform(s['point']) for s in record.state.state]
    yaws = [p.rotation.yaw % 360 for p in points]
    info['ego']['type'] = 'vehicle.toyota.prius'
    info['ego']['type_motion'] = 'ego'
    info['ego']['point'] = points
    info['ego']['dyaw'] = np.append([0.0], np.diff(yaws))
    info['ego']['speed'] = [s['speed'] for s in record.state.state]
    _tmp = dict()
    for i, s in enumerate(record.state.state):
        for npc in s['vehicles'] + s['pedestrians']:
            if npc['id'] not in _tmp:
                _tmp[npc['id']] = dict()
                _tmp[npc['id']]['type'] = npc['type']
                _tmp[npc['id']]['type_motion'] = npc['type_motion']
                _tmp[npc['id']]['point'] = []
                _tmp[npc['id']]['speed'] = []
            _tmp[npc['id']]['point'].append(get_carla_transform(npc['point']))
            _tmp[npc['id']]['speed'].append(npc['speed'])
    for _id, _info in _tmp.items():
        info[_id] = dict()
        points_npc = _info['point']
        yaws_npc = [p.rotation.yaw % 360 for p in points_npc]
        info[_id]['type'] = _info['type']
        info[_id]['type_motion'] = _info['type_motion']
        info[_id]['point'] = points_npc
        info[_id]['dyaw'] = np.append([0.0], np.diff(yaws_npc))
        info[_id]['speed'] = _info['speed']
    bss = dict()
    idxs_first_inter = dict()
    for _id, _info in info.items():
        # Compute only if the object is dynamic (+ego)
        if _info['type_motion'] not in [DYNAMIC, 'ego'] or 'walker' in _info['type']:
            continue
        # Collect others' information
        if _id == 'ego':
            mission = mission_ego
        else:   # NPC
            mission = {
                'start': transform_to_dict(_info['point'][0]),
                'dest': transform_to_dict(_info['point'][-1])
            }
        info_others = {k: v for k, v in info.items() if k != _id}
        bs, idx_inter_first = get_driving_pattern_sequence(
            mission=mission,
            obj=_info,
            obj_other=info_others,
            violation=record.violation
        )
        idxs_first_inter[_id] = idx_inter_first
        bss[_id] = bs  # postprocess(bs)
        if ego_only and _info['type_motion'] == 'ego':
            break
    return bss, idxs_first_inter


def postprocess(bs_raw: Sequence) -> Sequence:
    bs = Sequence()
    bs.violation = bs_raw.violation
    if len(bs_raw) < K:
        f = bs_raw[0]
        f.start = 0
        f.end = len(bs_raw)
        bs.append(f)
        return bs
    bs_tmp = Sequence()
    p_prev = None
    cnt = 1
    # Postprocess 1: Remove short actions
    for i, f in enumerate(bs_raw):
        if p_prev is None:
            p_prev = f
            bs_tmp.append(f)
            continue
        if p_prev == f:
            cnt += 1
        else:
            if cnt < K:  # 2 * FPS:
                bs_tmp.features = bs_tmp.features[:-cnt]
            cnt = 1
        bs_tmp.append(f)
        p_prev = f
    if cnt < K:
        if bs_tmp.features[-1] != DEST():
            bs_tmp.features = bs_tmp.features[:-cnt]
    # Postprocess 2: Merge consecutive patterns
    for f in bs_tmp:
        if len(bs) <= 0:
            # First feature
            bs.append(f)
        elif bs[-1] != f:
            # Append only when feature changes
            bs[-1].end = f.start
            bs.append(f)
    bs[-1].end = len(bs_raw)
    return bs
