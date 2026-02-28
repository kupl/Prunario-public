from collections import defaultdict
from copy import deepcopy
import math
import random
from typing import Any, Callable, Dict, Generator, List, Tuple

from fuzz.ads import ADS
from fuzz.carla import cc
from fuzz.carla.utils import get_carla_transform, get_carla_location
from fuzz.commons.exceptions import PlanningAgain, TryMutationAgain
from fuzz.commons.constants import *
from fuzz.commons.utils import get_logger
from fuzz.data import Scenario
from fuzz.simulation.utils import distance_2d


class Mutator:

    @classmethod
    def get_operators(cls) -> List[Callable[[Scenario], Scenario]]:
        return [getattr(cls, method) for method in dir(cls) if method.startswith('mops_')]


class PointMutator:

    @classmethod
    def to_forward(cls,
                   point: Dict[str, float],
                   dist: float = DISTANCE_FORWARD,
                   target: str = 'vehicles') -> Dict[str, float]:
        _dist_random = random.uniform(2, dist)
        wp_forward = cc.get_next_wp(point, _dist_random, target)
        if wp_forward is None:
            raise TryMutationAgain
        _point = deepcopy(point)
        _point['x'] = wp_forward.transform.location.x
        _point['y'] = wp_forward.transform.location.y
        _point['z'] = wp_forward.transform.location.z + 2.0
        if target == 'vehicles':
            _point['roll'] = wp_forward.transform.rotation.roll
            _point['pitch'] = wp_forward.transform.rotation.pitch
            _point['yaw'] = wp_forward.transform.rotation.yaw
        return _point

    @classmethod
    def to_backward(cls,
                    point: Dict[str, float],
                    dist: float = DISTANCE_BACKWARD,
                    target: str = 'vehicles') -> Dict[str, float]:
        _dist_random = random.uniform(2, dist)
        wp_backward = cc.get_previous_wp(point, _dist_random, target)
        if wp_backward is None:
            raise TryMutationAgain
        _point = deepcopy(point)
        _point['x'] = wp_backward.transform.location.x
        _point['y'] = wp_backward.transform.location.y
        _point['z'] = wp_backward.transform.location.z + 2.0
        if target == 'vehicles':
            _point['roll'] = wp_backward.transform.rotation.roll
            _point['pitch'] = wp_backward.transform.rotation.pitch
            _point['yaw'] = wp_backward.transform.rotation.yaw
        return _point

    @classmethod
    def to_near_random(cls,
                       point: Dict[str, float],
                       RANGE: float = RANGE_PERTURB) -> Dict[str, float]:
        def _r(_range): return random.uniform(-_range, _range)
        point['x'] += _r(RANGE)
        point['y'] += _r(RANGE)
        # No mutation to pitch and roll
        if 'yaw' in point:
            point['yaw'] += _r(RANGE)
        return point

    @classmethod
    def to_random(cls, _map: str) -> Dict[str, float]:
        _sp = random.sample(cc.get_spawnable_points(_map), k=1).pop()
        try:
            sp = cc.get_the_nearest_wp(_sp.location, target='vehicles').transform
        except:
            import traceback
            traceback.print_exc()
            sp = _sp
        return {
            'x': sp.location.x,
            'y': sp.location.y,
            'z': sp.location.z + 2.0,
            'roll': sp.rotation.roll,
            'pitch': sp.rotation.pitch,
            'yaw': sp.rotation.yaw
        }

    @classmethod
    def to_random_pedestrian(cls, _map: str) -> Dict[str, float]:
        sp = random.sample(cc.get_spawnable_points(_map), k=1).pop()
        sp = cc.get_the_nearest_wp(sp.location, target='pedestrians').transform
        if sp is None:
            raise TryMutationAgain
        else:
            return {
                'x': sp.location.x,
                'y': sp.location.y,
                'z': sp.location.z + 2.0,
                'roll': sp.rotation.roll,
                'pitch': sp.rotation.pitch,
                'yaw': sp.rotation.yaw
            }


class MissionMutator(Mutator):

    @classmethod
    def mops_ego_start_to_forward(cls, scenario: Scenario) -> Scenario:
        # TODO: Check if mutation failed
        scenario.mission['start'] = PointMutator.to_forward(
            scenario.mission['start'])
        # scenario.mission['start']['z'] += 2.0
        scenario.update_history('mission', 'start_to_forward')
        return scenario

    @classmethod
    def mops_ego_start_to_backward(cls, scenario: Scenario) -> Scenario:
        scenario.mission['start'] = PointMutator.to_backward(
            scenario.mission['start'])
        # scenario.mission['start']['z'] += 2.0
        scenario.update_history('mission', 'start_to_backward')
        return scenario

    @classmethod
    def mops_ego_start_to_random(cls, scenario: Scenario) -> Scenario:
        scenario.mission['start'] = PointMutator.to_random(scenario.map)
        scenario.update_history('mission', 'start_to_random')
        return scenario

    @classmethod
    def mops_ego_dest_to_forward(cls, scenario: Scenario) -> Scenario:
        scenario.mission['dest'] = PointMutator.to_forward(
            scenario.mission['dest'])
        scenario.update_history('mission', 'dest_to_forward')
        return scenario

    @classmethod
    def mops_ego_dest_to_backward(cls, scenario: Scenario) -> Scenario:
        scenario.mission['dest'] = PointMutator.to_backward(
            scenario.mission['dest'])
        scenario.update_history('mission', 'dest_to_backward')
        return scenario

    @classmethod
    def mops_ego_dest_to_random(cls, scenario: Scenario) -> Scenario:
        scenario.mission['dest'] = PointMutator.to_random(scenario.map)
        scenario.update_history('mission', 'dest_to_random')
        return scenario


class NPCVehicleMutator(Mutator):

    @classmethod
    def check_trajectory_overlapping(cls,
                                     mission: Dict[str, Any],
                                     v: Dict[str, Any]) -> bool:
        return True

    @classmethod
    def __mapping(cls,
                  _map: str,
                  v: Dict[str, Any],
                  target: str,
                  to: str) -> Dict[str, Any]:
        if to == 'forward':
            return PointMutator.to_forward(v[target])
        elif to == 'backward':
            return PointMutator.to_backward(v[target])
        elif to == 'random':
            return PointMutator.to_random(_map)
        else:
            raise ValueError(f'Invalid target: {to}')

    @classmethod
    def __apply_mission_mutation(cls,
                                 scenario: Scenario,
                                 target: str,
                                 to: str,
                                 idx: int = -1) -> Scenario:
        if idx >= 0:
            i = idx
            v = scenario.npcs['vehicles'][i]
        else:
            if target == 'dest':
                i, v = cls.__select_npc_vehicle_one(
                    scenario, type_motion=DYNAMIC)
                if i < 0:
                    # No dynamic vehicle to mutate
                    target = 'start'
                    i, v = cls.__select_npc_vehicle_one(scenario)
            else:
                i, v = cls.__select_npc_vehicle_one(scenario)
        _v = deepcopy(v)
        # _v[target] = cls.__mapping(scenario.map, v, target, to)
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            _v[target] = cls.__mapping(scenario.map, v, target, to)
            if cls.check_trajectory_overlapping(scenario.mission, _v):
                break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        if v['type_motion'] == STATIC:
            # Start = Dest
            scenario.npcs['vehicles'][i]['start'] = deepcopy(_v[target])
            scenario.npcs['vehicles'][i]['dest'] = deepcopy(_v[target])
            # v['dest'] = deepcopy(v['start'])
        elif v['type_motion'] == LINEAR:
            # Set dest depending on the start point
            if to == 'random':
                # Randomly set the direction of the destination only when mutating to random
                _v[target]['yaw'] = random.randint(0, 360)
            scenario.npcs['vehicles'][i]['start'] = deepcopy(_v[target])
            start = get_carla_transform(scenario.npcs['vehicles'][i]['start'])
            loc_dest = start.location + start.get_forward_vector() * 100
            scenario.npcs['vehicles'][i]['dest'] = {
                'x': loc_dest.x,
                'y': loc_dest.y,
                'z': loc_dest.z,
                'roll': start.rotation.roll,
                'pitch': start.rotation.pitch,
                'yaw': start.rotation.yaw
            }
        else: # DYNAMIC
            scenario.npcs['vehicles'][i][target] = _v[target]
        scenario.update_history('npc_vehicle', f'{target}_to_{to}')
        return scenario

    @classmethod
    def mops_npc_mutate_start_to_random(cls,
                                        scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='random'
        )

    @classmethod
    def mops_npc_mutate_start_to_forward(cls,
                                         scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='forward'
        )

    @classmethod
    def mops_npc_mutate_start_to_backward(cls,
                                          scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='backward'
        )

    @classmethod
    def mops_npc_mutate_dest_to_forward(cls,
                                        scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='forward'
        )

    @classmethod
    def mops_npc_mutate_dest_to_backward(cls,
                                         scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='backward'
        )

    @classmethod
    def mops_npc_mutate_dest_to_random(cls,
                                       scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='random'
        )

    @classmethod
    def mops_npc_change_type_vehicle(cls,
                                     scenario: Scenario) -> Scenario:
        i, v = cls.__select_npc_vehicle_one(scenario)
        # Assure the vehicle type of the NPC vehicle is not duplicated
        _types = deepcopy(TYPES_VEHICLE)
        _types.remove(v['bp'])
        scenario.npcs['vehicles'][i]['bp'] = random.sample(_types, k=1).pop()
        scenario.update_history('npc_vehicle', 'type')
        return scenario

    @classmethod
    def mops_introduce_new_linear_vehicle(cls,
                                          scenario: Scenario) -> Scenario:
        bp = random.sample(TYPES_VEHICLE, k=1).pop()
        id_v = len(scenario.npcs['vehicles']) + \
            len(scenario.npcs['pedestrians']) + 1
        speed = random.uniform(0.83333, 8.33333)    # m/s, 3km/h ~ 30km/h
        _v = {
            "id": id_v,
            "type_motion": LINEAR,  # Linear npc vehicle
            "bp": bp,
            "start": {},
            "dest": {},
            "speed": speed
        }
        start = None
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            start, dest = cc.get_interactable_mission(scenario.map,
                                                      scenario.mission,
                                                      type_motion=LINEAR,
                                                      target='vehicle',
                                                      bp=bp,
                                                      traj_exp=scenario.traj_exp)
            if start is None or dest is None:
                continue
            _v['start'] = {
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                "roll": start.rotation.roll,
                "pitch": start.rotation.pitch,
                "yaw": start.rotation.yaw
            }
            _v['dest'] = {
                "x": dest.location.x,
                "y": dest.location.y,
                "z": dest.location.z,
                "roll": dest.rotation.roll,
                "pitch": dest.rotation.pitch,
                "yaw": dest.rotation.yaw
            }
            if cls.check_trajectory_overlapping(scenario.mission, _v):
                break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        scenario.npcs['vehicles'].append(_v)
        scenario.update_history('npc_vehicle', 'new_linear')
        return scenario

    @classmethod
    def mops_introduce_new_dynamic_npc_vehicle(cls,
                                               scenario: Scenario) -> Scenario:
        bp = random.sample(TYPES_VEHICLE, k=1).pop()
        id_v = len(scenario.npcs['vehicles']) + \
            len(scenario.npcs['pedestrians']) + 1
        _v = {
            "id": id_v,
            "type_motion": DYNAMIC,  # Dynamic npc vehicle
            "bp": bp,
            "start": {},
            "dest": {}
        }
        start, dest = None, None
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            start, dest = cc.get_interactable_mission(scenario.map,
                                                      scenario.mission,
                                                      type_motion=DYNAMIC,
                                                      target='vehicle',
                                                      bp=bp,
                                                      traj_exp=scenario.traj_exp)
            if start is None or dest is None:
                continue
            _v['start'] = {
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                "roll": start.rotation.roll,
                "pitch": start.rotation.pitch,
                "yaw": start.rotation.yaw
            }
            _v['dest'] = {
                "x": dest.location.x,
                "y": dest.location.y,
                "z": dest.location.z,
                "roll": dest.rotation.roll,
                "pitch": dest.rotation.pitch,
                "yaw": dest.rotation.yaw
            }
            if cls.check_trajectory_overlapping(scenario.mission, _v):
                break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        scenario.npcs['vehicles'].append(_v)
        scenario.update_history('npc_vehicle', 'new_dynamic')
        return scenario

    @classmethod
    def mops_introduce_new_static_npc_vehicle(cls,
                                              scenario: Scenario) -> Scenario:
        # No need to consider destination
        bp = random.sample(TYPES_VEHICLE, k=1).pop()
        id_v = len(scenario.npcs['vehicles']) + \
            len(scenario.npcs['pedestrians']) + 1
        start, _ = cc.get_interactable_mission(scenario.map,
                                               scenario.mission,
                                               type_motion=STATIC,
                                               target='vehicle',
                                               bp=bp,
                                               traj_exp=scenario.traj_exp)
        if start is None:
            raise TryMutationAgain
        _v = {
            "id": id_v,
            "type_motion": STATIC,
            "bp": bp,
            "start": {
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                "roll": start.rotation.roll,
                "pitch": start.rotation.pitch,
                "yaw": start.rotation.yaw
            },
            "dest": {   # Same as the start point
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                "roll": start.rotation.roll,
                "pitch": start.rotation.pitch,
                "yaw": start.rotation.yaw
            },
            "speed": 0.0
        }
        scenario.npcs['vehicles'].append(_v)
        scenario.update_history('npc_vehicle', 'new_static')
        return scenario

    @classmethod
    def __select_npc_vehicle_one(cls,
                                 scenario: Scenario,
                                 type_motion: str = '') -> Tuple[int, Dict[str, Any]]:
        if type_motion == DYNAMIC:
            vs = [i for i, v in enumerate(
                scenario.npcs['vehicles']) if v['type_motion'] == DYNAMIC]
        else:
            vs = [i for i, v in enumerate(
                scenario.npcs['vehicles']) if v['type_motion']]
        n = len(vs)
        if n <= 0:
            return -1, {}
        _i = random.sample(vs, k=1).pop()
        return _i, deepcopy(scenario.npcs['vehicles'][_i])

    @classmethod
    def mops_introduce_new(cls, scenario: Scenario) -> Scenario:
        _pool = ['static'] * 1 + ['linear'] * 3 + ['dynamic'] * 6
        _type = random.sample(_pool, k=1).pop()
        if _type == 'static':
            return cls.mops_introduce_new_static_npc_vehicle(scenario)
        elif _type == 'linear':
            return cls.mops_introduce_new_linear_vehicle(scenario)
        else:
            return cls.mops_introduce_new_dynamic_npc_vehicle(scenario)

    @classmethod
    def get_operators(cls, seed: Scenario) -> List[Callable[[Scenario], Scenario]]:
        mops = []
        # Introduce new NPC
        mops += [
            cls.mops_introduce_new,
            cls.mops_introduce_new,
            cls.mops_introduce_new,
        ]
        # If the scenario has at least one NPC Vehicle
        if len(seed.npcs['vehicles']) > 0:
            # Mission mutation
            mops += [
                cls.mops_npc_mutate_start_to_forward,
                cls.mops_npc_mutate_start_to_backward,
                cls.mops_npc_mutate_start_to_random,
                cls.mops_npc_mutate_dest_to_forward,
                cls.mops_npc_mutate_dest_to_backward,
                cls.mops_npc_mutate_dest_to_random
            ]
            # NPC type mutation
            mops += [cls.mops_npc_change_type_vehicle]
        return mops

    @classmethod
    def __npc_vehicle_mutate_start_to_random(cls,
                                             scenario: Scenario,
                                             idx: int) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='random',
            idx=idx
        )

    @classmethod
    def __npc_vehicle_mutate_dest_to_random(cls,
                                            scenario: Scenario,
                                            idx: int) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='random',
            idx=idx
        )

    @classmethod
    def get_mission_random_operators(cls, seed: Scenario) -> List[Callable[[Scenario, int], Scenario]]:
        if len(seed.npcs['vehicles']) > 0:
            return [
                lambda s, idx: cls.__npc_vehicle_mutate_start_to_random(
                    s, idx),
                lambda s, idx: cls.__npc_vehicle_mutate_dest_to_random(s, idx)
            ]
        else:
            return []


class NPCPedestrianMutator(Mutator):

    @classmethod
    def check_trajectory_overlapping(cls,
                                     mission: Dict[str, Any],
                                     n: Dict[str, Any]) -> bool:
        return True

    @classmethod
    def __mapping(cls,
                  _map: str,
                  n: Dict[str, Any],
                  target: str,
                  to: str) -> Dict[str, Any]:
        if to == 'forward':
            return PointMutator.to_forward(n[target], target='pedestrians')
        elif to == 'backward':
            return PointMutator.to_backward(n[target], target='pedestrians')
        elif to == 'random':
            return PointMutator.to_random_pedestrian(_map)
        else:
            raise ValueError(f'Invalid target: {to}')

    @classmethod
    def __apply_mission_mutation(cls,
                                 scenario: Scenario,
                                 target: str,
                                 to: str,
                                 idx: int = -1) -> Scenario:
        if idx >= 0:
            i = idx
            n = scenario.npcs['pedestrians'][i]
        else:
            if target == 'dest':
                i, n = cls.__select_npc_pedestrian_one(
                    scenario, type_motion=DYNAMIC)
                if i < 0:
                    # No dynamic pedestrian to mutate
                    target = 'start'
                    i, n = cls.__select_npc_pedestrian_one(scenario)
            else:
                i, n = cls.__select_npc_pedestrian_one(scenario)
        _n = deepcopy(n)
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            _n[target] = cls.__mapping(scenario.map, n, target, to)
            if cls.check_trajectory_overlapping(scenario.mission, _n):
                break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        if n['type_motion'] == STATIC:
            # Start = Dest
            scenario.npcs['pedestrians'][i]['start'] = deepcopy(_n[target])
            scenario.npcs['pedestrians'][i]['dest'] = deepcopy(_n[target])
        elif n['type_motion'] == LINEAR:
            # Set dest depending on the start point
            if to == 'random':
                # Randomly set the direction of the destination only when mutating to random
                _n[target]['yaw'] = random.randint(0, 360)
            scenario.npcs['pedestrians'][i]['start'] = deepcopy(_n[target])
            start = get_carla_transform(scenario.npcs['pedestrians'][i]['start'])
            loc_dest = start.location + start.get_forward_vector() * 100
            scenario.npcs['pedestrians'][i]['dest'] = {
                'x': loc_dest.x,
                'y': loc_dest.y,
                'z': loc_dest.z,
                'roll': start.rotation.roll,
                'pitch': start.rotation.pitch,
                'yaw': start.rotation.yaw
            }
        else: # DYNAMIC
            scenario.npcs['pedestrians'][i][target] = _n[target]
        scenario.update_history('npc_pedestrian', f'{target}_to_{to}')
        return scenario

    @classmethod
    def mops_npc_pedestrian_mutate_start_to_forward(cls,
                                                    scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='forward'
        )

    @classmethod
    def mops_npc_pedestrian_mutate_start_to_backward(cls,
                                                     scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='backward'
        )

    @classmethod
    def mops_npc_pedestrian_mutate_start_to_random(cls,
                                                   scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='random'
        )

    @classmethod
    def mops_npc_pedestrian_mutate_dest_to_forward(cls,
                                                   scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='forward'
        )

    @classmethod
    def mops_npc_pedestrian_mutate_dest_to_backward(cls,
                                                    scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='backward'
        )

    @classmethod
    def mops_npc_pedestrian_mutate_dest_to_random(cls,
                                                  scenario: Scenario) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='random'
        )

    @classmethod
    def __select_npc_pedestrian_one(cls,
                                    scenario: Scenario,
                                    type_motion: str = '') -> Tuple[int, Dict[str, Any]]:
        if type_motion == DYNAMIC:
            ps = [i for i, p in enumerate(scenario.npcs['pedestrians'])
                  if p['type_motion'] == DYNAMIC]
        else:
            ps = [i for i, p in enumerate(scenario.npcs['pedestrians'])
                  if p['type_motion']]
        n = len(ps)
        if n <= 0:
            return -1, {}
        # Recover original index
        _i = random.sample(ps, k=1).pop()
        return _i, deepcopy(scenario.npcs['pedestrians'][_i])

    @classmethod
    def mops_introduce_new_linear_npc_pedestrian(cls,
                                                 scenario: Scenario,
                                                 near_dist: float = NEAR_DISTANCE,
                                                 far_dist: float = FAR_DISTANCE) -> Scenario:
        bp = random.sample(TYPES_PEDESTRIAN, k=1).pop()
        id_v = len(scenario.npcs['vehicles']) + \
            len(scenario.npcs['pedestrians']) + 1
        speed = random.uniform(0.83333, 2.91667)  # m/s, 3km/h ~ 10km/h
        _n = {
            "id": id_v,
            "type_motion": LINEAR,  # Linear npc pedestrian
            "bp": bp,
            "start": {},
            "dest": {},
            "speed": speed
        }
        start = None
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            start, dest = cc.get_interactable_mission(scenario.map,
                                                      scenario.mission,
                                                      type_motion=LINEAR,
                                                      target='pedestrian',
                                                      bp=bp,
                                                      traj_exp=scenario.traj_exp)
            if start is None or dest is None:
                continue
            _n['start'] = {
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                "roll": start.rotation.roll,
                "pitch": start.rotation.pitch,
                "yaw": start.rotation.yaw
            }
            _n['dest'] = {
                "x": dest.location.x,
                "y": dest.location.y,
                "z": dest.location.z,
                "roll": dest.rotation.roll,
                "pitch": dest.rotation.pitch,
                "yaw": dest.rotation.yaw
            }
            if cls.check_trajectory_overlapping(scenario.mission, _n):
                break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        scenario.npcs['pedestrians'].append(_n)
        scenario.update_history('npc_pedestrian', 'new_linear')
        return scenario

    @classmethod
    def mops_introduce_new_dynamic_npc_pedestrian(cls,
                                                  scenario: Scenario,
                                                  near_dist: float = NEAR_DISTANCE,
                                                  far_dist: float = FAR_DISTANCE) -> Scenario:
        bp = random.sample(TYPES_PEDESTRIAN, k=1).pop()
        id_p = len(scenario.npcs['vehicles']) + \
            len(scenario.npcs['pedestrians']) + 1
        _n = {
            "id": id_p,
            "type_motion": DYNAMIC,
            "bp": bp,
            "start": {},
            "dest": {},
            "speed": random.uniform(1.25, 2.916667)
        }
        start, dest = None, None
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            start, dest = cc.get_interactable_mission(scenario.map,
                                                      scenario.mission,
                                                      type_motion=DYNAMIC,
                                                      target='pedestrian',
                                                      bp=bp,
                                                      traj_exp=scenario.traj_exp)
            if start is None or dest is None:
                continue
            _n['start'] = {
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                'roll': start.rotation.roll,
                'pitch': start.rotation.pitch,
                'yaw': start.rotation.yaw
            }
            _n['dest'] = {
                "x": dest.location.x,
                "y": dest.location.y,
                "z": dest.location.z,
                'roll': dest.rotation.roll,
                'pitch': dest.rotation.pitch,
                'yaw': dest.rotation.yaw
            }
            if cls.check_trajectory_overlapping(scenario.mission, _n):
                break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        scenario.npcs['pedestrians'].append(_n)
        scenario.update_history('npc_pedestrian', 'new_dynamic')
        return scenario

    @classmethod
    def mops_introduce_new_static_npc_pedestrian(cls,
                                                 scenario: Scenario,
                                                 near_dist: float = NEAR_DISTANCE,
                                                 far_dist: float = FAR_DISTANCE) -> Scenario:
        bp = random.sample(TYPES_PEDESTRIAN, k=1).pop()
        id_p = len(scenario.npcs['vehicles']) + \
            len(scenario.npcs['pedestrians']) + 1
        start, _ = cc.get_interactable_mission(scenario.map,
                                               scenario.mission,
                                               type_motion=STATIC,
                                               target='pedestrian',
                                               bp=bp,
                                               traj_exp=scenario.traj_exp)
        if start is None:
            raise TryMutationAgain
        _n = {
            "id": id_p,
            "type_motion": STATIC,
            "bp": bp,
            "start": {
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z + 2.0,
                'roll': start.rotation.roll,
                'pitch': start.rotation.pitch,
                'yaw': start.rotation.yaw
            },
            "dest": {   # Same as the start point
                "x": start.location.x,
                "y": start.location.y,
                "z": start.location.z,
                'roll': start.rotation.roll,
                'pitch': start.rotation.pitch,
                'yaw': start.rotation.yaw
            },
            "speed": 0.0
        }
        scenario.npcs['pedestrians'].append(_n)
        scenario.update_history('npc_pedestrian', 'new_static')
        return scenario

    @classmethod
    def mops_npc_change_type_pedestrian(cls,
                                        scenario: Scenario) -> Scenario:
        i, p = cls.__select_npc_pedestrian_one(scenario)
        _types = deepcopy(TYPES_PEDESTRIAN)
        _types.remove(p['bp'])
        scenario.npcs['pedestrians'][i]['bp'] = random.sample(
            _types, k=1).pop()
        scenario.update_history('npc_pedestrian', 'type')
        return scenario

    @classmethod
    def mops_introduce_new(cls, scenario: Scenario) -> Scenario:
        _pool = ['static'] * 2 + ['linear'] * 3 + ['dynamic'] * 5
        _type = random.sample(_pool, k=1).pop()
        if _type == 'static':
            return cls.mops_introduce_new_static_npc_pedestrian(scenario)
        elif _type == 'linear':
            return cls.mops_introduce_new_linear_npc_pedestrian(scenario)
        else:
            return cls.mops_introduce_new_dynamic_npc_pedestrian(scenario)

    @classmethod
    def get_operators(cls, seed: Scenario) -> List[Callable[[Scenario], Scenario]]:
        mops = []
        # Introduce new NPC
        mops += [
            cls.mops_introduce_new,
            cls.mops_introduce_new,
            cls.mops_introduce_new
        ]
        # If the scenario has at least one NPC pedestrian
        if len(seed.npcs['pedestrians']) > 0:
            # Mission mutation
            mops += [
                cls.mops_npc_pedestrian_mutate_start_to_random,
                cls.mops_npc_pedestrian_mutate_start_to_forward,
                cls.mops_npc_pedestrian_mutate_start_to_backward,
                cls.mops_npc_pedestrian_mutate_dest_to_forward,
                cls.mops_npc_pedestrian_mutate_dest_to_backward,
                cls.mops_npc_pedestrian_mutate_dest_to_random,
                cls.mops_npc_change_type_pedestrian
            ]
        return mops

    @classmethod
    def __npc_pedestrian_mutate_start_to_random(cls,
                                                scenario: Scenario,
                                                idx: int) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='start',
            to='random',
            idx=idx
        )

    @classmethod
    def __npc_pedestrian_mutate_dest_to_random(cls,
                                               scenario: Scenario,
                                               idx: int) -> Scenario:
        return cls.__apply_mission_mutation(
            scenario=scenario,
            target='dest',
            to='random',
            idx=idx
        )

    @classmethod
    def get_mission_random_operators(cls, seed: Scenario) -> List[Callable[[Scenario, int], Scenario]]:
        if len(seed.npcs['pedestrians']) > 0:
            return [
                lambda s, idx: cls.__npc_pedestrian_mutate_start_to_random(
                    s, idx),
                lambda s, idx: cls.__npc_pedestrian_mutate_dest_to_random(
                    s, idx)
            ]
        else:
            return []


class PuddleMutator(Mutator):

    @classmethod
    def mops_introduce_new_puddle(cls,
                                  scenario: Scenario,
                                  near_dist: float = NEAR_DISTANCE) -> Scenario:
        _puddle = {
            "level": random.uniform(0, 0.5), #random.randint(0, 200) / 100,
            "x": -1,
            "y": -1,
            "z": -1,
            "size_x": random.uniform(400, 600),
            "size_y": random.uniform(400, 600),
            "size_z": 1000.0
        }
        trial = 0
        while trial < NUM_TRIAL_MUTATION:
            trial += 1
            sp, _ = cc.get_interactable_mission(scenario.map,
                                                scenario.mission,
                                                type_motion=STATIC,
                                                target='puddle',
                                                bp='puddle',
                                                traj_exp=scenario.traj_exp)
            # Keep puddles away from npcs' starting points
            for v in scenario.npcs['vehicles'] + scenario.npcs['pedestrians']:
                sp_npc = get_carla_transform(v['start'])
                if sp_npc.location.distance(sp.location) < 10.0:
                    print("Puddle Again")
                    sp = None
                    break
            if sp is None:
                continue
            _puddle['x'] = sp.location.x
            _puddle['y'] = sp.location.y
            _puddle['z'] = sp.location.z
            break
        if trial >= NUM_TRIAL_MUTATION:
            raise TryMutationAgain
        scenario.puddles.append(_puddle)
        scenario.update_history('puddle', 'new')
        return scenario

    @classmethod
    def mops_puddle_to_near(cls,
                            scenario: Scenario) -> Scenario:
        i, puddle = cls.__select_puddle_one(scenario)
        p = PointMutator.to_near_random(
            {
                'x': puddle['x'],
                'y': puddle['y'],
                'z': puddle['z']
            }
        )
        puddle['x'] = p['x']
        puddle['y'] = p['y']
        puddle['z'] = p['z']
        scenario.puddles[i] = puddle
        scenario.update_history('puddle', 'to_near')
        return scenario

    @classmethod
    def mops_puddle_to_random(cls,
                              scenario: Scenario) -> Scenario:
        i, puddle = cls.__select_puddle_one(scenario)
        p = PointMutator.to_random(scenario.map)
        puddle['x'] = p['x']
        puddle['y'] = p['y']
        puddle['z'] = p['z']
        scenario.puddles[i] = puddle
        scenario.update_history('puddle', 'to_random')
        return scenario

    @classmethod
    def mops_puddle_mutate_size(cls,
                                scenario: Scenario) -> Scenario:
        i, puddle = cls.__select_puddle_one(scenario)
        puddle['size_x'] = random.uniform(400, 600)
        puddle['size_y'] = random.uniform(400, 600)
        # size_z is fixed
        scenario.puddles[i] = puddle
        scenario.update_history('puddle', 'size')
        return scenario

    @classmethod
    def mops_puddle_mutate_friction(cls,
                                    scenario: Scenario) -> Scenario:
        i, puddle = cls.__select_puddle_one(scenario)
        puddle['level'] = random.uniform(0, 0.2) #random.randint(0, 200) / 100
        scenario.puddles[i] = puddle
        scenario.update_history('puddle', 'friction')
        return scenario

    @classmethod
    def __select_puddle_one(cls, scenario: Scenario) -> Tuple[int, Dict[str, Any]]:
        n = len(scenario.puddles)
        assert n > 0
        _i = random.randint(0, n - 1)
        return _i, deepcopy(scenario.puddles[_i])

    @classmethod
    def get_operators(cls, seed: Scenario) -> List[Callable[[Scenario], Scenario]]:
        mops = []
        mops += [cls.mops_introduce_new_puddle]
        if len(seed.puddles) > 0:
            mops += [
                lambda s: cls.mops_puddle_to_near(s),
                lambda s: cls.mops_puddle_mutate_size(s),
                lambda s: cls.mops_puddle_mutate_friction(s),
                lambda s: cls.mops_puddle_to_random(s)
            ]
        return mops


class WeatherMutator(Mutator):

    @classmethod
    def mops_weather_cloudiness(cls, scenario: Scenario) -> Scenario:
        scenario.weather['cloudiness'] = random.uniform(0, 100)
        scenario.update_history('weather', 'cloudiness')
        return scenario

    @classmethod
    def mops_weather_rain(cls, scenario: Scenario) -> Scenario:
        scenario.weather['precipitation'] = random.uniform(0, 100)
        scenario.update_history('weather', 'rain')
        return scenario

    @classmethod
    def mops_weather_puddle(cls, scenario: Scenario) -> Scenario:
        scenario.weather['precipitation_deposits'] = random.uniform(0, 100)
        scenario.update_history('weather', 'puddle')
        return scenario

    @classmethod
    def mops_weather_wind(cls, scenario: Scenario) -> Scenario:
        scenario.weather['wind_intensity'] = random.uniform(0, 100)
        scenario.update_history('weather', 'wind')
        return scenario

    @classmethod
    def mops_weather_fog(cls, scenario: Scenario) -> Scenario:
        scenario.weather['fog_density'] = random.uniform(0, 100)
        scenario.update_history('weather', 'fog')
        return scenario

    @classmethod
    def mops_weather_wetness(cls, scenario: Scenario) -> Scenario:
        scenario.weather['wetness'] = random.uniform(0, 100)
        scenario.update_history('weather', 'wetness')
        return scenario

    # Only supported on Carla >= 0.9.14, not on 0.9.13
    @classmethod
    def mops_weather_dust_storm(cls, scenario: Scenario) -> Scenario:
        """
        Mutate the dust storm of the weather. Range: [0, 100]
            - 0: No dust storm
            - 100: Completely dust storm

        :returns: The scenario mutated the dust storm of the weather.
        :rtype: Scenario
        """
        scenario.weather['dust_storm'] = random.uniform(0, 100)
        scenario.update_history('weather', 'dust_storm')
        return scenario


class TimeMutator(Mutator):

    @classmethod
    def mops_time_sun_azimuth_angle(cls, scenario: Scenario) -> Scenario:
        scenario.time['sun_azimuth_angle'] = random.uniform(0, 360)
        scenario.update_history('time', 'sun_azimuth_angle')
        return scenario

    @classmethod
    def mops_time_sun_altitude_angle(cls, scenario: Scenario) -> Scenario:
        scenario.time['sun_altitude_angle'] = random.uniform(-90, 90)
        scenario.update_history('time', 'sun_altitude_angle')
        return scenario


def check_validity_spawning_point(mutant: Scenario,
                                  npcs_only: bool = False) -> bool:
    # sp of the Ego vehicle
    sps = [get_carla_location(mutant.mission['start'])]
    # sp of NPC vehicles
    for v in mutant.npcs['vehicles']:
        sps.append(get_carla_location(v['start']))
    # sp of Pedestrians
    for p in mutant.npcs['pedestrians']:
        sps.append(get_carla_location(p['start']))
    # Check if there's any sps too close to each other
    for i in range(len(sps)):
        for j in range(len(sps)):
            if i != j:
                # If too close, return False
                if sps[i].distance(sps[j]) < FAR_DISTANCE:
                    return False
    if npcs_only:
        return True
    # Puddles should not be close each other
    sps = [get_carla_location(puddle) for puddle in mutant.puddles]
    for i in range(len(sps)):
        for j in range(len(sps)):
            if i != j:
                # If too close, return False
                if sps[i].distance(sps[j]) < FAR_DISTANCE * 5:
                    return False
    return True


def check_validity_mission(mutant: Scenario,
                           ads: ADS) -> bool:
    loc_s = get_carla_location(mutant.mission['start'])
    loc_d = get_carla_location(mutant.mission['dest'])
    if loc_s.distance(loc_d) < 5.0:
        return False
    wp_s = cc.get_map().get_waypoint(loc_s)
    wp_d = cc.get_map().get_waypoint(loc_d)
    dir_s = wp_s.transform.rotation.get_forward_vector().make_unit_vector()
    dir_ds = (wp_s.transform.location -
              wp_d.transform.location).make_unit_vector()
    angle = math.degrees(dir_s.get_vector_angle(dir_ds))
    cond_invalid = wp_s.road_id == wp_d.road_id and wp_s.lane_id == wp_d.lane_id and (
        angle < 10 if not math.isnan(angle) else True)
    if cond_invalid:
        return False
    try:
        wps = ads.plan_trajectory(mutant.mission)
    except PlanningAgain as e:
        import traceback
        traceback.print_exc()
        get_logger().info(f"Planning failed: {e}")
        raise TryMutationAgain
    mutant.traj_exp = wps
    _dist = 0.0
    wp_prev = wps[0]
    for i in range(1, len(wps)):
        _dist += wp_prev.location.distance(
            wps[i].location)
        if _dist > 50.0:
            break
        wp_prev = wps[i]
    else:
        if _dist < 50.0:
            return False
    return True


def check_trajectory_overlapping(scenario: Scenario) -> bool:
    return True


def generate_mutants(seed: Scenario,
                     ads: ADS,
                     data_generation: bool = False) -> Generator[Scenario, None, None]:
    seed.traj_exp = []
    """
    Generate mutants from the given seed and rank them by the given model's prediction score, if provided.

    :returns: List of mutants
    :rtype: List[Scenario]
    """
    # Aggregate all mutation operators
    if data_generation:
        mops = {
            'mission': [
                lambda s: MissionMutator.mops_ego_dest_to_random(
                    MissionMutator.mops_ego_start_to_random(s)),
            ]
        }
    else:
        mops = {
            'mission': MissionMutator.get_operators(),
            'vehicle': NPCVehicleMutator.get_operators(seed),
            'pedestrian': NPCPedestrianMutator.get_operators(seed),
            'weather': WeatherMutator.get_operators(),
            'time': TimeMutator.get_operators(),
            'puddle': PuddleMutator.get_operators(seed)
        }
    get_logger().info(f'# mutation operators: ' +
                      ' + '.join([f'{len(v)}({k})' for k, v in mops.items()]))
    # Apply each operators to the given seed scenario
    record = {
        'mission_done': False,
        'vehicle_done': False,
        'pedestrian_done': False,
        'weather_done': False,
        'time_done': False,
        'puddle_done': False
    }
    ''' Apply operators from each category '''

    while True:
        try:
            if all(record.values()):
                record = {
                    'mission_done': False,
                    'vehicle_done': False,
                    'pedestrian_done': False,
                    'weather_done': False,
                    'time_done': False,
                    'puddle_done': False
                }
            _mutant = deepcopy(seed)
            # Randomly select and apply one operator from each category.
            if not record['mission_done']:
                mutant = random.sample(
                    mops['mission'], k=1).pop()(deepcopy(_mutant))
                if ads.id == 'autoware':
                    if not check_validity_mission(mutant, ads):
                        continue
                record['mission_done'] = True
            if not data_generation:
                _vehicles = deepcopy(mutant.npcs['vehicles'])
                _pedestrians = deepcopy(mutant.npcs['pedestrians'])
                if not record['vehicle_done']:
                    for i, v in enumerate(mutant.npcs['vehicles']):
                        while not NPCVehicleMutator.check_trajectory_overlapping(mutant.mission, v):
                            mutant = random.sample(
                                mops_aux['vehicle_mission_random_only'], k=1).pop()(mutant, i)
                    else:
                        mutant = random.sample(
                            mops['vehicle'], k=1).pop()(mutant)
                    record['vehicle_done'] = True
                    # print("\rMission done Vehicle done", end='')
                if not record['pedestrian_done']:
                    for i, n in enumerate(mutant.npcs['pedestrians']):
                        while not NPCPedestrianMutator.check_trajectory_overlapping(mutant.mission, n):
                            mutant = random.sample(
                                mops_aux['pedestrian_mission_random_only'], k=1).pop()(mutant, i)
                    else:
                        mutant = random.sample(
                            mops['pedestrian'], k=1).pop()(mutant)
                    record['pedestrian_done'] = True
                # Check if the trajectory of the ego vehicle crosses with the trajectory of NPC vehicles.
                if not check_trajectory_overlapping(mutant):
                    get_logger().info("No overlap with npc")
                    record['vehicle_done'] = False
                    record['pedestrian_done'] = False
                    mutant.npcs['vehicles'] = _vehicles
                    mutant.npcs['pedestrians'] = _pedestrians
                    mutant.hist_mops['npc_vehicle'] = mutant.hist_mops['npc_vehicle'][:-1]
                    mutant.hist_mops['npc_pedestrian'] = mutant.hist_mops['npc_pedestrian'][:-1]
                    continue
                if not check_validity_spawning_point(mutant, npcs_only=True):
                    record['vehicle_done'] = False
                    record['pedestrian_done'] = False
                    mutant.npcs['vehicles'] = _vehicles
                    mutant.npcs['pedestrians'] = _pedestrians
                    mutant.hist_mops['npc_vehicle'] = mutant.hist_mops['npc_vehicle'][:-1]
                    mutant.hist_mops['npc_pedestrian'] = mutant.hist_mops['npc_pedestrian'][:-1]
                    continue
                if not record['weather_done']:
                    mutant = random.sample(mops['weather'], k=1).pop()(mutant)
                    record['weather_done'] = True
                if not record['time_done']:
                    mutant = random.sample(mops['time'], k=1).pop()(mutant)
                    record['time_done'] = True
                if not record['puddle_done']:
                    mutant = random.sample(mops['puddle'], k=1).pop()(mutant)
                    record['puddle_done'] = True
            if not check_validity_spawning_point(mutant):
                continue
            if mutant == _mutant:
                continue
            yield mutant
        except TryMutationAgain as e:
            get_logger().info("Try mutation again")
            continue
