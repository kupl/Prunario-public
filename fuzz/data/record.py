from __future__ import annotations
from itertools import zip_longest
import json
from pathlib import Path
import time
from typing import Any, Dict, List, Tuple, Union

from fuzz.commons.constants import *
from fuzz.carla.utils import *
from fuzz.commons.constants import *
from fuzz.commons.exceptions import NoRecordException, RecordLoadingException


class Scenario:

    def __init__(self, 
                 path_scenario: Path, 
                 name: str=NAME_FILE_SCENARIO,
                 empty: bool=False) -> None:
        self.path_scenario: Path = path_scenario
        if empty:   # For DriveFuzz
            js = {}
        elif path_scenario.is_file():
            js = json.load((self.path_scenario).open())
        else:
            js = json.load((self.path_scenario / name).open())
        # else:
        #     js = {}
        # # of mutation
        self.num_mutation: int = js['num_mutation'] if 'num_mutation' in js else 0
        # Map
        self.map: str = js['map'] if not empty else ''
        # Mission
        self.mission: Dict[str, Dict[str, float]] = js['mission'] if not empty else {}
        # NPCs
        self.npcs: Dict[str, List[Dict[str, Any]]] = js['npcs'] if not empty else {}
        # Weather
        self.weather: Dict[str, float] = js['weather'] if not empty else {}
        # Time
        self.time: Dict[str, float] = js['time'] if not empty else {}
        # Puddles
        self.puddles: List[Dict[str, float]] = js['puddles'] if not empty else []
        # Applied mutation operators
        # self.hist_mops: Dict[int, List[str]] = js['hist_mops'] 
        self.hist_mops: Dict[str, List[str]] = js['hist_mops'] if not empty else {}
        # expected trajectory
        self.traj_exp: List[carla.Transform] = []
        if (path_scenario / 'expected_trajectory.json').is_file():
            with (path_scenario / 'expected_trajectory.json').open() as f:
                _wps = json.load(f)
                for _wp in _wps:
                    self.traj_exp.append(carla.Transform(
                        carla.Location(x=_wp['x'], y=_wp['y'], z=_wp['z']),
                        carla.Rotation(roll=_wp['roll'], pitch=_wp['pitch'], yaw=_wp['yaw'])
                    ))

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
    
    def __hash__(self) -> int:
        return hash(self.__str__())

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Scenario):
            return False
        return self.__str__ == value.__str__

    def dump(self,
             p: Path,
             name: str = NAME_FILE_SCENARIO) -> None:
        # _p = deepcopy(self.path_scenario)
        _log = {
            'num_mutation': self.num_mutation,
            'map': self.map,
            'mission': self.mission,
            'npcs': self.npcs,
            'weather': self.weather,
            'time': self.time,
            'puddles': self.puddles,
            'hist_mops': self.hist_mops
        }
        with (p / name).open('w') as f:
            json.dump(_log, f, indent=4)
        with (p / 'expected_trajectory.json').open('w') as f:
            _wps = [
                {
                    'x': _wp.location.x,
                    'y': _wp.location.y,
                    'z': _wp.location.z,
                    'roll': _wp.rotation.roll,
                    'pitch': _wp.rotation.pitch,
                    'yaw': _wp.rotation.yaw
                } for _wp in self.traj_exp
            ]
            json.dump(_wps, f, indent=4)
        # del self.path_scenario
        # with (p / name).open('w') as f:
        #     json.dump(self.__dict__, f, indent=4)
        # self.path_scenario = _p

    def update_history(self, m_category: str, mop: str) -> None:
        if m_category not in self.hist_mops:
            self.hist_mops[m_category] = [mop]
        else:
            self.hist_mops[m_category].append(mop)

    def ddredundant_autofuzz(self,
                             s: Scenario) -> bool:
        TH1 = 0.1
        TH2 = 0.5
        res: List[bool] = []    # aggregate if each field is similar or not (True if different)
        # Mission
        res.extend([
            abs(self.mission['start']['x'] - s.mission['start']['x']) / XYZRANGE[self.map]['x'] > TH2,
            abs(self.mission['start']['y'] - s.mission['start']['y']) / XYZRANGE[self.map]['y'] > TH2,
            abs(self.mission['start']['z'] - s.mission['start']['z']) / XYZRANGE[self.map]['z'] > TH2,
            abs(self.mission['start']['roll'] % 360 - s.mission['start']['roll'] % 360) / 360 > TH2,
            abs(self.mission['start']['pitch'] % 360 - s.mission['start']['pitch'] % 360) / 360 > TH2,
            abs(self.mission['start']['yaw'] % 360 - s.mission['start']['yaw'] % 360) / 360 > TH2,
            abs(self.mission['dest']['x'] - s.mission['dest']['x']) / XYZRANGE[self.map]['x'] > TH2,
            abs(self.mission['dest']['y'] - s.mission['dest']['y']) / XYZRANGE[self.map]['y'] > TH2,
            abs(self.mission['dest']['z'] - s.mission['dest']['z']) / XYZRANGE[self.map]['z'] > TH2,
            abs(self.mission['dest']['roll'] % 360 - s.mission['dest']['roll'] % 360) / 360 > TH2,
            abs(self.mission['dest']['pitch'] % 360 - s.mission['dest']['pitch'] % 360) / 360 > TH2,
            abs(self.mission['dest']['yaw'] % 360 - s.mission['dest']['yaw'] % 360) / 360 > TH2
        ])
        # NPCs
        for target in ['vehicles', 'pedestrians']:
            for n1, n2 in zip_longest(self.npcs[target], s.npcs[target]):
                if n1 is None:
                    res.extend([True]*15)  # all different
                    continue
                elif n2 is None:
                    res.extend([True]*15)  # all different
                    continue
                res.extend([
                    n1['type_motion'] != n2['type_motion'],
                    n1['bp'] != n2['bp'],
                    abs(n1['start']['x'] - n2['start']['x']) / XYZRANGE[self.map]['x'] > TH2,
                    abs(n1['start']['y'] - n2['start']['y']) / XYZRANGE[self.map]['y'] > TH2,
                    abs(n1['start']['z'] - n2['start']['z']) / XYZRANGE[self.map]['z'] > TH2,
                    abs(n1['start']['roll'] % 360 - n2['start']['roll'] % 360) / 360 > TH2,
                    abs(n1['start']['pitch'] % 360 - n2['start']['pitch'] % 360) / 360 > TH2,
                    abs(n1['start']['yaw'] % 360 - n2['start']['yaw'] % 360) / 360 > TH2,
                    abs(n1['dest']['x'] - n2['dest']['x']) / XYZRANGE[self.map]['x'] > TH2,
                    abs(n1['dest']['y'] - n2['dest']['y']) / XYZRANGE[self.map]['y'] > TH2,
                    abs(n1['dest']['z'] - n2['dest']['z']) / XYZRANGE[self.map]['z'] > TH2,
                    abs(n1['dest']['roll'] % 360 - n2['dest']['roll'] % 360) / 360 > TH2,
                    abs(n1['dest']['pitch'] % 360 - n2['dest']['pitch'] % 360) / 360 > TH2,
                    abs(n1['dest']['yaw'] % 360 - n2['dest']['yaw'] % 360) / 360 > TH2
                ])
                _speed1 = n1['speed'] if 'speed' in n1 else 0.0
                _speed2 = n2['speed'] if 'speed' in n2 else 0.0
                if target == 'vehicles': 
                    res.append(abs(_speed1 - _speed2) / 8.33333 > TH2)
                else:
                    res.append(abs(_speed1 - _speed2) / 2.91667 > TH2)
        # Weather
        res.extend([
            abs(self.weather['cloudiness'] - s.weather['cloudiness']) / 100 > TH2,
            abs(self.weather['precipitation'] - s.weather['precipitation']) / 100 > TH2,
            abs(self.weather['precipitation_deposits'] - s.weather['precipitation_deposits']) / 100 > TH2,
            abs(self.weather['wind_intensity'] - s.weather['wind_intensity']) / 100 > TH2,
            abs(self.weather['fog_density'] - s.weather['fog_density']) / 100 > TH2,
            abs(self.weather['wetness'] - s.weather['wetness']) / 100 > TH2,
            abs(self.weather['dust_storm'] - s.weather['dust_storm']) / 100 > TH2
        ])
        # Time
        res.extend([
            abs(self.time['sun_azimuth_angle'] - s.time['sun_azimuth_angle']) / 360 > TH2,
            abs(self.time['sun_altitude_angle'] - s.time['sun_altitude_angle']) / 180 > TH2
        ])
        # Puddles
        for p1, p2 in zip_longest(self.puddles, s.puddles):
            if p1 is None:
                res.extend([True]*7)  # all different
                continue
            elif p2 is None:
                res.extend([True]*7)  # all different
                continue
            res.extend([
                abs(p1['level'] - p2['level']) / 0.5 > TH2,
                abs(p1['x'] - p2['x']) / XYZRANGE[self.map]['x'] > TH2,
                abs(p1['y'] - p2['y']) / XYZRANGE[self.map]['y'] > TH2,
                abs(p1['z'] - p2['z']) / XYZRANGE[self.map]['z'] > TH2,
                abs(p1['size_x'] - p2['size_x']) / 200 > TH2,
                abs(p1['size_y'] - p2['size_y']) / 200 > TH2,
                abs(p1['size_z'] - p2['size_z']) / 1000 > TH2
            ])
        return (res.count(True) / len(res)) < TH1


class State:

    def __init__(self, path_state: Path, other: bool=False) -> None:
        if other:
            self.__init_other(path_state)
        else:
            # Path to the state
            self.path_state: Path = path_state
            self.__idx_curr: int = 0
            with path_state.open() as f:
                try:
                    _state = json.load(f)
                    # Start time of the simulation
                    self.time_start: float = _state['time_start']
                    # Map
                    self.map: str = _state['map']
                    # State
                    self.state: List[Dict[str, Any]] = _state['state']
                    # Bounding boxes
                    self.bounding_boxes: Dict[str, Dict[str,
                                                        Dict[str, float]]] = _state['bounding_boxes']
                    # Violation informations
                    self.result: Dict[str, Any] = _state['result']
                    # End time of the simulation
                    self.time_end: float = _state['time_end']
                except json.decoder.JSONDecodeError as _:
                    raise RecordLoadingException(str(path_state))

    def __init_other(self, path_state: Path) -> None:
        self.path_state = path_state.parent
        self.__idx_curr: int = 0
        with path_state.open() as f:
            _state = json.load(f)
            self.time_start = 0.0
            self.state: List[Dict[str, Any]] = []
            self.map: str = _state['map_name']
            points_ego = _state['object_trajectory']['ego_car']['trajectory']
            speeds_ego = _state['vehicle_states'] ['speed']
            pns_vehicles = {
                _id: _s for _id, _s in _state['object_trajectory']['vehicles'].items()
            }
            pns_peds = {
                _id: _s for _id, _s in _state['object_trajectory']['walkers'].items()
            }
            for i in range(len(points_ego)):
                point_ego = points_ego[i]
                speed_ego = speeds_ego[i]
                state_vehicles = []
                for _id, _state_vehicle in pns_vehicles.items():
                    _type_motion, _pns = _state_vehicle.values()
                    if _type_motion.lower() in ['immobile', 'static']:
                        type_motion = STATIC
                    elif _type_motion.lower() in ['linear']:
                        type_motion = LINEAR
                    else:   # autopilot or maneuver
                        type_motion = DYNAMIC
                    __pns = _pns[i] if i < len(_pns) else _pns[-1]
                    __p = __pns[:-2]
                    __speed = __pns[-2] if i < len(_pns) else 0.0
                    __type = __pns[-1]
                    state_vehicles.append({
                        "id": _id,
                        "type": __type,
                        'type_motion': type_motion,
                        "point": {
                            'x': __p[0],
                            'y': __p[1],
                            'z': __p[2],
                            'roll': __p[3],
                            'pitch': __p[4],
                            'yaw': __p[5]
                        },
                        'speed': __speed
                    })
                state_peds = []
                for _id, _state_pedestrian in pns_peds.items():
                    _type_motion, _pns = _state_pedestrian.values()
                    if _type_motion.lower() in ['immobile', 'static']:
                        type_motion = STATIC
                    elif _type_motion.lower() in ['linear']:
                        type_motion = LINEAR
                    else:   # autopilot or maneuver
                        type_motion = DYNAMIC
                    __pns = _pns[i] if i < len(_pns) else _pns[-1]
                    __p = __pns[:-2]
                    __speed = __pns[-2] if i < len(_pns) else 0.0
                    __type = __pns[-1]
                    state_peds.append({
                        "id": _id,
                        "type": __type,
                        'type_motion': type_motion,
                        "point": {
                            'x': __p[0],
                            'y': __p[1],
                            'z': __p[2],
                            'roll': __p[3],
                            'pitch': __p[4],
                            'yaw': __p[5]
                        },
                        'speed': __speed
                    })
                self.state.append({
                    'point': {
                        'x': point_ego[0],
                        'y': point_ego[1],
                        'z': point_ego[2],
                        'roll': point_ego[3],
                        'pitch': point_ego[4],
                        'yaw': point_ego[5]
                    },
                    'speed': speed_ego,
                    'vehicles': state_vehicles,
                    'pedestrians': state_peds
                })

            self.bounding_boxes = {}
            res = _state['events']
            self.result = {
                "collision": res['crash'],
                "collision_with": res['crash_with'],
                "stalling": res['stuck'],
                "speeding": res['speeding'],
                "lane_invasion": res['lane_invasion']
            }
            self.time_end = 0.0

    @classmethod
    def from_json(cls, path: Union[str, Path]) -> State:
        if isinstance(path, str):
            path = Path(path)
        state = State(path)
        return state

    def any_violation(self) -> bool:
        return self.result['collision'] or self.result['collision_with'] or self.result['stalling'] or self.result['speeding']

    def __iter__(self) -> State:
        return self

    def __next__(self) -> Dict[str, Any]:
        if self.__idx_curr == len(self.state):
            raise StopIteration
        else:
            self.__idx_curr += 1
            return self.state[self.__idx_curr-1]

    def __len__(self) -> int:
        return len(self.state)

    def __getitem__(self, idx: Any) -> Any:
        return self.state[idx]

    def __str__(self) -> str:
        return str(self.__dict__)

    def update(self, state_t: Dict[str, Any]) -> None:
        self.state.append(state_t)

    def dump(self) -> None:
        self.time_end = time.time()
        with (self.path_state / NAME_FILE_STATE).open('w') as f:
            del self.path_state
            json.dump(self.__dict__, f)


class Record:

    def __init__(self, path_mb: Path, other: bool=False) -> None:
        if other:
            self.__init_other(path_state=path_mb)
        else:
            self.path_mb: Path = path_mb
            self.any_violation: bool = (path_mb / NAME_FILE_VIOLATION).is_file()
            if self.any_violation:
                self.violation: str = (
                    path_mb / NAME_FILE_VIOLATION).open().read().strip()
            else:
                self.violation: str = ''
            self.state: State = State.from_json(path_mb / NAME_FILE_STATE)
            self.map: str = self.state.map

    def __init_other(self, path_state: Path) -> None:
        self.state: State = State(path_state, other=True)
        self.path_mb: Path = path_state.parent
        self.any_violation = self.state.any_violation()
        self.map: str = self.state.map
        if self.state.result['collision']:
            if 'vehicle' in self.state.result['collision_with']:
                self.violation = 'collision_vehicle'
            elif 'walker' in self.state.result['collision_with']:
                self.violation = 'collision_pedestrian'
            else:
                self.violation = 'collision'
        elif self.state.result['stalling']:
            self.violation = 'stalling'
        elif self.state.result['speeding']:
            self.violation = 'speeding'
        elif self.state.result['lane_invasion']:
            self.violation = 'lane_invasion'
        else:
            self.violation = ''