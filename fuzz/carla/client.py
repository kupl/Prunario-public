import math
import random
from typing import Dict, List, Tuple, Union

import carla
import numpy as np

from fuzz.carla.utils import GlobalRoutePlanner, RoadOption, get_carla_location, get_carla_transform, transform_to_dict
from fuzz.commons.constants import *
from fuzz.commons.exceptions import LoadWorldException
from fuzz.commons.utils import get_logger
# from fuzz.simulation.utils import distance_2d


class __CarlaClient:

    def __init__(self,
                 host: str = 'localhost',
                 port: int = 2000) -> None:
        self.__client: carla.Client = carla.Client(host=host, port=port)
        self.__world: carla.World = self.__client.get_world()
        self.__map: carla.Map = self.__world.get_map()
        self.__spawnable_points: List[carla.Transform] = self.__map.get_spawn_points(
        )
        self.__obstacles: List[carla.BoundingBox] = sum([self.__world.get_level_bbs(obs) for obs in CARLA_OBSTACLES], [])
        for ob in self.__obstacles:
            if 'town01' in self.__map.name.lower():
                if ob.location.z - ob.extent.z < 10.0:
                    ob.location.z = 0
                    ob.extent.z = 20.0
            else:
                ob.location.z = 0
                ob.extent.z = 20.0

    def tick(self) -> None:
        self.__world.tick()

    # def get_tm(self) -> carla.TrafficManager:
    #     return self.__tm

    def load_world(self, _map: str, force: bool=False) -> None:
        # Change map only if the current map is different with the given map
        if _map.lower() not in self.__map.name.lower() or force:
            get_logger().info(f"Change Map: {self.__map.name} -> {_map}")
            try:
                self.__world = self.__client.load_world(_map)
                self.__map = self.__world.get_map()
                self.__spawnable_points = self.__map.get_spawn_points()
                for tl in self.__world.get_actors().filter('*traffic_light*'):
                    tl.set_state(carla.TrafficLightState.Green)
                    tl.set_red_time(0.0)
                    tl.set_yellow_time(0.0)
                    tl.freeze(True)
                self.__obstacles = sum([self.__world.get_level_bbs(obs) for obs in CARLA_OBSTACLES], [])
                for ob in self.__obstacles:
                    if 'town01' in self.__map.name.lower():
                        if ob.location.z - ob.extent.z < 10.0:
                            ob.location.z = 0
                            ob.extent.z = 20.0
                    else:
                        ob.location.z = 0
                        ob.extent.z = 20.0
            except Exception as _:
                raise LoadWorldException(_map)

    def reload_world(self) -> None:
        self.__world = self.__client.reload_world(reset_settings=False)
        self.__map = self.__world.get_map()
        self.__spawnable_points = self.__map.get_spawn_points()
        for tl in self.__world.get_actors().filter('*traffic_light*'):
            tl.set_state(carla.TrafficLightState.Green)
            tl.set_red_time(0.0)
            tl.set_yellow_time(0.0)
            tl.freeze(True)
        self.__obstacles = sum([self.__world.get_level_bbs(obs) for obs in CARLA_OBSTACLES], [])
        for ob in self.__obstacles:
            if 'town01' in self.__map.name.lower():
                if ob.location.z - ob.extent.z < 10.0:
                    ob.location.z = 0
                    ob.extent.z = 20.0
            else:
                ob.location.z = 0
                ob.extent.z = 20.0

    def get_world(self) -> carla.World:
        return self.__world

    def get_map(self) -> carla.Map:
        return self.__map

    def get_the_nearest_wp(self,
                           point: Union[Dict[str, float], carla.Waypoint, carla.Transform, carla.Location],
                           target: str = 'vehicles') -> carla.Waypoint:
        lane_type = carla.LaneType.Driving if target in 'vehicles' else carla.LaneType.Sidewalk
        if isinstance(point, carla.Waypoint):
            loc = point.transform.location
        elif isinstance(point, carla.Transform):
            loc = point.location
        elif isinstance(point, dict):
            loc = carla.Location(
                x=point['x'],
                y=point['y'],
                z=point['z']
            )
        else:
            loc = point
        return self.__map.get_waypoint(
            loc, lane_type=lane_type
        )

    def get_next_wp(self,
                    point: Dict[str, float],
                    distance: float,
                    target: str = 'vehicles') -> carla.Waypoint:
        wp_next = self.get_the_nearest_wp(point, target).next(distance)
        if len(wp_next) <= 0:
            return self.get_the_nearest_wp(point, target)
        else:
            return self.get_the_nearest_wp(wp_next.pop(), target)

    def get_previous_wp(self,
                        point: carla.Waypoint,
                        distance: float,
                        target: str = 'vehicles') -> carla.Waypoint:
        wp_prev = self.get_the_nearest_wp(point, target).previous(distance)
        if len(wp_prev) <= 0:
            return self.get_the_nearest_wp(point, target)
        else:
            return self.get_the_nearest_wp(wp_prev.pop(), target)

    def try_spawn_actor(self, 
                        bp: str, 
                        point: Union[Dict[str, float], carla.Transform],
                        attach_to: carla.Actor = None) -> Union[None, carla.Actor]:
        if isinstance(point, dict):
            sp = get_carla_transform(point)
        else:
            sp = point
        self.__world.get_blueprint_library().filter(bp)
        bp_actor = self.__world.get_blueprint_library().find(bp)
        actor = self.__world.try_spawn_actor(bp_actor, sp, attach_to=attach_to)
        return actor

    def get_ego_apollo(self) -> carla.Actor:
        ego = None
        while ego is None:
            actor_list = self.__world.get_actors()
            for actor in actor_list:
                try:
                    if (
                        "hero" == actor.attributes["role_name"]
                        or "ego_vehicle" == actor.attributes["role_name"]
                    ):
                        ego = actor
                        break
                except:
                    pass
        return ego

    def set_ego_start_apollo(self, 
                             start: Dict[str, float]) -> None:
        # Pick up the ego vehicle
        ego = self.get_ego_apollo()
        # Set the transform of the ego vehicle
        transform = get_carla_transform(start)
        ego.set_transform(transform)

    def is_junction(self, point: Dict[str, float]) -> bool:
        wp_nearest = self.get_the_nearest_wp(point)
        return wp_nearest.is_junction

    def is_corner(self, point: Dict[str, float]) -> bool:
        wps = []
        # Get the nearest waypoint
        wp = self.get_the_nearest_wp(point)
        wps.append(wp)
        while wp.next(2) != wps[-1] and len(wps) < 5:
            wp = wp.next(1)[0]
            wps.append(wp)
        while wp.previous(2) != wps[0] and len(wps) < 5:
            wp = wp.previous(1)[0]
            wps = [wp] + wps
        assert len(wps) == 5
        start_end_dot_prod = wps[0].transform.get_forward_vector().dot(
            wps[CORNERING_INTERVAL-1].transform.get_forward_vector())
        return start_end_dot_prod < 0.9967  # CORNERING_THERESHOLD

    def get_current_lane_id(self,
                            point: Dict[str, float]) -> int:
        # lane id here means the id of the lane where the ego vehicle is from the leftmost lane
        wp = self.get_the_nearest_wp(point)
        id_lane = 0
        # Count the number of lanes to the leftmost lane
        while wp.lane_change in [carla.LaneChange.Left, carla.LaneChange.Both]:
            if wp.lane_type == carla.LaneType.Bidirectional:
                break
            id_lane += 1
            wp_l = wp.get_left_lane()
            if wp_l is None:
                break
            wp = wp_l
        return id_lane + 1

    def plan_trajectory(self,
                        mission: Dict[str, Dict[str, float]],
                        target: str = 'vehicle') -> List[Tuple[carla.Transform, RoadOption]]:
        if target.lower() == 'vehicle':
            sampling_resolution = 0.5 #4.5
        else:
            sampling_resolution = 4.5 #2.0   # For pedestrians
        start = get_carla_location(mission['start'])
        dest = get_carla_location(mission['dest'])
        gp = GlobalRoutePlanner(self.get_map(), sampling_resolution)
        _trace = gp.trace_route(start, dest)
        if target.lower() != 'vehicle':  # For pedestrians
            _trace_tmp = [self.__map.get_waypoint(
                wp.transform.location,
                lane_type=carla.LaneType.Sidewalk
            ) for wp, _ in _trace]
            if len(_trace_tmp) < 10:
                # Pedestrian route can be not found in Town04
                _trace_tmp = [self.__map.get_waypoint(
                    wp.transform.location,
                    # lane_type=carla.LaneType.Sidewalk
                ) for wp, _ in _trace]
            _trace = []
            # Interpolate the waypoints
            for i in range(len(_trace_tmp) - 1):
                wp1 = _trace_tmp[i]
                wp2 = _trace_tmp[i+1]
                dist = wp1.transform.location.distance(wp2.transform.location)
                if dist > 1.0:
                    _xs = np.linspace(wp1.transform.location.x, wp2.transform.location.x, int(dist)+1)
                    _ys = np.linspace(wp1.transform.location.y, wp2.transform.location.y, int(dist)+1)
                    _zs = np.linspace(wp1.transform.location.z, wp2.transform.location.z, int(dist)+1)
                    _p1 = (wp1.transform.rotation.pitch - 180) % 360 - 180
                    _p2 = (wp2.transform.rotation.pitch - 180) % 360 - 180
                    _pitches = np.linspace(_p1, _p2, int(dist)+1)
                    _yaw1 = wp1.transform.rotation.yaw % 360
                    _yaw2 = wp2.transform.rotation.yaw % 360
                    if abs(_yaw1 - _yaw2) > 180:
                        if _yaw1 > _yaw2:
                            _yaw1 -= 360
                        else:
                            _yaw2 -= 360
                    _yaws = np.linspace(_yaw1, _yaw2, int(dist)+1)
                    for _x, _y, _z, _pitch, _yaw in zip(_xs, _ys, _zs, _pitches, _yaws):
                        _trace.append(carla.Transform(
                            location=carla.Location(x=_x, y=_y, z=_z),
                            rotation=carla.Rotation(roll=0.0, pitch=_pitch, yaw=_yaw)
                        ))
        else:
            _trace = [wp.transform for wp, _ in _trace]
        # For the case that the trace overflows the destination
        trace = []
        for _t in _trace:
            trace.append(_t)
            dist_xy = ((_t.location.x - dest.x) ** 2 + \
                (_t.location.y - dest.y) ** 2) ** 0.5
            if dist_xy < sampling_resolution:
                break
        if len(trace) > 1:
            wps = []
            for i in range(len(trace) - 1):
                if trace[i].location != trace[i+1].location:
                    wps.append(trace[i])
            if wps[-1].location != trace[-1].location:
                wps.append(trace[-1])
        else:
            wps = trace
        return wps

    def get_spawnable_points(self,
                             _map: str,
                             near_to: Optional[carla.Transform] = None,
                             near_dist: Optional[float] = None,
                             far_from: Optional[carla.Transform] = None,
                             far_dist: Optional[float] = None) -> List[carla.Transform]:
        # If the loaded map is different from the provided map, load the given map
        if not self.__map.name.lower().endswith(_map.lower()):
            self.load_world(_map.lower())
        spawnable_points = self.__spawnable_points
        # Points near to the given point
        if near_to is not None and near_dist is not None:
            spawnable_points = [p for p in self.__spawnable_points if p.location.distance(
                near_to.location) < near_dist]
            if len(spawnable_points) <= 0:
                spawnable_points = self.__spawnable_points
        # Points far from the given point
        if far_from is not None and far_dist is not None:
            spawnable_points = [p for p in spawnable_points if p.location.distance(
                far_from.location) > far_dist]
            if len(spawnable_points) <= 0:
                spawnable_points = self.__spawnable_points
        return spawnable_points
    
    def get_spawnable_points_by_id(self,
                                   _map: str,
                                   _id: int) -> carla.Transform:
        return self.__spawnable_points[_id]

    def check_trajectory_overlap(self,
                                 wps_to_zone: Union[List[carla.Waypoint], List[carla.BoundingBox]],
                                 wps: List[carla.Waypoint]) -> bool:
        if len(wps_to_zone) <= 0 or len(wps) <= 0:
            return False
        if isinstance(wps_to_zone[0], carla.Waypoint):
            wps_to_zone = [wp.transform for wp in wps_to_zone]
        if isinstance(wps[0], carla.Waypoint):
            wps = [wp.transform for wp in wps]
        if isinstance(wps_to_zone[0], carla.BoundingBox):
            zones = set(wps_to_zone)
        else:
            zones = {carla.BoundingBox(
                wp.location,
                carla.Vector3D(x=1.0, y=1.0, z=3.0),
            ) for wp in wps_to_zone}
        for wp in wps:
            for zone in zones:
                if distance_2d(wp.location, zone.location) > max(zone.extent.x, zone.extent.y) + 1.0:
                    continue
                if zone.contains(wp.location, carla.Transform()):
                    return True
        return False

    # Get a mission of NPC, which is interactable(=trajectory overlaps) with the ego vehicle
    def get_interactable_mission(self,
                                 _map: str,
                                 mission: Dict[str, Dict[str, float]],
                                 type_motion: str,
                                 target: str,
                                 bp: str,
                                 traj_exp: List[carla.Transform],
                                 near_dist: Optional[float] = None,
                                 far_dist: Optional[float] = None) -> Tuple[carla.Transform, carla.Transform]:
        start_ego = get_carla_transform(mission['start'])
        sps_start = self.get_spawnable_points(_map, start_ego, near_dist, start_ego, far_dist)
        if type_motion == STATIC:
            # Points only near to the ego vehicle's trajectory
            sps = []
            if target == 'puddle':
                for p in sps_start:
                    _bb_p = carla.BoundingBox(
                        carla.Location(
                            x=p.location.x,
                            y=p.location.y,
                            z=p.location.z
                        ),
                        carla.Vector3D(
                            x=4.0, 
                            y=4.0, 
                            z=10.0
                        )
                    )
                    if self.check_trajectory_overlap([_bb_p], traj_exp):
                        sps.append(p)
            else:   # Pedestrians
                for p in sps_start:
                    if self.check_trajectory_overlap([p], traj_exp):
                        sps.append(p)
            if len(sps) <= 0:
                return None, None
            # start_i = random.sample(sps_start, k=1).pop()
            # Pick up a random point
            start_i = random.sample(sps, k=1).pop()
            return start_i, None
        elif type_motion == DYNAMIC:
            dest_ego = get_carla_transform(mission['dest'])
            sps_dest = self.get_spawnable_points(_map, dest_ego, near_dist, dest_ego, far_dist)
            trial = 0
            _th = 500.0 if target == 'vehicle' else 200.0
            while trial < 30:
                start_i = random.sample(sps_start, k=1).pop()
                dest_i = random.sample(sps_dest, k=1).pop()
                # Check if the start and destination points are different
                if start_i.location == dest_i.location:
                    continue
                if target == 'vehicle':
                    __wp = self.get_the_nearest_wp(start_i)
                    if __wp.is_junction:
                        continue
                else:   # Pedestrian
                    if _map != 'Town04':
                        start_i = self.get_the_nearest_wp(start_i, target=target).transform
                        dest_i = self.get_the_nearest_wp(dest_i, target=target).transform
                    else:
                        # For Town04, the pedestrian's start and destination points are hard to be on the sidewalk
                        target = 'vehicle'
                _wps = self.plan_trajectory(
                    {'start': transform_to_dict(start_i),
                     'dest': transform_to_dict(dest_i)},
                    target=target
                )
                wps = []
                # Trim the length of the trajectory
                dist = 0
                for i in range(len(_wps) - 1):
                    dist += _wps[i].location.distance(_wps[i+1].location)
                    wps.append(_wps[i])
                    if dist > _th:
                        break
                # Check if the trajectory overlaps with the ego vehicle's trajectory
                if self.check_trajectory_overlap(wps, traj_exp):
                    return start_i, dest_i
                trial += 1
            # if trial >= 1:
            return None, None
        elif type_motion == LINEAR:
            trial = 0
            _bb_x = BB[bp]['extent']['x']
            _bb_y = BB[bp]['extent']['y']
            for _ob in self.__obstacles:
                if _ob.extent.x < _bb_x:
                    _ob.extent.x = _bb_x
                if _ob.extent.y < _bb_y:
                    _ob.extent.y = _bb_y
            # for wp in traj_exp:
            #     self.mark_point(wp, size=(0.2, 0.2), color='green')
            while trial < 3:
                _sps_start = set()
                for _sp in self.__spawnable_points:
                    for i, _wp in enumerate(traj_exp):
                        if 3.0 < _wp.location.distance(_sp.location) < 20.0:
                            _sps_start.add(carla.Transform(
                                location=carla.Location(
                                    x=_sp.location.x,
                                    y=_sp.location.y,
                                    z=_sp.location.z
                                ),
                                rotation=carla.Rotation(
                                    roll=_sp.rotation.roll,
                                    pitch=_sp.rotation.pitch,
                                    yaw=_sp.rotation.yaw
                                )
                            ))
                            break
                sps_start = list(_sps_start)
                # for i in range(len(sps_start)):
                #     self.mark_point(sps_start[i], size=(0.2, 0.2), color='blue')
                start = random.sample(sps_start, k=1).pop()
                if start is None:
                    return None, None
                # start.rotation.yaw = random.randint(0, 360)
                if random.random() < 0.5:
                    start.rotation.yaw += random.randint(-20, 20)
                else:
                    start.rotation.yaw += random.randint(-20, 20) + 180
                start.rotation.yaw = start.rotation.yaw % 360
                dist = 90 if target == 'vehicle' else 50
                loc_dest = start.location + start.get_forward_vector() * dist
                dest = carla.Transform(
                    location=carla.Location(
                        x=loc_dest.x,
                        y=loc_dest.y,
                        z=loc_dest.z
                    ),
                    rotation=carla.Rotation(
                        roll=start.rotation.roll,
                        pitch=start.rotation.pitch,
                        yaw=start.rotation.yaw
                    )
                )
                # Get expected trajectory
                dist_delta = 1.0
                cnt = 0
                wps = []
                wp_curr = get_carla_transform(transform_to_dict(start))
                while distance_2d(wp_curr.location, dest.location) > 3.0:
                    wp_curr = get_carla_transform(transform_to_dict(start))
                    v_g = wp_curr.get_forward_vector()
                    wp_curr.location += v_g * (dist_delta * cnt)
                    wps.append(wp_curr)
                    cnt += 1
                # Check if the trajectory overlaps with the ego vehicle's trajectory
                # # # Also, check if the trajectory does not overlap with obstacles
                # import time
                # for ob in self.__obstacles:
                #     # Draw box for debug
                #     self.__world.debug.draw_box(
                #         box=ob,
                #         rotation=carla.Rotation(),
                #         thickness=0.1,
                #         color=carla.Color(255, 0, 0),   # 빨강
                #         life_time=0.0                  # 10초 동안 표시
                #     )
                # print(len(self.__obstacles))
                # time.sleep(10000)
                # for wp in wps:
                #     self.mark_point(wp, size=(0.2, 0.2), color='red')
                # print(not self.check_trajectory_overlap(self.__obstacles, wps))
                # time.sleep(3)
                cond1 = self.check_trajectory_overlap(wps, traj_exp)
                cond2 = self.check_trajectory_overlap(self.__obstacles, wps)
                # print(f"Overlap with ego: {cond1}, No obs: {cond2}")
                if cond1 and not cond2:
                    # for wp in wps:
                    #     self.mark_point(wp, size=(0.2, 0.2), color='red')
                    return start, dest
                trial += 1
            # if trial >= 30:
            return None, None
        else:
            return None, None
        

    def translate_point(self,
                        sp: carla.Transform,
                        dist: float,
                        to: str = 'right') -> carla.Transform:
        if to == 'right':
            v_dir = sp.get_right_vector().make_unit_vector()
        elif to == 'left':
            v_dir = sp.get_right_vector().make_unit_vector() * -1
        elif to == 'forward':
            v_dir = sp.get_forward_vector().make_unit_vector()
        else:   # to == 'backward'
            v_dir = sp.get_forward_vector().make_unit_vector() * -1
        v = v_dir * dist
        return carla.Transform(
            location=carla.Location(
                x=sp.location.x + v.x,
                y=sp.location.y + v.y,
                z=sp.location.z
            ),
            rotation=sp.rotation
        )


    def update_spawnable_points(self, _map: str, points: List[carla.Transform]) -> None:
        sps_new = []
        points_carla = set(self.get_the_nearest_wp(
            p).transform for p in points)
        if _map.lower() not in self.__map.name.lower():
            self.load_world(_map.lower())
        for sp in self.__spawnable_points:
            for _p in points_carla:
                if sp.location.distance(_p.location) < 5:
                    break
            else:
                sps_new.append(sp)
        if len(sps_new) <= 0:
            get_logger().info("Every spawnable points are occupied. Resetting the spawnable points.")
            sps_new = self.__map.get_spawn_points()
        get_logger().info(
            f"# ran points: {len(points)}, # spawnable points: {len(self.__spawnable_points)} → {len(sps_new)}")
        self.__spawnable_points = sps_new

    def mark_point(self,
                   point: carla.Transform,
                   size: Tuple[float, float] = (0.1, 0.1),
                   color: str = 'blue') -> None:
        if color.lower() == 'blue':
            color = carla.Color(r=0, g=0, b=1, a=1)
        elif color.lower() == 'red':
            color = carla.Color(r=1, g=0, b=0, a=1)
        elif color.lower() == 'green':
            color = carla.Color(r=0, g=1, b=0, a=1)
        else:
            color = carla.Color(r=0, g=0, b=1, a=1)
        self.__world.debug.draw_box(
            box=carla.BoundingBox(
                point.location if isinstance(
                    point, carla.Transform) else point,
                carla.Vector3D(size[0], size[1], 1.0)
            ),
            rotation=point.rotation if isinstance(
                point, carla.Transform) else carla.Rotation(),
            life_time=0,
            thickness=0.5,
            color=color
        )


def distance_2d(loc1: carla.Location,
                loc2: carla.Location) -> float:
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)

cc = __CarlaClient(host=Config.HOST_CARLA, port=Config.PORT_CARLA)
