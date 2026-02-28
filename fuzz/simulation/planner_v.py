import time
from typing import Any, Dict, List, Union

from fuzz.carla import cc
from fuzz.carla.utils import get_carla_location, get_carla_transform, get_carla_rotation
from fuzz.commons.constants import *
from fuzz.commons.exceptions import DDSimulateFail, ExtractionException
from fuzz.commons.utils import get_logger
from fuzz.model import Model, Extractor
from fuzz.simulation.utils import distance_2d

import carla


class WayPointV:

    def __init__(self,
                 transform: carla.Transform,
                 speed: float,
                 time: float) -> None:
        self.transform: carla.Transform = transform
        self.speed: float = speed
        self.time: float = time

    def __str__(self) -> str:
        return f'[{self.time:.2f}] {self.transform}, speed: {self.speed:.2f}km/h'

    def __repr__(self) -> str:
        return str(self)


class PlannerV:

    def __init__(self,
                 dt: float = 1/FPS) -> None:
        self.dt: float = dt
        self.__idx_route_curr: int = 0
        self.__idx_min_prev: int = 0
        self.__idx_min_next: int = 1

    def plan_trajectory_ego(self,
                            mission: Dict[str, Any],
                            model: Model,
                            start_delay: float,
                            traj_exp: List[carla.Transform] = [],
                            path_run: Union[None, Path]=None,
                            idx_ignore: int=-1) -> List[WayPointV]:
        # If the model is not trained, no prediction and run the simulation.
        if not model.trained:
            print("Model not trained")
            raise DDSimulateFail
        self.__idx_route_curr = 0
        self.__idx_min_prev: int = 0
        self.__idx_min_next: int = 1
        # For Ego and Dynamic NPC
        wps = traj_exp
        ext = Extractor(mission=mission, traj_exp=traj_exp)
        # Run
        wps_v = []
        # Pad first (start_delay) seconds with zero speed
        for _ in range(int(start_delay * FPS)):
            wps_v.append(WayPointV(
                transform=wps[0],
                speed=0.0,
                time=len(wps_v) * self.dt
            ))
        wp_curr = wps[0]
        speed = 0.0
        speeds = [speed]
        dest = get_carla_location(mission['dest'])
        if wps[-1].location.distance(dest) > 20.0:
            # The planned trajectory could be shorter than the real trajectory
            dest = wps[-1].location
        # print(f"dest: {dest}")
        st = time.time()
        try:
            while distance_2d(dest, wp_curr.location) > 1.0: #len(wps_v) < duration_max:
                if time.time() - st > 5:
                    get_logger().info(f"DDSimulate Fail({'ego' if 'type_motion' not in mission else 'npc'}, {path_run if path_run is not None else ''})")
                    return wps_v
                speed_curr = speeds[-1]
                # Extract speed feature
                try:
                    f = ext.get_speed_feature(wp_curr, speed_curr)
                except ExtractionException:
                    return wps_v
                # Speed prediction
                if idx_ignore >= 0:
                    _features = list(f.values())[:idx_ignore] + list(f.values())[idx_ignore+1:]
                    speed = model(_features)
                else:
                    speed = model(list(f.values()))
                if speed <= 0.0:
                    speed = 0.0  # 0.001
                elif speed > 30.0:
                    speed = 30.0
                if abs(speed_curr - speed) > 30.0 / 3.6 * (1/FPS):
                    if speed_curr < speed:
                        speed = speed_curr + 30.0 / 3.6 * (1/FPS)
                    else:
                        speed = speed_curr - 30.0 / 3.6 * (1/FPS)
                # Update
                # Average speed, km/h -> m/s
                dist_to_drive = (speed + speeds[-1]) / 2 / 3.6 * self.dt
                if dist_to_drive > 0.0:
                    try:
                        wp_next = self.get_next_wp(wp_curr, dist_to_drive, wps)
                    except IndexError as _:
                        wp_next = wp_curr
                else:
                    # Speed is 0.0
                    wp_next = wp_curr
                wps_v.append(WayPointV(
                    transform=wp_next, #wp_curr,
                    speed=speed,
                    time=len(wps_v) * self.dt,
                ))
                wp_curr = wp_next
                speeds.append(speed)
        finally:
            if path_run is not None:
                __times = []
                __speeds = []
                for wp in wps_v:
                    __times.append(wp.time)
                    __speeds.append(wp.speed)
        return wps_v

    def plan_trajectory(self,
                        npc: Dict[str, Any],
                        model: Model, #Union[None, Model],
                        type_motion: str = DYNAMIC,
                        speed_linear: float = -1.0,
                        duration_max: int = 300 * FPS,
                        path_run: Union[None, Path]=None,
                        idx_ignore: int=-1) -> List[WayPointV]:
        # DdNpc
        if type_motion == STATIC:   # immobile NPC
            # wp = cc.get_the_nearest_wp(npc['start'])
            _t = get_carla_transform(npc['start'])
            # return [WayPointV(wp, 0.0, 0.0, transform=_t)]
            return [WayPointV(_t, 0.0, 0.0) for _ in range(duration_max)]
        elif type_motion == LINEAR:  # Linear NPC
            wps_v = []
            # wp_curr = cc.get_the_nearest_wp(npc['start'])
            wp_curr = get_carla_transform(npc['start'])
            dest = get_carla_transform(npc['dest'])
            dist_delta = speed_linear * self.dt
            for _ in range(1 * FPS):
                wps_v.append(WayPointV(
                    transform=wp_curr,
                    # speed=speed,
                    speed=speed_linear * 3.6,
                    time=len(wps_v) * self.dt,
                ))
            cnt = 0
            while distance_2d(wp_curr.location, dest.location) > 3.0:
                wp_curr = get_carla_transform(npc['start'])
                v_g = wp_curr.get_forward_vector()
                wp_curr.location += v_g * (dist_delta * cnt)
                wps_v.append(WayPointV(
                    transform=wp_curr,
                    speed=speed_linear * 3.6,
                    time=len(wps_v) * self.dt,
                ))
                cnt += 1
            wps_v.append(WayPointV(
                transform=dest,
                speed=0.0,
                time=len(wps_v) * self.dt,
            ))
            return wps_v
        else:   # Dynamic NPC
            # For dynamic pedestrian
            if 'bp' in npc and 'walker' in npc['bp']:
                _wps = cc.plan_trajectory(npc, target='pedestrian')
                speed = float(npc['speed'])
                t_curr = get_carla_transform({
                    'x': _wps[0].location.x,
                    'y': _wps[0].location.y,
                    'z': _wps[0].location.z,
                    'roll': _wps[0].rotation.roll,
                    'pitch': _wps[0].rotation.pitch,
                    'yaw': _wps[0].rotation.yaw
                })
                wp_curr = _wps[0]
                wps_v = []
                for _ in range(1 * FPS):
                    wps_v.append(WayPointV(
                        transform=t_curr,
                        speed=speed * 3.6,
                        time=len(wps_v) * self.dt
                    ))
                for i in range(len(_wps) - 1):
                    # Sometimes same planned waypoints are generated
                    if _wps[i].location == _wps[i+1].location:
                        continue
                    dir_diff = (_wps[i+1].location -
                                _wps[i].location).make_unit_vector()
                    while distance_2d(t_curr.location, _wps[i+1].location) >= 0.1:
                        delta_dist = speed * self.dt
                        t_curr.location += dir_diff * delta_dist
                        t_curr = carla.Transform(
                            location=get_carla_location({
                                'x': t_curr.location.x,
                                'y': t_curr.location.y,
                                'z': t_curr.location.z
                            }),
                            rotation=get_carla_rotation({
                                'roll': t_curr.rotation.roll,
                                'pitch': t_curr.rotation.pitch,
                                'yaw': t_curr.rotation.yaw
                            })
                        )
                        wps_v.append(WayPointV(
                            transform=t_curr,
                            speed=speed * 3.6,
                            time=i * self.dt
                        ))
                    t_curr.location.x = _wps[i+1].location.x
                    t_curr.location.y = _wps[i+1].location.y
                    t_curr.location.z = _wps[i+1].location.z
                    t_curr.rotation.roll = _wps[i+1].rotation.roll
                    t_curr.rotation.pitch = _wps[i+1].rotation.pitch
                    t_curr.rotation.yaw = _wps[i+1].rotation.yaw
                wps_v.append(WayPointV(
                    transform=_wps[-1],
                    speed=0.0,
                    time=len(wps_v) * self.dt
                ))
                return wps_v
            else:
                assert model is not None
                # For dynamic NPC Vehicle
                _wps_v = cc.plan_trajectory(npc)
                wps_v = self.plan_trajectory_ego(npc, model, start_delay=1.5, traj_exp=_wps_v, path_run=path_run, idx_ignore=idx_ignore)
                wps_v.append(WayPointV(
                    transform=_wps_v[-1],
                    speed=0.0,
                    time=len(wps_v) * self.dt
                ))
                if not model.trained:
                    return wps_v
            return wps_v


    def get_next_wp(self,
                    wp: carla.Transform,
                    dist: float,
                    route: List[carla.Transform]) -> carla.Transform:
        if dist <= 0.0:
            return wp
        _p_prev = route[self.__idx_min_prev]
        _p_next = route[self.__idx_min_next]
        # Get the direction
        try:
            dir = (_p_next.location - _p_prev.location).make_unit_vector()
        except RuntimeError:
            print("ERROR")
            dir = _p_next.location.make_unit_vector()
        # Get the next point
        wp_new = carla.Transform()
        wp_new.location = wp.location + dist * dir
        # Update the nearest points' index
        _min = max(0, self.__idx_route_curr-5)
        _max = min(len(route), self.__idx_route_curr+5)
        for i in range(self.__idx_route_curr, _max-1):
            try:
                _dir1 = (route[i].location -
                         wp_new.location).make_unit_vector()
            except RuntimeError:
                _dir1 = route[i].location.make_unit_vector()
            try:
                _dir2 = (route[i+1].location -
                         wp_new.location).make_unit_vector()
            except RuntimeError:
                _dir2 = route[i+1].location.make_unit_vector()
            # if the wp is between the two points
            if _dir1.dot(_dir2) < 0.5: #0:
                idx1, idx2 = i, i+1
                break
        try:
            self.__idx_route_curr = idx1
        except UnboundLocalError:
            idx1 = self.__idx_min_prev
            idx2 = self.__idx_min_next
        self.__idx_min_prev = idx1
        self.__idx_min_next = idx2
        p_prev = route[idx1]
        p_next = route[idx2]
        d1 = distance_2d(p_prev.location, wp_new.location)
        d2 = distance_2d(p_next.location, wp_new.location)
        # Correcting the next point
        wp_new.location.x = (d2 * p_prev.location.x +
                             d1 * p_next.location.x) / (d1 + d2)
        wp_new.location.y = (d2 * p_prev.location.y +
                             d1 * p_next.location.y) / (d1 + d2)
        wp_new.location.z = (d2 * p_prev.location.z +
                             d1 * p_next.location.z) / (d1 + d2)
        wp_new.rotation.roll = (d2 * (p_prev.rotation.roll %
                                360) + d1 * (p_next.rotation.roll % 360)) / (d1 + d2)
        _pitch_prev = (p_prev.rotation.pitch - 180) % 360 - 180
        _pitch_next = (p_next.rotation.pitch - 180) % 360 - 180
        wp_new.rotation.pitch = (
            d2 * _pitch_prev + d1 * _pitch_next) / (d1 + d2)
        _yaw_prev = p_prev.rotation.yaw % 360
        _yaw_next = p_next.rotation.yaw % 360
        if abs(_yaw_prev - _yaw_next) > 180:
            if _yaw_prev > _yaw_next:
                _yaw_prev -= 360
            else:
                _yaw_next -= 360
        wp_new.rotation.yaw = (d2 * _yaw_prev + d1 * _yaw_next) / (d1 + d2)
        return wp_new
