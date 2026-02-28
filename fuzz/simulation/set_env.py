from copy import deepcopy
import os
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Tuple, Union

from fuzz.data import Scenario
from fuzz.simulation.utils import CollisionSensor, LaneInvasionSensor, State, GlobalRoutePlanner, DummyWorld
from fuzz.carla.utils import get_carla_transform

import carla

from agents.navigation.behavior_agent import BehaviorAgent

FPS = 20


class Environment:

    def __init__(self, 
                 scenario: Scenario, 
                 ego_vehicle: carla.Actor, 
                 world: carla.World,
                 tm: carla.TrafficManager) -> None:
        if not Path('shared/logs/other').exists():
            self.scenario: Scenario = scenario
            self.path_log: Path = deepcopy(self.scenario.path_scenario)
            self.world: carla.World = world #CarlaDataProvider.get_world()
            self.map: carla.Map = self.world.get_map()
            self.vehicles: List[Tuple[int, carla.Actor, Any, Any, str]] = []
            self.pedestrians: List[Tuple[int, carla.Actor, Any, Any, str]] = []
            self.puddles: List[carla.BoundingBox] = []
            # For simulation
            self.ego_vehicle: carla.Actor = ego_vehicle
            self.timer_stalling: int = 0
            self.__init: bool = False
            self.state: State = State(self.scenario.path_scenario)
            self.camera_logging: List[carla.Actor] = []
            self.collision_sensor: CollisionSensor = CollisionSensor(
                self.ego_vehicle)
            self.lane_invasion_sensor: LaneInvasionSensor = LaneInvasionSensor(
                ego_vehicle)
            self.speed_max: float = 33.0
            self.speed_max_prev: float = self.speed_max
            self.buffer_speeding: int = 3 * FPS  # 3 seconds
            self.no_speeding: bool = False
            self.yaw_prev: float = ego_vehicle.get_transform().rotation.yaw
            self.dist_min: float = 10000.0
            self.time_running: float = 0.0
            self.time_start: float = 0.0
            self.idx_ped: Dict[int, int] = {}
        else:
            self.__init: bool = False
            self.ego_vehicle: carla.Actor = ego_vehicle
            self.world: carla.World = world
            self.camera_logging = []

    def spawn_vehicles(self, vehicles: List[Dict[str, Any]]) -> List[Tuple[int, carla.Actor, Any, Any, str]]:
        spawned_vehicles = []
        gp = GlobalRoutePlanner(self.map, 0.5)
        for i, v in enumerate(vehicles):
            bp = self.world.get_blueprint_library().find(v['bp'])
            sp_t = get_carla_transform(v['start'])
            vehicle = self.try_spawn_actor(bp, sp_t)
            if vehicle is not None:
                vehicle.set_simulate_physics(True)
                if 'type_motion' in v:
                    if v['type_motion'] == 'dynamic':
                        start = get_carla_transform(v['start'])
                        dest = get_carla_transform(v['dest'])
                        trace = gp.trace_route(start.location, dest.location)
                        agent = BehaviorAgent(
                            vehicle,
                            behavior='cautious',
                            opt_dict={
                                'ignore_traffic_lights': True,
                                'ignore_stop_signs': True,
                                # 'sampling_resolution': 6.0
                            }
                        )
                        agent.set_destination(
                            start_location=start.location,
                            end_location=dest.location,
                        )
                        agent.get_local_planner().set_global_plan(trace)
                        dummy_world = DummyWorld(self.world, vehicle)
                        spawned_vehicles.append(
                            (v['id'], vehicle, agent, dest.location, v['type_motion']))
                    elif v['type_motion'] == 'linear':
                        start = get_carla_transform(v['start'])
                        speed_v = v['speed']
                        dir_v = start.get_forward_vector().make_unit_vector()
                        vector = speed_v * dir_v
                        vehicle.set_target_velocity(vector)
                        spawned_vehicles.append(
                            (v['id'], vehicle, [], vector, v['type_motion']))
                    elif v['type_motion'] == 'static':
                        spawned_vehicles.append(
                            (v['id'], vehicle, [], '', v['type_motion']))
                elif 'nav_type' in v and v['nav_type'] == 0:  # Linear
                    forward_vec = sp_t.rotation.get_forward_vector()
                    vehicle.set_target_velocity(forward_vec * v["speed"])
                    spawned_vehicles.append(
                        (v['id'], vehicle, [], '', v['nav_type']))
                elif 'nav_type' in v and v['nav_type'] == 1:  # Autopilot
                    vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.ignore_lights_percentage(vehicle, 100)
                    self.tm.ignore_signs_percentage(vehicle, 100)
                    spawned_vehicles.append(
                        (v['id'], vehicle, [], '', v['nav_type']))
                elif 'nav_type' in v and v['nav_type'] == 2:  # Immobile
                    spawned_vehicles.append(
                        (v['id'], vehicle, [], '', v['nav_type']))
                else:
                    spawned_vehicles.append(
                        (v['id'], vehicle, [], '', v['type_motion']))
        return spawned_vehicles

    def spawn_pedestrians(self, pedestrians: List[Dict[str, Any]]) -> List[Tuple[int, carla.Actor, Any, Any, str]]:
        spawned_pedestrians = []
        for i, p in enumerate(pedestrians):
            bp = self.world.get_blueprint_library().find(p['bp'])
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'true')
            sp_t = get_carla_transform(p['start'])
            pedestrian = self.try_spawn_actor(bp, sp_t)
            if pedestrian is not None:
                if 'type_motion' in p:
                    if p['type_motion'] == 'dynamic':
                        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
                        controller = self.world.try_spawn_actor(
                            walker_controller_bp, carla.Transform(), pedestrian
                        )
                        if controller is not None:
                            gp = GlobalRoutePlanner(self.map, 2.0)
                            dest = get_carla_transform(p['dest'])
                            trace = gp.trace_route(
                                sp_t.location, dest.location)
                            wps = [self.map.get_waypoint(wp.transform.location, lane_type=carla.LaneType.Sidewalk).transform
                                   for wp, _ in trace]
                            _speed = float(p['speed'])
                            spawned_pedestrians.append(
                                (p['id'], pedestrian, controller,
                                 (wps, _speed), p['type_motion'])
                            )
                    elif p['type_motion'] == 'linear':
                        forward_vec = sp_t.rotation.get_forward_vector()
                        controller_walker = carla.WalkerControl()
                        controller_walker.direction = forward_vec
                        controller_walker.speed = p["speed"]
                        pedestrian.apply_control(controller_walker)
                        spawned_pedestrians.append(
                            (p['id'], pedestrian, None, (None, None), p['type_motion']))
                    elif p['type_motion'] == 'static':
                        spawned_pedestrians.append(
                            (p['id'], pedestrian, None, (None, None), p['type_motion']))
                elif 'nav_type' in p and p['nav_type'] == 0:  # Linear
                    forward_vec = sp_t.rotation.get_forward_vector()
                    controller_walker = carla.WalkerControl()
                    controller_walker.direction = forward_vec
                    controller_walker.speed = p["speed"]
                    pedestrian.apply_control(controller_walker)
                    spawned_pedestrians.append(
                        (p['id'], pedestrian, None, (None, None), p['nav_type']))
                elif 'nav_type' in p and p['nav_type'] == 1:  # Autopilot
                    walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
                    controller = self.world.try_spawn_actor(
                        walker_controller_bp, sp_t, pedestrian
                    )
                    # self.mark_point(get_carla_transform(p['start']), color='red', size=(2, 2))
                    if controller is not None:
                        dest = get_carla_transform(p['dest'])
                        speed_max = float(p['speed'])
                        spawned_pedestrians.append(
                            (p['id'], pedestrian, controller,
                             (dest.location, speed_max), p['nav_type'])
                        )
                    spawned_pedestrians.append(
                        (p['id'], pedestrian, None, (None, None), p['nav_type']))
                elif 'nav_type' in p and p['nav_type'] == 2:  # Immobile
                    spawned_pedestrians.append(
                        (p['id'], pedestrian, None, (None, None), p['nav_type']))
                else:
                    spawned_pedestrians.append(
                        (p['id'], pedestrian, None, (None, None), p['type_motion']))
        return spawned_pedestrians

    def spawn_puddles(self, puddles: List[Dict[str, float]]) -> List[carla.BoundingBox]:
        puddles_spawned = []
        bp = self.world.get_blueprint_library().find('static.trigger.friction')
        for puddle in puddles:
            bp.set_attribute('friction', str(puddle["level"]))
            bp.set_attribute('extent_x', str(puddle["size_x"]))
            bp.set_attribute('extent_y', str(puddle["size_y"]))
            bp.set_attribute('extent_z', str(puddle["size_z"]))
            sp = get_carla_transform(
                point={
                    'x': puddle['x'],
                    'y': puddle['y'],
                    'z': puddle['z'],
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': 0.0,
                }
            )
            size = carla.Location(
                x=puddle['size_x'],
                y=puddle['size_y'],
                z=puddle['size_z']
            )

            _puddle = self.try_spawn_actor(bp, sp)

            if _puddle is not None:
                puddles_spawned.append(_puddle)
                self.world.debug.draw_box(
                    box=carla.BoundingBox(
                        sp.location,
                        size * 1e-2
                    ),
                    rotation=sp.rotation,
                    life_time=0,
                    thickness=puddle["level"] * 1,  # the stronger the thicker
                    color=carla.Color(r=0, g=0, b=1, a=100)
                )
        return puddles_spawned

    def set_environment(self, _weather: Dict[str, float], _time: Dict[str, float]) -> None:
        weather = self.world.get_weather()
        weather.cloudiness = _weather['cloudiness']
        weather.precipitation = _weather['precipitation']
        weather.precipitation_deposits = _weather['precipitation_deposits']
        weather.wind_intensity = _weather['wind_intensity']
        weather.fog_density = _weather['fog_density']
        weather.wetness = _weather['wetness']
        weather.dust_storm = _weather['dust_storm']
        weather.sun_azimuth_angle = _time['sun_azimuth_angle']
        weather.sun_altitude_angle = _time['sun_altitude_angle']
        self.world.set_weather(weather)

    def try_spawn_actor(self,
                        bp: carla.ActorBlueprint,
                        sp: Union[carla.Transform, Dict[str, float]],
                        attach_to: carla.Actor = None) -> Optional[carla.Actor]:
        if not isinstance(sp, carla.Transform):
            sp = get_carla_transform(sp)
        return self.world.try_spawn_actor(bp, sp, attach_to=attach_to)

    def mark_point(self,
                   point: carla.Transform,
                   size: Tuple[float, float] = (0.1, 0.1),
                   color: str = 'blue') -> None:
        if color.lower() == 'blue':
            color = carla.Color(r=0, g=0, b=1, a=100)
        elif color.lower() == 'red':
            color = carla.Color(r=1, g=0, b=0, a=100)
        elif color.lower() == 'green':
            color = carla.Color(r=0, g=1, b=0, a=100)
        else:
            color = carla.Color(r=0, g=0, b=100, a=100)
        self.world.debug.draw_box(
            box=carla.BoundingBox(
                point.location,
                carla.Vector3D(size[0], size[1], 1.0)
            ),
            rotation=point.rotation,
            life_time=0,
            thickness=0.5,
            color=color
        )

    def __distance_2d(self, a: carla.Location, b: carla.Location) -> float:
        return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

    def attach_cameras_for_logging(self, log_dir) -> None:
        self.log_dir = log_dir
        self.path_images = self.log_dir.parent / 'images'
        if not self.path_images.is_dir():
            self.path_images.mkdir(exist_ok=True)
        self.__clean_image_dir()

        def __on_front_camera_capture(image) -> None:
            image.save_to_disk(f"{self.path_images}/front-{image.frame}.jpg")

        def __on_top_camera_capture(image) -> None:
            image.save_to_disk(f"{self.path_images}/top-{image.frame}.jpg")

        # Set RGB camera for logging
        bp_rgb = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp_rgb.set_attribute("image_size_x", "800")
        bp_rgb.set_attribute("image_size_y", "600")
        bp_rgb.set_attribute("fov", "90")
        bp_rgb.set_attribute('role_name', '__log_camera__')

        camera_front = self.world.spawn_actor(
            bp_rgb,
            carla.Transform(
                carla.Location(x=-6, z=5),
                carla.Rotation(pitch=-30)
            ),
            attach_to=self.ego_vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        camera_front.listen(lambda image: __on_front_camera_capture(image))
        self.camera_logging.append(camera_front)

        camera_top = self.world.spawn_actor(
            bp_rgb,
            carla.Transform(
                carla.Location(z=30.0),
                carla.Rotation(pitch=-90.0)
            ),
            attach_to=self.ego_vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        camera_top.listen(lambda image: __on_top_camera_capture(image))
        self.camera_logging.append(camera_top)

    def initialize(self) -> None:
        self.vehicles = self.spawn_vehicles(self.scenario.npcs['vehicles'])
        self.pedestrians = self.spawn_pedestrians(
            self.scenario.npcs['pedestrians'])
        self.puddles = self.spawn_puddles(self.scenario.puddles)
        self.set_environment(self.scenario.weather, self.scenario.time)
        dest = get_carla_transform(self.scenario.mission['dest'])
        self.attach_cameras_for_logging(self.path_log)
        self.world.tick()
        self.mark_point(dest, color='green', size=(0.15, 0.15))
        self.speed_max: float = self.ego_vehicle.get_speed_limit() * 1.1
        self.speed_max_prev: float = self.speed_max
        self.buffer_speeding: int = 3 * 20  # 3 seconds
        self.no_speeding: bool = False
        self.state.bounding_boxes['ego'] = {
            'extent': {
                'x': self.ego_vehicle.bounding_box.extent.x,
                'y': self.ego_vehicle.bounding_box.extent.y,
                'z': self.ego_vehicle.bounding_box.extent.z
            },
            'point': {
                'x': self.ego_vehicle.get_location().x,
                'y': self.ego_vehicle.get_location().y,
                'z': self.ego_vehicle.get_location().z
            }
        }
        for _, v, _, _, _ in self.vehicles:
            self.state.bounding_boxes[f'{v.type_id}'] = {
                'extent': {
                    'x': v.bounding_box.extent.x,
                    'y': v.bounding_box.extent.y,
                    'z': v.bounding_box.extent.z
                },
                'point': {
                    'x': v.get_location().x,
                    'y': v.get_location().y,
                    'z': v.get_location().z
                }
            }
        for id_p, p, controller, (trs_ped, speed_ped), type_motion in self.pedestrians:
            self.state.bounding_boxes[f'{p.type_id}'] = {
                'extent': {
                    'x': p.bounding_box.extent.x,
                    'y': p.bounding_box.extent.y,
                    'z': p.bounding_box.extent.z
                },
                'point': {
                    'x': p.get_location().x,
                    'y': p.get_location().y,
                    'z': p.get_location().z
                }
            }
            if type_motion == 'dynamic':
                self.idx_ped[id_p] = 1
                tr_start = trs_ped[0]
                tr_next = trs_ped[1]
                dir_diff = (tr_next.location -
                            tr_start.location).make_unit_vector()
                controller_walker = carla.WalkerControl()
                controller_walker.direction = dir_diff
                controller_walker.speed = speed_ped
                p.apply_control(controller_walker)
            p.set_collisions(True)
            p.set_simulate_physics(True)

    def run_step(self) -> bool:
        try:
            if Path('shared/logs/other').exists():
                if not Path('shared/logs/lock').exists():
                    if not self.__init:
                        self.__init = True
                        self.attach_cameras_for_logging(Path('shared/logs'))
                while Path('shared/logs/done').exists():
                    for camera in self.camera_logging:
                        try:
                            camera.destroy()
                        except RuntimeError as _:
                            continue
                    self.camera_logging = []
                    time.sleep(100000)
                return False
            speed = self.ego_vehicle.get_velocity().length() * 3.6
            p = self.ego_vehicle.get_transform()
            # Run and log NPC Vehicles' state
            states_npc_vehicles = []
            for id_v, vehicle, agent, vector, type_motion in self.vehicles:
                if vehicle is not None and vehicle.is_alive:
                    _id = id_v
                    _t = vehicle.get_transform()
                    _v = vehicle.get_velocity().length() * 3.6
                    ctrl_vehicle = vehicle.get_control()
                    wp_npc_vehicle = self.map.get_waypoint(
                        vehicle.get_location()
                    )
                    states_npc_vehicles.append({
                        "id": _id,
                        'type': vehicle.type_id,
                        'type_motion': type_motion,
                        'point': {
                            'x': _t.location.x,
                            'y': _t.location.y,
                            'z': _t.location.z,
                            'roll': _t.rotation.roll,
                            'pitch': _t.rotation.pitch,
                            'yaw': _t.rotation.yaw
                        },
                        'speed': _v,
                        'speed_limit': vehicle.get_speed_limit(),
                        'throttle': ctrl_vehicle.throttle,
                        'steer': ctrl_vehicle.steer,
                        'brake': ctrl_vehicle.brake,
                        'hand_brake': ctrl_vehicle.hand_brake,
                        'reverse': ctrl_vehicle.reverse,
                        'manual_gear_shift': ctrl_vehicle.manual_gear_shift,
                        'gear': ctrl_vehicle.gear,
                        'at_traffic_light': vehicle.is_at_traffic_light(),
                        'id_junction': wp_npc_vehicle.junction_id
                    })
                    if type_motion == 'dynamic':
                        ctrl = agent.run_step()
                        vehicle.apply_control(ctrl)
                        ctrl_p = vehicle.get_physics_control()
                        for _wheel in ctrl_p.wheels:
                            _wheel.tire_friction = 3.5
                        if agent.done():
                            vehicle.destroy()

            # State of pedestrians around the ego-vehicle
            states_npc_pedestrians = []
            for id_p, pedestrian, controller, (trs_ped, speed_ped), type_motion in self.pedestrians:
                if pedestrian is not None and pedestrian.is_alive:
                    _id = id_p
                    _t = pedestrian.get_transform()
                    _v = pedestrian.get_velocity().length() * 3.6
                    states_npc_pedestrians.append({
                        "id": _id,
                        'type': pedestrian.type_id,
                        'type_motion': type_motion,
                        'point': {
                            'x': _t.location.x,
                            'y': _t.location.y,
                            'z': _t.location.z,
                            'roll': _t.rotation.roll,
                            'pitch': _t.rotation.pitch,
                            'yaw': _t.rotation.yaw
                        },
                        'speed': _v
                    })
                    if type_motion == 'dynamic':
                        tr_next = trs_ped[self.idx_ped[id_p]]
                        if self.__distance_2d(tr_next.location, pedestrian.get_location()) < 1.5:
                            if self.idx_ped[id_p] == len(trs_ped) - 1:
                                # The end of the route
                                pedestrian.destroy()
                                # controller.destroy()
                            else:
                                # controller.go_to_location(loc_next)
                                self.idx_ped[id_p] += 1
                                tr_next_next = trs_ped[self.idx_ped[id_p]]
                                try:
                                    dir_diff = (tr_next_next.location -
                                                tr_next.location).make_unit_vector()
                                    controller_walker = carla.WalkerControl()
                                    controller_walker.direction = dir_diff
                                    controller_walker.speed = speed_ped
                                    pedestrian.apply_control(controller_walker)
                                except RuntimeError:
                                    os.system(f'echo "{tr_next_next.location}" >> autoware/log.txt')
                                    os.system(f'echo "{tr_next.location}" >> autoware/log.txt')
                                    os.system(f'echo "------" >> autoware/log.txt')
            ctrl = self.ego_vehicle.get_control()
            vel = self.ego_vehicle.get_velocity()
            ego_right_vec = self.ego_vehicle.get_transform().get_right_vector()
            lat_speed = abs(vel.x * ego_right_vec.x + vel.y * ego_right_vec.y)
            lat_speed *= 3.6  # m/s to km/h
            ego_fwd_vec = self.ego_vehicle.get_transform().get_forward_vector()
            lon_speed = abs(vel.x * ego_fwd_vec.x + vel.y * ego_fwd_vec.y)
            lon_speed *= 3.6
            max_steer_angle = 0
            for wheel in self.ego_vehicle.get_physics_control().wheels:
                if wheel.max_steer_angle > max_steer_angle:
                    max_steer_angle = wheel.max_steer_angle
            yaw_curr = self.ego_vehicle.get_transform().rotation.yaw
            yaw_diff = yaw_curr - self.yaw_curr
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
            elif yaw_diff < -180:
                yaw_diff = 360 + yaw_diff

            yaw_rate = yaw_diff * FPS
            self.yaw_curr = yaw_curr
            for _, __v, _, _, _ in self.vehicles:
                if __v is not None and __v.is_alive:
                    dist = p.location.distance(__v.get_location())
                    if dist < self.dist_min:
                        self.dist_min = dist
            for _, __p, _, _, _ in self.pedestrians:
                if __p is not None and __p.is_alive:
                    dist = p.location.distance(__p.get_location())
                    if dist < self.dist_min:
                        self.dist_min = dist
            # Update state
            self.state.update({
                'point': {
                    'x': p.location.x,
                    'y': p.location.y,
                    'z': p.location.z,
                    'roll': p.rotation.roll,
                    'pitch': p.rotation.pitch,
                    'yaw': p.rotation.yaw
                },
                'speed': speed,
                'speed_limit': self.ego_vehicle.get_speed_limit(),
                'throttle': ctrl.throttle,
                'steer': ctrl.steer,
                'brake': ctrl.brake,
                'hand_brake': ctrl.hand_brake,
                'reverse': ctrl.reverse,
                'manual_gear_shift': ctrl.manual_gear_shift,
                'gear': ctrl.gear,
                'vehicles': states_npc_vehicles,
                'pedestrians': states_npc_pedestrians,
                'id_junction': self.map.get_waypoint(p.location).junction_id,
                "drivefuzz": {
                    'rate_yaw': yaw_rate,
                    'lat_speed': lat_speed,
                    'lon_speed': lon_speed,
                    'steer': ctrl.steer * max_steer_angle,
                    "dist_min": self.dist_min
                }
            })
            # Check collision
            if self.collision_sensor.check_collision():
                self.state.result['collision'] = True
                self.state.result['collision_with'] = self.collision_sensor.check_collision_with(
                )
                with (self.path_log / 'violation.txt').open('w') as _f:
                    _f.write('collision_vehicle')
            # Check collision with pedestrian
            bb_ego = self.ego_vehicle.bounding_box
            bb_ego.extent.x *= 1.05
            bb_ego.extent.y *= 1.05
            for id_p, pedestrian, _, (loc_dest, _), type_motion in self.pedestrians:
                # Collision check
                if pedestrian.is_alive and p.location.distance(pedestrian.get_location()) < 5.0:
                    bb_ped = pedestrian.bounding_box
                    if any([bb_ego.contains(_v, p) for _v in bb_ped.get_world_vertices(pedestrian.get_transform())]):
                        self.state.result['collision'] = True
                        self.state.result['collision_with'] = id_p
                        with (self.path_log / 'violation.txt').open('w') as _f:
                            _f.write('collision_ped')
            # Check stalling
            if self.timer_stalling > FPS * 20:    # 20frame/s * 20 sec
                self.state.result['stalling'] = True
                with (self.path_log / 'violation.txt').open('w') as _f:
                    _f.write('stalling')
            if speed > 1:
                self.timer_stalling = 0
            else:
                self.timer_stalling += 1
            # Check speeding
            self.speed_max = self.ego_vehicle.get_speed_limit() * 1.1
            if abs(self.speed_max - self.speed_max_prev) < 2:
                # If speed limit is decreased, no detection until the current speed is less than the decreased speed limit.
                if self.speed_max < self.speed_max_prev:
                    self.no_speeding = True
                self.speed_max_prev = self.speed_max
            if self.no_speeding:
                # If speed decreased enough, start detection.
                if speed <= self.speed_max:
                    self.no_speeding = False
                self.buffer_speeding = FPS * 3    # 3 seconds
            if speed > self.speed_max:
                self.buffer_speeding -= 1
                if self.buffer_speeding <= 0:
                    self.state.result['speeding'] = True
                    with (self.path_log / 'violation.txt').open('w') as _f:
                        _f.write(f'speeding({speed}>{self.speed_max})')
            # Check destination
            dest = get_carla_transform(self.scenario.mission['dest'])
            # os.system(f'echo "{p, dest}" >> autoware/log.txt')
            if self.__distance_2d(p.location, dest.location) < 1.0:
                # if p.location.distance(dest.location) < 1.0:
                self.state.result['normal'] = True
            # Check timeout (15 min)
            self.time_running = time.time() - self.time_start
            if self.time_running > 15 * 60:
                self.state.result['timeout'] = True
                with (self.path_log / 'violation.txt').open('w') as _f:
                    _f.write('timeout')
            # End condition
            if self.state.result['normal'] or self.state.any_violation():
                done = self.path_log / 'done'
                done.touch()
                self.state.dump()
                self.clean_up()
                end = self.path_log / 'end'
                end.touch()
                # Wait until to the end
            return any(self.state.result.values())
        except:
            import traceback
            os.system(f'echo "{traceback.format_exc()}" >> autoware/log.txt')

    def clean_up(self) -> None:
        sensors = [
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
        ]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        for camera in self.camera_logging:
            try:
                camera.destroy()
            except RuntimeError as _:
                continue
        self.camera_logging = []
        self.__save_camera_video()

    def __save_camera_video_aux(self, loc: str) -> None:
        if len(list(self.path_images.glob(f'{loc}-*.jpg'))) > 0:
            cmd_cat = f"cat $(ls {self.path_images / f'{loc}-*.jpg'} | sort -V)"
            cmd_ffmpeg = " ".join([
                "ffmpeg",
                "-f image2pipe",
                f"-r {FPS}",
                "-vcodec mjpeg",
                "-i -",
                "-vcodec libx264",
                f'"{self.log_dir / loc}.mp4" > /dev/null 2>&1'
            ])
            try:
                cmd = f"{cmd_cat} | {cmd_ffmpeg}"
                os.system(cmd)
            except Exception as _:
                import traceback
                os.system(
                    f'echo "{traceback.format_exc()}" >> autoware/log.txt')
        else:
            pass

    def __save_camera_video(self) -> None:
        if hasattr(self, 'path_images'):
            try:
                self.__save_camera_video_aux('front')
                self.__save_camera_video_aux('top')
            except Exception as _:
                import traceback
                os.system(
                    f'echo "{traceback.format_exc()}" >> autoware/log.txt')
        self.__clean_image_dir()

    def __clean_image_dir(self) -> None:
        cmd = f"rm -f {self.path_images}/*.jpg"
        os.system(cmd)
