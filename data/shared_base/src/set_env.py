import os
from pathlib import Path
import math
import time
from typing import Any, Dict, List, Optional, Tuple, Union

from ..modules.carla_data_provider import CarlaDataProvider
from .scenario import Scenario
from .sensors import CollisionSensor, LaneInvasionSensor
from .state import State
from .utils import get_carla_transform, GlobalRoutePlanner, DummyWorld, get_bbox, angle_from_center_view_fov, norm_2d

import carla
import numpy as np

# from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent

FPS = 20


class Environment:

    def __init__(self, ego_vehicle: carla.Actor, tm: carla.TrafficManager) -> None:
        if not Path('shared/logs/other').exists():
            self.scenario: Scenario = Scenario(Path('shared/logs'))
            self.path_log: Path = self.scenario.path_scenario
            self.world: carla.World = CarlaDataProvider.get_world()
            self.map: carla.Map = self.world.get_map()
            self.vehicles: List[Tuple[int, carla.Actor, Any, Any, str]] = []
            self.pedestrians: List[Tuple[int, carla.Actor, Any, Any, str]] = []
            self.puddles: List[carla.BoundingBox] = []
            # For simulation
            self.ego_vehicle: carla.Actor = ego_vehicle
            self.timer_stalling: int = 0
            self.__init: bool = False
            self.state: State = State(self.scenario.path_scenario, self.scenario.map)
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
            self.threshold_stalling: int = 20
            self.tm: carla.TrafficManager = tm
            self.min_d: float = 10000.0
            self.d_angle_norm: float = 1.0
            self.dev_dist: float = 0.0
            self.offroad_d: float = 10000.0
            self.wronglane_d: float = 10000.0
        else:
            self.__init: bool = False
            self.ego_vehicle: carla.Actor = ego_vehicle
            self.world: carla.World = CarlaDataProvider.get_world()
            self.camera_logging = []

    def spawn_vehicles(self, vehicles: List[Dict[str, Any]]) -> List[Tuple[int, carla.Actor, Any, Any, str]]:
        spawned_vehicles = []
        gp = GlobalRoutePlanner(self.map, 0.5)
        for i, v in enumerate(vehicles):
            # vehicle = CarlaDataProvider.request_new_actor(
            #     v['bp'], get_carla_transform(v['start']), f'v{i}'
            # )
            # if v['start']['z'] < 0.01:
            v['start']['z'] += 2.0
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
                        # self.mark_point(get_carla_transform(v['start']), color='red', size=(2, 2))
                        agent.get_local_planner().set_global_plan(trace)
                        dummy_world = DummyWorld(self.world, vehicle)
                        spawned_vehicles.append(
                            (v['id'], vehicle, agent, dest.location, v['type_motion']))
                    elif v['type_motion'] == 'linear':
                        start = get_carla_transform(v['start'])
                        dest = get_carla_transform(v['dest'])
                        speed_v = v['speed']
                        dir_v = start.get_forward_vector().make_unit_vector()
                        vector = speed_v * dir_v
                        vehicle.set_target_velocity(vector)
                        spawned_vehicles.append(
                            (v['id'], vehicle, dest, vector, v['type_motion']))
                    elif v['type_motion'] == 'static':
                        spawned_vehicles.append(
                            (v['id'], vehicle, [], '', v['type_motion']))
                # For drivefuzz
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
                elif 'nav_type' in v and v['nav_type'] == 3:  # Maneuver
                    maneuvers = [
                        [0, 0, 0],
                        [0, 8, 0],
                        [0, 8, 0],
                        [0, 8, 0],
                        [0, 8, 0],
                    ]
                    spawned_vehicles.append(
                        (v['id'], vehicle, [], (sp_t, maneuvers), v['nav_type']))
                else:
                    spawned_vehicles.append(
                        (v['id'], vehicle, [], '', v['type_motion']))
        return spawned_vehicles

    def spawn_pedestrians(self, pedestrians: List[Dict[str, Any]]) -> List[Tuple[int, carla.Actor, Any, Any, str]]:
        spawned_pedestrians = []
        for i, p in enumerate(pedestrians):
            # pedestrian = CarlaDataProvider.request_new_actor(
            #     p['bp'], get_carla_transform(p['start']), f'p{i}'
            # )
            # if p['start']['z'] < 0.01:
            p['start']['z'] += 2.0
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
                        # self.mark_point(get_carla_transform(
                        #     p['start']), color='red', size=(2, 2))
                        if controller is not None:
                            # pedestrian.set_simulate_physics(enabled=False)
                            gp = GlobalRoutePlanner(self.map, 2.0)
                            dest = get_carla_transform(p['dest'])
                            trace = gp.trace_route(
                                sp_t.location, dest.location)
                            _wps = [self.map.get_waypoint(wp.transform.location, lane_type=carla.LaneType.Sidewalk).transform
                                   for wp, _ in trace]
                            if len(_wps) < 10:
                                _wps = [self.map.get_waypoint(wp.transform.location).transform
                                   for wp, _ in trace]
                            # for _wp in _wps:
                            #     self.mark_point(_wp, color='blue', size=(0.2, 0.2))
                            # _wps = [sp_t] + _wps[1:-1] + [dest]
                            # _wps = [sp_t] + _wps[1:]
                            wps = []
                            for j in range(len(_wps) - 1):
                                if _wps[j].location == _wps[j+1].location:
                                    continue
                                wps.append(_wps[j])
                            _speed = float(p['speed'])
                            spawned_pedestrians.append(
                                (p['id'], pedestrian, controller,
                                 (wps, _speed), p['type_motion'])
                            )
                    elif p['type_motion'] == 'linear':
                        dest = get_carla_transform(p['dest'])
                        forward_vec = sp_t.rotation.get_forward_vector()
                        controller_walker = carla.WalkerControl()
                        controller_walker.direction = forward_vec
                        controller_walker.speed = p["speed"]
                        pedestrian.apply_control(controller_walker)
                        spawned_pedestrians.append(
                            (p['id'], pedestrian, dest, (None, None), p['type_motion']))
                    elif p['type_motion'] == 'static':
                        spawned_pedestrians.append(
                            (p['id'], pedestrian, None, (None, None), p['type_motion']))
                # For DriveFuzz
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
                        # controller.start()
                        # controller.set_max_speed(speed_max)
                        # controller.go_to_location(dest.location)
                    # spawned_pedestrians.append(
                    #     (p['id'], pedestrian, None, (None, None), p['nav_type']))
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

    def __initialize(self) -> None:
        self.vehicles = self.spawn_vehicles(self.scenario.npcs['vehicles'])
        self.pedestrians = self.spawn_pedestrians(
            self.scenario.npcs['pedestrians'])
        self.puddles = self.spawn_puddles(self.scenario.puddles)
        self.set_environment(self.scenario.weather, self.scenario.time)
        dest = get_carla_transform(self.scenario.mission['dest'])
        self.attach_cameras_for_logging(self.path_log)
        CarlaDataProvider.get_world().tick()
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
                # p.set_target_velocity(dir_diff * speed)
                controller_walker = carla.WalkerControl()
                controller_walker.direction = dir_diff
                controller_walker.speed = speed_ped
                p.apply_control(controller_walker)
                # actor_walker.apply_control(controller_walker)
                # os.system(f'echo "{dir_diff}" >> autoware/log.txt')
                # os.system(f'echo "{dir_diff * speed}" >> autoware/log.txt')
                # os.system(f'echo "{speed}" >> autoware/log.txt')
                # os.system('echo "-------" >> autoware/log.txt')
                # controller.start()
                # controller.go_to_location(wps_ped[1])   # [0] is a start point
                # controller.set_max_speed(speed_max)
            if type_motion == 1: # Autopilot
                controller.start()
                controller.set_max_speed(speed_ped)
                controller.go_to_location(dest.location)
            p.set_collisions(True)
            p.set_simulate_physics(True)

    def run_step(self) -> None:
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
                return
            if (self.path_log / 'lock').exists() or (self.path_log / 'end').exists():
                self.time_start = time.time()
                self.time_running = 0.0
                _brake = carla.VehicleControl(brake=1.0)
                self.ego_vehicle.apply_control(_brake)
                # self.ego_vehicle.set_target_velocity(carla.Vector3D())
                # os.system(f'echo "{self.ego_vehicle.get_velocity()}" >> autoware/log.txt ')
                # os.system(f'echo "{self.ego_vehicle.get_transform()}" >> autoware/log.txt ')
                return
            if not self.__init:
                # if (self.path_log / 'drivefuzz').exists() or (self.path_log / 'samota').exists():
                #     self.threshold_stalling = 20
                self.__init = True
                self.__initialize()
            speed = self.ego_vehicle.get_velocity().length() * 3.6
            p = self.ego_vehicle.get_transform()
            # Run and log NPC Vehicles' state
            states_npc_vehicles = []
            for id_v, vehicle, agent, _maneuvers, type_motion in self.vehicles:
                if vehicle is not None and vehicle.is_alive:
                    # # I set the maximum distance as 100, since current lidar sensor considers <= 100 only.
                    # if vehicle.get_location().distance(p.location) < 100:
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
                        # agent.get_local_planner().set_speed(30)
                        try:
                            ctrl = agent.run_step()
                            vehicle.apply_control(ctrl)
                            ctrl_p = vehicle.get_physics_control()
                            for _wheel in ctrl_p.wheels:
                                _wheel.tire_friction = 3.5
                            if agent.done():
                                # Stop vehicle
                                vehicle.set_target_velocity(carla.Vector3D())
                        except:
                            pass
                    elif type_motion == 'linear':
                        dest = agent
                        if self.__distance_2d(dest.location, vehicle.get_location()) < 1:
                            vehicle.set_target_velocity(carla.Vector3D())
                        else:
                            vehicle.set_target_velocity(_maneuvers)
                    elif type_motion == 'static':
                        if len(self.state.state) > 1.5 * FPS:
                            vehicle.set_target_velocity(carla.Vector3D())
                    if type_motion == 3:    # Maneuver
                        sp, maneuvers = _maneuvers
                        maneuver_id = int(len(self.state.state) / 100)
                        if maneuver_id < 5:
                            maneuver = maneuvers[maneuver_id]

                            if maneuver[2] == 0:
                                # print(f"\nPerforming maneuver #{maneuver_id} at frame {state.num_frames}")
                                # mark as done
                                maneuver[2] = len(self.state.state)

                                # retrieve the actual actor vehicle object
                                # there is only one actor in Trajectory mode
                                actor_vehicle = vehicle

                                # perform the action
                                actor_direction = maneuver[0]
                                actor_speed = maneuver[1]

                                forward_vec = sp.rotation.get_forward_vector()

                                if actor_direction == 0: # forward

                                    actor_vehicle.set_target_velocity(
                                        forward_vec * actor_speed
                                    )

                            elif maneuver[2] > 0 and abs(maneuver[2] - len(self.state.state)) < 40:
                                # continuously apply lateral force to the vehicle
                                # for 40 frames (2 secs)
                                actor_direction = maneuver[0]
                                apex_degree = maneuver[1]

                                """
                                Model smooth lane changing through varying thetas
                                (theta)
                                45           * *
                                30       * *     * *
                                15     *             * *
                                0  * *                   *
                                0 5 10 15 20 25 30 35 40 (t = # frame)
                                """

                                theta_max = apex_degree
                                force_constant = 5 # should weigh by actor_speed?

                                t = abs(maneuver[2] - len(self.state.state))
                                if t < 20:
                                    theta = t * (theta_max / 20)
                                else:
                                    theta = t * -1 * (theta_max / 20) + 2 * theta_max

                                if actor_direction != 0: # skip if fwd
                                    if actor_direction == -1: # switch to left lane
                                        theta *= -1 # turn cc-wise
                                    elif actor_direction == 1: # switch to right lane
                                        pass # turn c-wise

                                    theta_rad = math.radians(theta)
                                    sin = math.sin(theta_rad)
                                    cos = math.cos(theta_rad)

                                    x0 = forward_vec.x
                                    y0 = forward_vec.y

                                    x1 = cos * x0 - sin * y0
                                    y1 = sin * x0 + cos * y0

                                    dir_vec = carla.Vector3D(x=x1, y=y1, z=0.0)
                                    actor_vehicle.set_target_velocity(
                                        dir_vec * force_constant
                                    )
                    # elif type_motion == 'linear':
                    #     vehicle.set_target_velocity(vector)

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
                        try:
                            tr_next = trs_ped[self.idx_ped[id_p]]
                            if self.__distance_2d(tr_next.location, pedestrian.get_location()) < 0.1:
                                __loc = carla.Location(
                                    x=tr_next.location.x,
                                    y=tr_next.location.y,
                                    z=pedestrian.get_location().z
                                )
                                pedestrian.set_location(__loc)
                                if self.idx_ped[id_p] == len(trs_ped) - 1:
                                    # The end of the route
                                    controller_walker = carla.WalkerControl()
                                    controller_walker.speed = 0.0
                                    pedestrian.apply_control(controller_walker)
                                    # pedestrian.destroy()
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
                                    except RuntimeError as e:
                                        import traceback
                                        os.system(f'echo "{traceback.format_exc()}" >> autoware/log.txt')
                                        os.system(f'echo "{tr_next_next.location}" >> autoware/log.txt')
                                        os.system(f'echo "{tr_next.location}" >> autoware/log.txt')
                                        os.system(f'echo "------" >> autoware/log.txt')
                        except:
                            pass
                                # pedestrian.set_target_velocity(
                                #     dir_diff * speed)
                                # os.system(
                                #     f'echo "{dir_diff}" >> autoware/log.txt')
                                # os.system(
                                #     f'echo "{dir_diff * speed}" >> autoware/log.txt')
                                # os.system(
                                #     f'echo "{speed}" >> autoware/log.txt')
                                # os.system('echo "-------" >> autoware/log.txt')
                    # if type_motion == 1: # Autopilot
                    #     os.system(f'echo "{pedestrian.get_location()}" >> autoware/log.txt')
            ctrl = self.ego_vehicle.get_control()
            ### For DriveFuzz ###
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
            yaw_diff = yaw_curr - self.yaw_prev
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
            elif yaw_diff < -180:
                yaw_diff = 360 + yaw_diff

            yaw_rate = yaw_diff * FPS
            self.yaw_prev = yaw_curr
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
            ### For AutoFuzz ###
            # ego_bbox = get_bbox(self.ego_vehicle)
            # ego_front_bbox = ego_bbox[:2]
            # for _, vehicle, _, _, _ in self.vehicles:
            #     if vehicle is None or not vehicle.is_alive:
            #         continue
            #     if vehicle.id == self.ego_vehicle.id:
            #         continue

            #     d_angle_norm_i = angle_from_center_view_fov(vehicle, self.ego_vehicle, fov=90)
            #     self.d_angle_norm = np.min([self.d_angle_norm, d_angle_norm_i])
            #     if d_angle_norm_i == 0:
            #         other_bbox = get_bbox(vehicle)
            #         for other_b in other_bbox:
            #             for ego_b in ego_bbox:
            #                 d = norm_2d(other_b, ego_b)
            #                 # print('vehicle', i, 'd', d)
            #                 self.min_d = np.min([self.min_d, d])


            # for _, pedestrian, _, _, _ in self.pedestrians:
            #     if pedestrian is None or not pedestrian.is_alive:
            #         continue
            #     d_angle_norm_i = angle_from_center_view_fov(pedestrian, self.ego_vehicle, fov=90)
            #     self.d_angle_norm = np.min([self.d_angle_norm, d_angle_norm_i])
            #     if d_angle_norm_i == 0:
            #         pedestrian_location = pedestrian.get_transform().location
            #         for ego_b in ego_front_bbox:
            #             d = norm_2d(pedestrian_location, ego_b)
            #             # print('pedestrian', i, 'd', d)
            #             self.min_d = np.min([self.min_d, d])
            # current_waypoint = self.map.get_waypoint(self.ego_vehicle.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
            # ego_forward = self.ego_vehicle.get_transform().get_forward_vector()
            # ego_forward = np.array([ego_forward.x, ego_forward.y])
            # ego_forward /= np.linalg.norm(ego_forward)
            # ego_right = self.ego_vehicle.get_transform().get_right_vector()
            # ego_right = np.array([ego_right.x, ego_right.y])
            # ego_right /= np.linalg.norm(ego_right)
            # lane_center_waypoint = self.map.get_waypoint(self.ego_vehicle.get_location(), lane_type=carla.LaneType.Any)
            # lane_center_transform = lane_center_waypoint.transform
            # lane_center_location = lane_center_transform.location
            # lane_forward = lane_center_transform.get_forward_vector()
            # lane_forward = np.array([lane_forward.x, lane_forward.y])
            # lane_forward /= np.linalg.norm(lane_forward)
            # lane_right = self.ego_vehicle.get_transform().get_right_vector()
            # lane_right = np.array([lane_right.x, lane_right.y])
            # lane_right /= np.linalg.norm(lane_right)
            # dev_dist = self.ego_vehicle.get_location().distance(lane_center_location)
            # dev_angle = math.acos(np.clip(np.dot(ego_forward, lane_forward), -1, 1)) / np.pi
            # dev_dist *= (dev_angle + 0.5)
            # if dev_dist > self.dev_dist:
            #     self.dev_dist = dev_dist
            # if current_waypoint and not current_waypoint.is_junction:
            #     self.__get_d(-0.1, lane_right, 'left', lane_center_waypoint)
            #     self.__get_d(0.1, lane_right, 'right', lane_center_waypoint)
            # self.__get_d(-0.1, ego_right, 'ego_left', lane_center_waypoint)
            # self.__get_d(0.1, ego_right, 'ego_right', lane_center_waypoint)
            # self.__get_d(0.1, ego_forward, 'ego_forward', lane_center_waypoint)
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
                # 'lane_invasion': self.check_lane_invasion(),
                # 'at_traffic_light': world.player.is_at_traffic_light(),
                # 'affecting_traffic_light_state': state_traffic_light,
                'vehicles': states_npc_vehicles,
                'pedestrians': states_npc_pedestrians,
                # 'statics': states_statics,
                'id_junction': self.map.get_waypoint(p.location).junction_id,
                "drivefuzz": {
                    'rate_yaw': yaw_rate,
                    'lat_speed': lat_speed,
                    'lon_speed': lon_speed,
                    'steer': ctrl.steer * max_steer_angle,
                    "dist_min": self.dist_min
                },
                # "autofuzz": {
                #     'd_angle_norm': self.d_angle_norm,
                #     'min_d': self.min_d,
                #     'dev_dist': self.dev_dist,
                #     'offroad_d': self.offroad_d,
                #     'wronglane_d': self.wronglane_d
                # }
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
            if self.timer_stalling > FPS * self.threshold_stalling:    # 20frame/s * 20 sec
                self.state.result['stalling'] = True
                with (self.path_log / 'violation.txt').open('w') as _f:
                    _f.write('stalling')
            if speed > 1:
                self.timer_stalling = 0
            else:
                self.timer_stalling += 1
            # Check lane invasion
            if self.lane_invasion_sensor.check_lane_invasion():
                self.state.result['lane_invasion'] = True
                with (self.path_log / 'violation.txt').open('w') as _f:
                    _f.write('lane_invasion')
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
                while True:
                    time.sleep(10)
        except:
            import traceback
            os.system(f'echo "{traceback.format_exc()}" >> autoware/log.txt')

    def clean_up(self) -> None:
        if not Path('shared/logs/other').exists():
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
                f'"{self.log_dir / loc}.mp4"'
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

    def __get_d(self, coeff, dir, dir_label, lane_center_waypoint):
        n = 1
        current_location = self.ego_vehicle.get_location()
        while n*coeff < 7:
            new_loc = carla.Location(current_location.x + n*coeff*dir[0], current_location.y + n*coeff*dir[1], 0)
            # print(coeff, dir, dir_label)
            # print(dir_label, 'current_location, dir, new_loc', current_location, dir, new_loc)
            new_wp = self.map.get_waypoint(new_loc,project_to_road=False, lane_type=carla.LaneType.Any)

            if not (new_wp and new_wp.lane_type in [carla.LaneType.Driving, carla.LaneType.Parking, carla.LaneType.Bidirectional] and np.abs(new_wp.transform.rotation.yaw%360 - lane_center_waypoint.transform.rotation.yaw%360) < 120):
                # if new_wp and new_wp.lane_type in [carla.LaneType.Driving, carla.LaneType.Parking, carla.LaneType.Bidirectional]:
                #     print('new_wp.transform.rotation.yaw, lane_center_waypoint.transform.rotation.yaw', new_wp.transform.rotation.yaw, lane_center_waypoint.transform.rotation.yaw)
                break
            else:
                n += 1
            # if new_wp:
            #     print(n, new_wp.transform.rotation.yaw)

        d = new_loc.distance(current_location)
        # print(d, new_loc, current_location)


        if new_wp and new_wp.lane_type in [carla.LaneType.Driving, carla.LaneType.Parking, carla.LaneType.Bidirectional]:
            # print(dir_label, 'wronglane_d', d)
            if d < self.wronglane_d:
                self.wronglane_d = d
                # f_out.write('wronglane_d,'+str(self.wronglane_d)+'\n')
                # print(dir_label, 'current_location, dir, new_loc', current_location, dir, new_loc, 'wronglane_d,'+str(self.wronglane_d)+'\n')
        else:
            # if not new_wp:
            #     s = 'None wp'
            # else:
            #     s = new_wp.lane_type
            # print(dir_label, 'offroad_d', d, s, coeff)
            # if new_wp:
                # print(dir_label, 'lanetype', new_wp.lane_type)
            if d < self.offroad_d:
                self.offroad_d = d
                # f_out.write('offroad_d,'+str(self.offroad_d)+'\n')
                # print(dir_label, 'current_location, dir, new_loc', current_location, dir, new_loc, 'offroad_d,'+str(self.offroad_d)+'\n')