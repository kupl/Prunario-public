import json
import math
from pathlib import Path
import time
from typing import Tuple

from fuzz.ads import ADS, Autoware
from fuzz.carla import cc
from fuzz.carla.utils import *
from fuzz.commons.constants import *
from fuzz.commons.exceptions import RunAgain, RunNextMutant
from fuzz.commons.utils import get_logger, Timer
from fuzz.data import Record, Scenario
# from fuzz.simulation.set_env_new import Environment
from fuzz.simulation.utils import copy_simulation_result, clean_logging_dir, init_logging_dir, is_fp

import numpy as np
from timeout_decorator.timeout_decorator import TimeoutError


def __wait_until_driving_done(path_run: Path,
                              scenario: Scenario,
                              ads: ADS,
                              pid_record: int,
                              timer: Timer,
                              mute: bool = False,
                              interval: float = POLLING_INTERVAL) -> Record:
    # Wait until the driving done
    t = time.time()
    # Setting for a max speed of the ego vehicle
    # if ads.id == 'autoware':
    if isinstance(ads, Autoware):
        ego = [actor
            for actor in cc.get_world().get_actors().filter('vehicle.*')
            if actor.attributes['role_name'] == 'ego_vehicle'].pop()
    else:   # old version
        ego = [actor
            for actor in cc.get_world().get_actors().filter('vehicle.*')
            if actor.attributes['role_name'] in ['autoware_v1', 'autoware_v2']].pop()
    # elif ads.id == 'behavioragent':
    #     ego = ads.ego
    # else:
    #     ego = cc.get_ego_apollo() #cc.get_world().get_actors().filter(ads.bp)[0]
        # print(cc.get_world().get_actors())
        # ego = [actor for actor in cc.get_world().get_actors() if actor.attributes['role_name'] in ['ego_vehicle', 'hero']].pop()
    # if ads.id == 'apollo':
    # if ads.id in ['apollo', 'behavioragent']:
    #     scenario.path_scenario = Config.PATH_SIMULATION_LOG
    #     env = Environment(scenario, ego, ads, cc.get_world()) #, cc.get_tm())
    #     env.time_start = time.time()
    #     env.time_running = 0.0
    #     env.initialize()
    #     scenario.path_scenario = path_run
    
    speed_max_curr = ego.get_speed_limit()
    start = get_carla_location(scenario.mission['start'])
    dest = get_carla_location(scenario.mission['dest'])
    # if target == 'autoware':
    #     __set_speed_limit(30)
    any_violation = False
    # if ads.id == 'autoware':
    def cond_end(): return not (Config.PATH_SIMULATION_LOG / 'done').is_file()
    ads.restart_planner(wait=False)
    # elif ads.id == 'apollo':
    #     def cond_end(): return not (Config.PATH_SIMULATION_LOG / 'done').is_file()
    #     __id_callback = cc.get_world().on_tick(lambda _: env.run_step())
    #     res = ads.set_goal(scenario.mission)
    #     # def cond_end(): return not any_violation
    cnt_stuck = 0
    p_prev = start
    t_check = 0
    if mute:
        get_logger().info(f"Waiting for the driving to be done...")
    while cond_end():
        _t = time.time()
        t_run = time.time() - t
        t_all = time.time() - timer.time_start
        if t_run > 1200:  # if driving stucks
            get_logger().info("Driving timeout. Run again.")
            raise RunAgain
        speed_max = ego.get_speed_limit()
        if ego.get_location() == p_prev:
            cnt_stuck += 1
        if (speed_max <= 5 or speed_max > 200) or (cnt_stuck > 60 * 20):
            print()
            get_logger().info("Bad connection with Carla. Run again.")
            raise RunAgain
        if not mute:
            speed_curr = 3.6 * ego.get_velocity().length()
            x_ego = ego.get_location().x
            y_ego = ego.get_location().y
            dist = math.sqrt((x_ego - dest.x)**2 + (y_ego - dest.y)**2)
            _str_log = f"\r[Fuzzing time: {t_all:.1f} (s)] [{t_run:.1f} (s)]:"
            _str_log += f" {speed_curr:.1f}/{speed_max_curr:.1f} (km/h)"
            # _str_log += f" {speed_curr:.1f} (km/h)"
            _str_log += f" ({x_ego:.1f},{y_ego:.1f})>({dest.x:.1f},{dest.y:.1f})"
            _str_log += f" {dist:.1f}m left        "
            print(_str_log, end='')
        # if ads.id == 'apollo':
        #     # if t_run < 20 and speed_curr < 1:
        #     #     get_logger().info("Run the next mutant due to the routing failure.")
        #     #     raise RunNextMutant
        #     # any_violation = env.run_step()
        #     if t_run - t_check > 3:
        #         ads.keep_module_alive()
        #         t_check = t_run
        # if ads.id == 'behavioragent':
        #     env.run_step()
        # #     cc.tick()
        time.sleep(max(1 / FPS - (time.time() - _t), 0))
    timer.end_driving_simulation()
    print()
    get_logger().info("Driving is done. Saving logs.")
    # if ads.id == 'autoware':
    while not (Config.PATH_SIMULATION_LOG / 'end').is_file():
        # time.sleep(max(1 / FPS - (time.time() - _t), 0))
        cc.tick()
        time.sleep(1 / FPS)
    if ads.id == 'apollo':
        cc.get_world().remove_on_tick(__id_callback)
    # if ads.id == 'behavioragent':
    #     cc.get_world().remove_on_tick(__id_callback1)
    #     cc.get_world().remove_on_tick(__id_callback2)
    # Save the simulation result
    any_violation = (Config.PATH_SIMULATION_LOG /
                        'violation.txt').is_file()
    if any_violation:
        if (Config.PATH_SIMULATION_LOG / 'violation.txt').open().read().strip() == 'timeout':
            any_violation = False
    if any_violation:
        ads.finish_recording(pid_record)
    copy_simulation_result(path_run, scenario, ads=ads)
    record = Record(path_run)
    ads.restart()
    return record


def clean_up(ads: ADS) -> None:
    get_logger().info("Cleaning up the simulation")
    clean_logging_dir()
    ads.restart()
    # __restart_target(target)
    get_logger().info("Done.")


def run(path_run: Path,
        scenario: Scenario,
        ads: ADS,
        timer: Timer,
        id_gpu: int,
        mute: bool = False,
        is_drivefuzz: bool = False,
        is_samota: bool = False) -> Record:
    # if ads.id == 'apollo':
    #     cc.reload_world()
    init_logging_dir()
    while True:
        chances_run_again = CHANCES_RUN_AGAIN
        # scenario.path_scenario = path_run
        cc.load_world(scenario.map)
        while True:
            # pid_record = __start_recording(target)
            pid_record = ads.start_recording()
            scenario.dump(Config.PATH_SIMULATION_LOG)
            if ads.id == 'autoware':
                Config.PATH_LOCK.touch()
                if is_drivefuzz:
                    (Config.PATH_SIMULATION_LOG / 'drivefuzz').touch()
                elif is_samota:
                    (Config.PATH_SIMULATION_LOG / 'samota').touch()
            if chances_run_again <= 0:
                get_logger().info("No more try for the mutant. Try next mutant.")
                raise RunNextMutant
            chances_run_again -= 1
            try:
                timer.start_simulation()
                # Run target ADS
                if not ads.connect(scenario=scenario,
                                   timeout=LOAD_TIMEOUT,
                                   id_gpu=id_gpu,
                                   use_traffic_manager=(is_drivefuzz or is_samota)):
                    get_logger().info(f"Failed to load {ads.id}. Run next mutant.")
                    raise RunNextMutant
                    # raise RunAgain
                break
            except RunAgain:
                clean_up(ads)
                continue
            except TimeoutError:
                get_logger().info(f"Timeout error. Run next mutant.")
                clean_up(ads)
                raise RunNextMutant
        if ads.id in ['autoware', 'behavioragent']:
            # Send routing request
            get_logger().info(f"Send routing request to {ads.id}.")
            res = ads.set_goal(scenario.mission)
            if not res:
                get_logger().info("Run the next mutant due to the routing failure.")
                chances_run_again += 1
                raise RunNextMutant
        # Start driving
        timer.start_driving_simulation()
        # Unlock the logging
        if ads.id == 'autoware':
            Config.PATH_LOCK.unlink()
            ads.engage()
        elif ads.id == 'behavioragent':
            ads.engage()
        # __start_driving()
        # wait until the driving to be done
        record = __wait_until_driving_done(path_run,
                                           scenario=scenario,
                                           ads=ads,
                                           timer=timer,
                                           pid_record=pid_record,
                                           mute=mute)
        # if is_fp(record, timeout_stalling=Config.TIMEOUT_STALLING):
        #     get_logger().info("False positive is detected")
        #     with (path_run / 'fp').open('w') as f:
        #         f.write('')
        timer.end_simulation()
        return record


def run_drivefuzz(path_run,
                  scen_drivefuzz,
                  ads,
                  timer,
                  id_gpu):
    path_run = Path(path_run)
    path_run.mkdir(parents=True, exist_ok=True)
    scenario = Scenario(path_scenario=Path(''), empty=True)
    scenario.path_scenario = Path(path_run)
    # Map
    scenario.map = scen_drivefuzz.town
    # Mission
    _start = {
        "x": scen_drivefuzz.seed_data["sp_x"],
        "y": scen_drivefuzz.seed_data["sp_y"],
        "z": scen_drivefuzz.seed_data["sp_z"],
        "roll": scen_drivefuzz.seed_data["roll"],
        "pitch": scen_drivefuzz.seed_data["pitch"],
        "yaw": scen_drivefuzz.seed_data["yaw"]
    }
    _dest = {
        "x": scen_drivefuzz.seed_data["wp_x"],
        "y": scen_drivefuzz.seed_data["wp_y"],
        "z": scen_drivefuzz.seed_data["wp_z"],
        "yaw": scen_drivefuzz.seed_data["wp_yaw"]
    }
    wp_dest = cc.get_the_nearest_wp(_dest)
    _dest['roll'] = wp_dest.transform.rotation.roll
    _dest['pitch'] = wp_dest.transform.rotation.pitch
    _dest['yaw'] = wp_dest.transform.rotation.yaw
    scenario.mission = {
        'start': _start,
        "dest": _dest
    }
    # NPCs
    scenario.npcs = {
        'vehicles': [],
        'pedestrians': []
    }
    for _id, actor in enumerate(scen_drivefuzz.actors):
        _target = 'vehicles' if actor['type'] == 0 else "pedestrians"
        _bp = "vehicle.bmw.grandtourer" if actor['type'] == 0 else "walker.pedestrian.0001"
        scenario.npcs[_target].append({
            "id": _id,
            "nav_type": actor['nav_type'],
            "bp": _bp,
            "start": {
                "x": actor['spawn_point'][0][0],
                "y": actor['spawn_point'][0][1],
                "z": actor['spawn_point'][0][2],
                "roll": actor['spawn_point'][1][0],
                "pitch": actor['spawn_point'][1][1],
                "yaw": actor['spawn_point'][1][2]
            },
            "dest": {
                "x": actor['dest_point'][0][0],
                "y": actor['dest_point'][0][1],
                "z": actor['dest_point'][0][2],
                "roll": actor['dest_point'][1][0],
                "pitch": actor['dest_point'][1][1],
                "yaw": actor['dest_point'][1][2]
            } if actor['nav_type'] == 1 else None,
            "speed": actor['speed'],
            "maneuvers": actor['maneuvers']
        })
    # Puddles
    for puddle in scen_drivefuzz.puddles:
        scenario.puddles.append({
            "level": puddle['level'],
            "x": puddle['spawn_point'][0][0],
            "y": puddle['spawn_point'][0][1],
            "z": 0,
            "size_x": puddle['size'][0],
            "size_y": puddle['size'][1],
            "size_z": 1000
        })
    # Weather
    scenario.weather['cloudiness'] = scen_drivefuzz.weather['cloud']
    scenario.weather['precipitation'] = scen_drivefuzz.weather['rain']
    scenario.weather['precipitation_deposits'] = scen_drivefuzz.weather['puddle']
    scenario.weather['wind_intensity'] = scen_drivefuzz.weather['wind']
    scenario.weather['fog_density'] = scen_drivefuzz.weather['fog']
    scenario.weather['wetness'] = scen_drivefuzz.weather['wetness']
    scenario.weather['dust_storm'] = 0
    # Time
    scenario.time['sun_azimuth_angle'] = scen_drivefuzz.weather['angle']
    scenario.time['sun_altitude_angle'] = scen_drivefuzz.weather['altitude']

    # Save scenario
    scenario.dump(scenario.path_scenario)
    with (path_run / 'scenario_d.json').open('w') as f:
        json.dump({
            'actors': scen_drivefuzz.actors,
            'puddles': scen_drivefuzz.puddles,
            'weather': scen_drivefuzz.weather
        }, f, indent=4)

    return run(path_run, scenario, ads, timer, id_gpu, is_drivefuzz=True)


def run_scenariofuzz(path_run,
                     scen_scenariofuzz,
                     ads,
                     timer,
                     id_gpu):
    path_run = Path(path_run)
    path_run.mkdir(parents=True, exist_ok=True)
    scenario = Scenario(path_scenario=Path(''), empty=True)
    scenario.path_scenario = Path(path_run)
    print(scen_scenariofuzz.seed_data)
    # Map
    scenario.map = scen_scenariofuzz.town
    # Mission
    _start = {
        "x": scen_scenariofuzz.ego_sp.location.x,
        "y": scen_scenariofuzz.ego_sp.location.y,
        "z": scen_scenariofuzz.ego_sp.location.z,
        "roll": scen_scenariofuzz.ego_sp.rotation.roll,
        "pitch": scen_scenariofuzz.ego_sp.rotation.pitch,
        "yaw": scen_scenariofuzz.ego_sp.rotation.yaw
    }
    _dest = {
        "x": scen_scenariofuzz.ego_wp.location.x,
        "y": scen_scenariofuzz.ego_wp.location.y,
        "z": scen_scenariofuzz.ego_wp.location.z,
        "roll": scen_scenariofuzz.ego_wp.rotation.roll,
        "pitch": scen_scenariofuzz.ego_wp.rotation.pitch,
        "yaw": scen_scenariofuzz.ego_wp.rotation.yaw
    }
    scenario.mission = {
        'start': _start,
        "dest": _dest
    }
    # NPCs
    scenario.npcs = {
        'vehicles': [],
        'pedestrians': []
    }
    for _id, actor in enumerate(scen_scenariofuzz.actors):
        _target = 'vehicles' if actor['type'] == 0 else "pedestrians"
        _bp = "vehicle.bmw.grandtourer" if actor['type'] == 0 else "walker.pedestrian.0001"
        scenario.npcs[_target].append({
            "id": _id,
            "nav_type": actor['nav_type'],
            "bp": _bp,
            "start": {
                "x": actor['spawn_point'][0][0],
                "y": actor['spawn_point'][0][1],
                "z": actor['spawn_point'][0][2],
                "roll": actor['spawn_point'][1][0],
                "pitch": actor['spawn_point'][1][1],
                "yaw": actor['spawn_point'][1][2]
            },
            "dest": {
                "x": actor['dest_point'][0][0],
                "y": actor['dest_point'][0][1],
                "z": actor['dest_point'][0][2],
                "roll": actor['dest_point'][1][0],
                "pitch": actor['dest_point'][1][1],
                "yaw": actor['dest_point'][1][2]
            } if actor['nav_type'] == 1 else None,
            "speed": actor['speed'],
            "maneuvers": actor['maneuvers']
        })
    # Puddles
    for puddle in scen_scenariofuzz.puddles:
        scenario.puddles.append({
            "level": puddle['level'],
            "x": puddle['spawn_point'][0][0],
            "y": puddle['spawn_point'][0][1],
            "z": 0,
            "size_x": puddle['size'][0],
            "size_y": puddle['size'][1],
            "size_z": 1000
        })
    # Weather
    scenario.weather['cloudiness'] = scen_scenariofuzz.weather['cloud']
    scenario.weather['precipitation'] = scen_scenariofuzz.weather['rain']
    scenario.weather['precipitation_deposits'] = scen_scenariofuzz.weather['puddle']
    scenario.weather['wind_intensity'] = scen_scenariofuzz.weather['wind']
    scenario.weather['fog_density'] = scen_scenariofuzz.weather['fog']
    scenario.weather['wetness'] = scen_scenariofuzz.weather['wetness']
    scenario.weather['dust_storm'] = 0
    # Time
    scenario.time['sun_azimuth_angle'] = scen_scenariofuzz.weather['angle']
    scenario.time['sun_altitude_angle'] = scen_scenariofuzz.weather['altitude']

    # Save scenario
    scenario.dump(scenario.path_scenario)
    with (path_run / 'scenario_d.json').open('w') as f:
        json.dump({
            'actors': scen_scenariofuzz.actors,
            'puddles': scen_scenariofuzz.puddles,
            'weather': scen_scenariofuzz.weather
        }, f, indent=4)

    return run(path_run, scenario, ads, timer, id_gpu)


def run_scenariofuzz1(sp,
                      wp,
                      ads,
                      town,
                      id_gpu):
    Config.PATH_ID_FUZZER.touch()
    # print(Config.PATH_ID_FUZZER)
    scenario = Scenario(path_scenario=Path(''), empty=True)
    scenario.mission['start'] = {
        "x": sp.location.x,
        "y": sp.location.y,
        "z": sp.location.z,
        "roll": sp.rotation.roll,
        "pitch": sp.rotation.pitch,
        "yaw": sp.rotation.yaw
    }
    scenario.mission['dest'] = {
        "x": wp.location.x,
        "y": wp.location.y,
        "z": wp.location.z,
        "roll": wp.rotation.roll,
        "pitch": wp.rotation.pitch,
        "yaw": wp.rotation.yaw
    }
    scenario.map = town
    while True:
        chances_run_again = CHANCES_RUN_AGAIN
        # scenario.path_scenario = path_run
        # cc.load_world(scenario.map)
        while True:
            # pid_record = __start_recording(target)
            # pid_record = ads.start_recording()
            if ads.id == 'autoware':
                Config.PATH_LOCK.touch()
            try:
                # timer.start_simulation()
                # Run target ADS
                if not ads.connect(scenario=scenario,
                                   timeout=LOAD_TIMEOUT,
                                   id_gpu=id_gpu):
                    get_logger().info(f"Failed to load {ads.id}.")
                    raise RunNextMutant
                break
            except RunAgain:
                clean_up(ads)
                continue
        # Send routing request
        get_logger().info(f"Send routing request to {ads.id}.")
        # if not __send_routing_request(scenario.mission['dest'], target=target):
        if ads.id == 'autoware':
            res = ads.set_goal(scenario.mission)
        elif ads.id == 'apollo':
            res = ads.set_goal(scenario.mission)
        if not res:
            get_logger().info("Run the next mutant due to the routing failure.")
            chances_run_again += 1
            raise RunNextMutant
        break


def run_samota(path_run,
               scenario,
               ads,
               timer,
               id_gpu) -> Tuple[float, float, float, float, float, float]:
    from fuzz.feedback import get_values
    timer.start_run(path_run.name)
    record = run(path_run, scenario, ads, timer, id_gpu, is_samota=True)
    timer.end_run()
    timer.start_feedback()
    values = get_values(scenario, record)
    timer.end_feedback()
    with (path_run / 'feedback.txt').open('w') as f:
        f.write(','.join(map(str, values)))
    return values


def run_autofuzz(path_run: Path,
                 scenario: Scenario,
                 ads: ADS,
                 timer: Timer,
                 id_gpu: int) -> None:
    # Config.PATH_ID_FUZZER.touch()
    if ads.id == 'apollo':
        cc.reload_world()
    while True:
        # Send routing request
        get_logger().info(f"Send routing request to {ads.id}.")
        # if not __send_routing_request(scenario.mission['dest'], target=target):
        while True:
            if ads.id == 'autoware':
                res = ads.set_goal(scenario.mission)
            elif ads.id == 'apollo':
                res = ads.set_goal(scenario.mission)
            if not res:
                get_logger().info("Routing failure. Trying again.")
                continue
            break
            # raise RunNextMutant
        # Start driving
        timer.start_driving_simulation()
        # Unlock the logging
        if ads.id == 'autoware':
            Config.PATH_LOCK.unlink()
        # __start_driving()
        ads.engage()
        # wait until the driving to be done
        record = __wait_until_driving_done(path_run,
                                           scenario=scenario,
                                           ads=ads,
                                           pid_record=3)#pid_record)
        timer.end_driving_simulation()
        # if is_fp(record, timeout_stalling=Config.TIMEOUT_STALLING):
        #     get_logger().info("False positive is detected")
        #     with (path_run / 'fp').open('w') as f:
        #         f.write('')
        timer.end_simulation()
        return record

# def run_autofuzz(path_run,
#                  scenario,
#                  ads,
#                  timer,
#                  id_gpu) -> Tuple[List[float], Tuple[float, float], str, float]:
# def run_autofuzz(path_seed,
#                  town,
#                  _scenario) -> Tuple[np.ndarray, Tuple[Optional[float], Optional[float]], Optional[str], float]:
#     from fuzz.feedback import estimate_objectives
#     path_seed = Path(path_seed)
#     # Mission
#     seed = list(path_seed.glob(f'{town}_*.json'))[_scenario['id_seed']]
#     scenario = Scenario(seed)
#     # NPC Vehicle
#     _scenario['vehicle_list']
#     # NPC Pedestrian
#     _scenario['pedestrian_list']
#     _weather = WEATHERS[_scenario['weather_index']]
#     # Weather
#     scenario.weather = {
#         'cloudiness': _weather.cloudiness,
#         'precipitation': _weather.precipitation,
#         'precipitation_deposits': _weather.precipitation_deposits,
#         'wind_intensity': _weather.wind_intensity,
#         'fog_density': _weather.fog_density,
#         'wetness': _weather.wetness,
#         'dust_storm': _weather.dust_storm
#     }
#     # Time
#     scenario.time = {
#         'sun_azimuth_angle': _weather.sun_azimuth_angle,
#         'sun_altitude_angle': _weather.sun_altitude_angle,
#     }
#     # Puddle
#     scenario.puddles = [{
#         'level': _scenario['friction'],
#         "x": -10000.0,
#         "y": -10000.0,
#         "z": 0.0,
#         "size_x": 1000000.0,
#         "size_y": 1000000.0,
#         "size_z": 1000000.0,
#     }]
#     # print(_scenario['static_list'][0])
#     print(_scenario['vehicle_list'][0].spawn_transform)
#     print(_scenario['pedestrian_list'][0].spawn_transform)
#     exit()
#     # return ([0, 0, 0, 0], (0, 0), '', 0)
#     return (np.array([3, 1, 20, 7, 7, 2, 0, 1, 0, 0]), (None, None), '', 0)
