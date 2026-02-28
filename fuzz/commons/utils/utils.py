from copy import deepcopy
import json
import logging
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, Iterator, Tuple

from fuzz.commons.constants import *

import docker
import numpy as np


logger = logging.getLogger()
logging.basicConfig(
    format='[%(asctime)s] [%(levelname)s]: %(message)s',
    level=logging.INFO,
    stream=sys.stdout,
    datefmt='%Y/%m/%d %I:%M:%S %p'
)


def get_logger() -> logging.Logger:
    global logger
    return logger


def exec_docker_container(name_container: str, **kwargs) -> str:
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(name_container)
        if 'detach' in kwargs and kwargs['detach']:
            return container.exec_run(**kwargs).output
        else:
            return container.exec_run(**kwargs).output.decode('utf-8')
    except Exception as e:
        import traceback
        traceback.print_exc()
        get_logger().error(
            f"Error while execute command in the docker container: {e}")
        exit(1)


def run_docker_container(kill_previous_run: bool = False, **kwargs) -> bool:
    already_running = False
    try:
        name_container = kwargs['name']
        docker_client = docker.from_env()
        # Check if container is already running
        if name_container in [c.name for c in docker_client.containers.list()]:
            already_running = True
            container_carla = docker_client.containers.get(name_container)
            if kill_previous_run:
                get_logger().info(
                    f"   - Container '{name_container}' is already running. I'll stop it and run another one.")
                container_carla.stop()
                while container_carla.status != 'removing':
                    container_carla.reload()
                    time.sleep(1)
            else:
                get_logger().info(
                    f"   - Container '{name_container}' is already running")
                return already_running
        # Check if there's any stopped container
        try:
            container_carla = docker_client.containers.get(name_container)
            get_logger().info(
                f"   - '{name_container}' is not running(stopped before). I'll start the container")
            container_carla.start()
        except Exception as _:
            get_logger().info(f"   - Start the container '{name_container}'")
            container_carla = docker_client.containers.run(**kwargs)
        os.system(
            f'docker update --restart unless-stopped {name_container} > /dev/null 2>&1')
    except Exception as e:
        get_logger().error(
            f"Error while starting carla simulator with docker: {e}")
        exit(1)
    return already_running


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    # Copied from
    # https://github.com/davheld/tf/blob/master/src/tf/transformations.py#L1100

    _AXES2TUPLE: Dict[str, Tuple[int, int, int, int]] = {
        'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
        'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
        'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
        'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
        'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
        'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
        'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
        'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)
    }

    _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

    _NEXT_AXIS: List[int] = [1, 2, 0, 1]

    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = np.empty((4, ), dtype=np.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion


def run_carla_with_docker(version: str,
                          port: int,
                          offscreen: bool,
                          id_gpu: int,
                          kill_previous_run: bool = False) -> None:
    cmd = f'/bin/bash ./CarlaUE4.sh -nosound -quality-level=Epic -fps 20 -carla-port={port}'
    if offscreen:
        get_logger().info(f"Run carla with docker container without screen.")
        # ' -graphicsadapter=1 '
        cmd += f"-e NVIDIA_VISIBLE_DEVICES={id_gpu} --gpus 'device={id_gpu}' -RenderOffScreen -graphicsadapter={id_gpu}"
    else:
        get_logger().info(
            f"Run carla with docker container - DISPLAY: {DISPLAY}")
        if DISPLAY is None:
            get_logger().error("Please set environment variable 'DISPLAY' before running Carla simulator. ex) export DISPLAY=:1; xhost +")
            exit(1)
    # _ports[str(Config.PORT_TM)] = Config.PORT_TM
    already_running = run_docker_container(
        kill_previous_run=kill_previous_run,
        image=f'carlasim/carla:{version}',
        command=cmd,
        detach=True,
        name=Config.NAME_CONTAINER_CARLA,
        privileged=True,
        # network_mode='host',
        runtime="nvidia",
        environment={
            "DISPLAY": DISPLAY
        },
        volumes={
            "/tmp/.X11-unix": {
                "bind": "/tmp/.X11-unix",
                "mode": "rw"
            },
            str(Path(__file__).resolve().parent.parent.parent.parent / 'data'): {
                "bind": "/home/carla/Import",
                "mode": "rw"
            }
        },
        ports={str(port): port for port in range(port, port+3)}
    )
    if not already_running:
        get_logger().info("Carla simulator is just started. Waiting 10 seconds for the simulator to be loaded.")
        time.sleep(10)


def get_carla_host_ip() -> str:
    try:
        return exec_docker_container(
            Config.NAME_CONTAINER_CARLA, cmd='hostname -I').strip()
    except Exception as e:
        get_logger().error(
            f"Error while getting carla host ip address: {e}")
        exit(1)


def is_container_running(name_container: str) -> bool:
    docker_client = docker.from_env()
    try:
        container = docker_client.containers.get(name_container)
        return container.status == 'running'
    except docker.errors.NotFound:
        return False


def construct_seed(path_seed: Path, dry_run: bool) -> Iterator[List[Path]]:
    __MAPS = [
        'Town01',
        'Town02',
        'Town03',
        'Town04',
        'Town05',
        'Town06',
        'Town07',
        'Town10HD',
    ]
    # if dry_run:
    #     yield list(path_seed.glob("*.json"))
    for __map in __MAPS:
        seeds = sorted(path_seed.glob(f'{__map}*.json'))
        if len(seeds) == 0:
            get_logger().error(
                f"Seed files for map {__map} are not found. Skip to the next map.")
            continue
        yield seeds
        # for p in sorted(path_seed.glob(f'{__map}*.json')):
        #     yield p


class Timer:

    def __init__(self,
                 path_log: Path,
                 pruning: bool,
                 timeout: int = TIMEOUT_FUZZING,
                 timeout_per_map: int = TIMEOUT) -> None:
        self.path_timer: Path = path_log / 'time.json'
        if (self.path_timer).is_file():
            with (self.path_timer).open('r') as f:
                self.__logs = json.load(f)
                self.time_ran: float = self.__logs['time_ran']
                self.time_start: float = self.__logs['time_start']
        else:
            self.__logs: Dict[str, Any] = dict()
            self.time_ran: float = 0.0
            self.time_start: float = time.time()
        self.timeout: int = timeout
        self.__seed_curr: str = ''
        self.__run_curr: str = ''
        self.pruning: bool = pruning
        self.timeout_per_map: int = timeout_per_map
        # For scenariofuzz
        self.__campaign_curr: int = 1
        self.__cycle_curr: int = 1
        self.__round_curr: int = 1

    def dump(self) -> None:
        path_timer = deepcopy(self.path_timer)
        del self.path_timer
        self.__logs['time_ran'] = time.time() - self.time_start #self.time_ran
        self.__logs['time_start'] = self.time_start
        with path_timer.open('w') as f:
            json.dump(self.__logs, f, indent=4)
        self.path_timer = path_timer

    def start_main(self) -> None:
        self.time_ran = 0.0

    def start_fuzz(self) -> None:
        if 'start' not in self.__logs.keys():
            self.__logs['start'] = time.time()

    def start_seed(self, seed_curr: str) -> None:
        self.__seed_curr = seed_curr
        if self.__seed_curr not in self.__logs.keys():
            self.__logs[self.__seed_curr] = dict()
        if 'start' not in self.__logs[self.__seed_curr].keys():
            self.__logs[self.__seed_curr]['start'] = time.time()

    def start_run(self, run_curr: str) -> None:
        self.__run_curr = run_curr
        self.__logs[self.__seed_curr][self.__run_curr] = dict()
        self.__logs[self.__seed_curr][self.__run_curr]['start'] = time.time()

    def start_pruning(self) -> None:
        self.__logs[self.__seed_curr][self.__run_curr]['pruning'] = dict()
        self.__logs[self.__seed_curr][self.__run_curr]['pruning']['start'] = time.time()

    def end_pruning(self, num_pruned: int) -> None:
        self.__logs[self.__seed_curr][self.__run_curr]['pruning']['num_pruned'] = num_pruned
        self.__logs[self.__seed_curr][self.__run_curr]['pruning']['end'] = time.time()

    def start_mutation(self) -> None:
        # if 'mutation' not in self.__logs[self.__seed_curr][self.__run_curr].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['mutation'] = dict()
        self.__logs[self.__seed_curr][self.__run_curr]['mutation']['start'] = time.time()

    def end_mutation(self) -> None:
        # if 'end' not in self.__logs[self.__seed_curr][self.__run_curr]['mutation'].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['mutation']['end'] = time.time()

    def start_simulation(self) -> None:
        # if 'simulation' not in self.__logs[self.__seed_curr][self.__run_curr].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['simulation'] = dict()
        self.__logs[self.__seed_curr][self.__run_curr]['simulation']['start'] = time.time()

    def start_driving_simulation(self) -> None:
        # if 'start_driving' not in self.__logs[self.__seed_curr][self.__run_curr]['simulation'].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['simulation']['start_driving'] = time.time()

    def end_driving_simulation(self) -> None:
        # if 'end_driving' not in self.__logs[self.__seed_curr][self.__run_curr]['simulation'].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['simulation']['end_driving'] = time.time()

    def end_simulation(self) -> None:
        # if 'end' not in self.__logs[self.__seed_curr][self.__run_curr]['simulation'].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['simulation']['end'] = time.time()

    def start_feedback(self) -> None:
        # if 'feedback' not in self.__logs[self.__seed_curr][self.__run_curr].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['feedback'] = dict()
        self.__logs[self.__seed_curr][self.__run_curr]['feedback']['start'] = time.time()

    def end_feedback(self) -> None:
        # if 'end' not in self.__logs[self.__seed_curr][self.__run_curr]['feedback'].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['feedback']['end'] = time.time()

    def end_run(self) -> None:
        # if 'end' not in self.__logs[self.__seed_curr][self.__run_curr].keys():
        self.__logs[self.__seed_curr][self.__run_curr]['end'] = time.time()
        # self.time_ran += (self.__logs[self.__seed_curr][self.__run_curr]
        #                   ['end'] - self.__logs[self.__seed_curr][self.__run_curr]['start'])
        self.time_ran = time.time() - self.time_start

    def end_seed(self) -> None:
        if 'end' not in self.__logs[self.__seed_curr].keys():
            self.__logs[self.__seed_curr]['end'] = time.time()
        # self.time_ran += (self.__logs[self.__seed_curr]['end'] - self.__logs[self.__seed_curr]['start'])

    def start_train(self) -> None:
        self.__logs[self.__seed_curr][self.__run_curr]['train'] = dict()
        self.__logs[self.__seed_curr][self.__run_curr]['train']['start'] = time.time()

    def end_train(self) -> None:
        self.__logs[self.__seed_curr][self.__run_curr]['train']['end'] = time.time()

    def is_timeout(self) -> bool:
        return time.time() - self.time_start > self.timeout
