import json
import math
import time
from typing import Any, Tuple, Union
import websocket

import numpy as np

from fuzz.commons.constants import *
from fuzz.commons.exceptions import RunNextMutant, ROS2Timeout
from fuzz.commons.utils import exec_docker_container, get_logger


def call_ros2(cmd: str,
              matching_str: str,
              timeout: int = 120,
              timeout_cmd: int = 10,
              wait_for_output: bool = True,
              return_output: bool = False,
              name_container=Config.NAME_CONTAINER_ADS,
              old: bool = False) -> Union[str, bool]:
    t = time.time()
    while True:
        if timeout_cmd > 0:
            if old:
                command = f'bash -c "cd autoware_carla_launch && source env.sh && cd .. && timeout {timeout_cmd} {cmd}"'
            else:
                command = f'bash -c "source scripts/env.sh && timeout {timeout_cmd} {cmd}"'
        else:
            if old:
                command = f'bash -c "cd autoware_carla_launch && source env.sh && cd .. &&  {cmd}"'
            else:
                command = f'bash -c "source scripts/env.sh && {cmd}"'
        output = exec_docker_container(
            name_container=name_container,
            cmd=command
        )
        if (Config.PATH_SIMULATION_LOG / 'init_error').is_file():
            raise RunNextMutant
        if time.time() - t > timeout:
            return False
            # raise ROS2Timeout
        if 'xmlrpc.client.Fault' in output:
            return False
        if 'Traceback ' in output:
            continue
        if output == '':
            continue
        if return_output:
            return output
        if 'rclpy.executors.ExternalShutdownException' in output:
            return False
        if output != '' and matching_str in output:
            return True
        if not wait_for_output:
            return False
        time.sleep(2)


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


def euler_from_quaternion(x, y, z, w):
        # from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class Connection:

    def __init__(self,
                 host: str,
                 port: str) -> None:
        # Reference: https://github.com/lgsvl/PythonAPI/blob/9bc29c9aee5e98d4d99e6a410c04b153b8f7feef/lgsvl/dreamview/dreamview.py
        self.host: str = host
        self.port: str = port

    def try_connect(self) -> None:
        url: str = f"ws://{self.host}:{self.port}/websocket"
        try:
            self.__ws: websocket.Websocket = websocket.create_connection(url)
        except Exception as _:
            import traceback
            traceback.print_exc()
            get_logger().critical(
                f"Cannot connect to dreamview at https://{self.host}:{self.port}. Please check if Apollo is running.")
            exit(1)

    def send(self, msg: Union[str, Dict]) -> None:
        if isinstance(msg, dict):
            msg = json.dumps(msg)
        self.__ws.send(msg)

    def recv(self) -> Dict[str, Any]:
        return json.loads(self.__ws.recv())
