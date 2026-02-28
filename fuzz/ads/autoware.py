import math
import os
import re
import time
import xml.etree.ElementTree as ET

# from scipy.spatial.transform import Rotation as R

from fuzz.ads.ads import ADS
from fuzz.ads.utils import call_ros2, euler_from_quaternion, quaternion_from_euler
from fuzz.carla.utils import get_carla_transform
from fuzz.commons.exceptions import TryMutationAgain, PlanningAgain
from fuzz.commons.constants import *
from fuzz.commons.utils import exec_docker_container, get_logger
from fuzz.data import Scenario

import timeout_decorator
from timeout_decorator.timeout_decorator import TimeoutError


class Autoware(ADS):

    def __init__(self, _map: str, version: str='20241212') -> None:
        super().__init__()
        self.id: str = 'autoware'
        self.bp: str = 'vehicle.toyota.prius'
        self.map: str = _map
        self.version: str = version
        self.__sec: float = 0.0
        self.__nanosec: float = 0.0

    @timeout_decorator.timeout(180)
    def __wait_to_be_loaded(self, timeout: int) -> bool:
        cmd = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '/system/component_state_monitor/component/autonomous/localization'
        ])
        cond = bool(call_ros2(
            cmd=cmd, matching_str='available: true', timeout=timeout))
        return cond

    def __check_pose_initialization(self,
                                    scenario: Scenario,
                                    timeout: int) -> bool:
        t = time.time()
        # 1. Check nearest voxel transformation likelihood
        cmd = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '--field',
            'data',
            '/localization/pose_estimator/nearest_voxel_transformation_likelihood'
        ])
        output = str(call_ros2(cmd=cmd, matching_str='', return_output=True))
        try:
            value = float(output.split('\n')[0])
            cond1 = value >= NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD
        except Exception as _:
            return False
        # time.sleep(10)
        # return True
        get_logger().info(
            f"NVTL: {value:.2f} Threshold: {NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD}")
        if not cond1:
            get_logger().info(
                f"Nearest voxel transformation likelihood is too low. Run again.")
            return False
        cmd = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '--field',
            'pose.pose',
            '/localization/pose_with_covariance'
        ])
        output = str(call_ros2(cmd=cmd, matching_str='', return_output=True))
        try:
            _pos, _ori = output.split(
                'position:\n')[-1].split('orientation:\n')
            # Check position
            x, y, _ = re.findall(r'[-+]?(?:\d*\.*\d+)', _pos)
            _x = scenario.mission['start']['x']
            _y = scenario.mission['start']['y']
            # Check orientation
            _, _, z, w = re.findall(r'-?\d*\.?\d+(?:[eE][-+]?\d+)?', _ori)
            _, _, _z, _w = quaternion_from_euler(
                0.0, 0.0, math.radians(-scenario.mission['start']['yaw']))
            offset_x = abs(float(x) - _x)
            # Note that the y-axis should be flipped!
            offset_y = abs(-float(y) - _y)
            offset_z = abs(float(z)-float(_z))
            offset_w = abs(float(w)-float(_w))
            cond2 = offset_x < 1 and offset_y < 1 and offset_z < 0.01 and offset_w < 0.01
        except Exception as _:
            return False
        get_logger().info(
            f"Localization-Simulator Initial Pose & Orientation Offset: ({offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f}, {offset_w:.2f})")
        return True

    def connect(self,
                scenario: Scenario,
                timeout: int,
                id_gpu: int,
                use_traffic_manager: bool = False,
                is_samota: bool = False) -> bool:
        get_logger().info("Loading Autoware...")
        _args = {
            'host': Config.HOST_CARLA,
            'port': Config.PORT_CARLA,
            'timeout': TIMEOUT_CARLA,
            'spawn_point': ','.join([
                str(scenario.mission['start']['x']),
                str(scenario.mission['start']['y']),
                str(scenario.mission['start']['z']),
                str(scenario.mission['start']['roll']),
                str(scenario.mission['start']['pitch']),
                str(scenario.mission['start']['yaw'])
            ]),
            'carla_map': scenario.map,
            'fixed_delta_seconds': f"{1 / FPS:.2f}",
            'max_real_delta_seconds': f"{1 / FPS:.2f}",
            'use_traffic_manager': use_traffic_manager,
            'port_tm': Config.PORT_TM
        }
        if use_traffic_manager:
            _args['use_traffic_manager'] = True
        file_bridge = 'autoware_carla_interface.launch.xml'
        _launch = ET.parse(Config.PATH_DATA / file_bridge)
        for _arg in _launch.getroot().find('group').iter('arg'):
            _d = _arg.attrib
            if _d['name'] in _args.keys():
                _d['default'] = str(object=_args[_d['name']])
        with (Config.PATH_CARLA_AUTOWARE / f'_{file_bridge}').open('wb') as f:
            _launch.write(f, encoding='utf-8', xml_declaration=True)
        #  > /dev/null 2>&1
        file_xml = f'_e2e_simulator_{self.version}.launch.xml'
        cmd = 'bash -c' + ' "' + \
            ' && '.join([
                'source scripts/env.sh',
                ' '.join([
                    f'CUDA_VISIBLE_DEVICES={id_gpu} ros2 launch autoware_launch {file_xml}',
                    f'map_path:=$HOME/carla_map/{scenario.map}',
                    'vehicle_model:=sample_vehicle',
                    'sensor_model:=awsim_sensor_kit',
                    'simulator_type:=carla',
                    f'carla_map:={scenario.map} &'
                ])
            ]) + '"'
        # Load Autoware
        try:
            exec_docker_container(
                name_container=Config.NAME_CONTAINER_ADS, cmd=cmd)
            if not self.__wait_to_be_loaded(timeout):
                return False
        except TimeoutError:
            get_logger().info("Timeout while loading")
            return False
        # Check if pose is initialized well.
        if not self.__check_pose_initialization(scenario, timeout):
            return False
        return True

    def __is_routing_valid(self,
                           timeout: int) -> bool:
        cmd_check = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '/api/operation_mode/state'
        ])
        out = call_ros2(
            cmd=cmd_check, matching_str='is_autonomous_mode_available: true', wait_for_output=False)
        if out:
            get_logger().info(f'Routing request done!')
            return True
        else:
            return False

    def set_goal(self,
                 mission: Dict[str, Dict[str, float]],
                 timeout: int = 3,
                 is_planning: bool = False) -> bool:
        dest = mission['dest']
        q = quaternion_from_euler(0.0, 0.0, math.radians(-dest['yaw']))
        ps = [dest['x'], -dest['y'], dest['z'], q[0], q[1], q[2], q[3]]
        cmd = " ".join([
            'ros2',
            'topic',
            'pub',
            '--once',
            "/planning/mission_planning/goal",
            "geometry_msgs/msg/PoseStamped",
            f"'{{ header: {{ stamp: now, frame_id: 'map'}}, pose: {{ position: {{x: {ps[0]}, y: {ps[1]}, z: {ps[2]} }}, orientation: {{x: {ps[3]}, y: {ps[4]}, z: {ps[5]}, w: {ps[6]} }} }} }}'",
        ])
        if is_planning:
            _ = call_ros2(cmd=cmd, matching_str='', wait_for_output=False, name_container=Config.NAME_CONTAINER_ADS_PLANNER)
        else:
            _ = call_ros2(cmd=cmd, matching_str='', wait_for_output=False)
            time.sleep(10)
            return self.__is_routing_valid(timeout)

    def engage(self) -> None:
        # Ros command to start driving
        cmd = " ".join([
            'ros2',
            'topic',
            'pub',
            '--once',
            "/autoware/engage",
            "autoware_auto_vehicle_msgs/msg/Engage" if self.version == '20240523' else "autoware_vehicle_msgs/msg/Engage",
            "'{engage: True}'",
            "-1"
        ])
        call_ros2(cmd=cmd, matching_str='')

    def start_recording(self) -> int:
        # For recording ros messages
        cmd = " ".join([
            'ros2',
            'bag',
            'record',
            "-a",
            '-o',
            f"{DIR_HOME}/shared/logs/bag",
            "&"
        ])
        call_ros2(cmd=cmd, timeout_cmd=-1, matching_str='')
        ps_all = exec_docker_container(
            name_container=Config.NAME_CONTAINER_ADS,
            cmd='ps -ef'
        )
        pid_record = int([line for line in ps_all.splitlines()
                         if cmd in line].pop(0).split()[1])
        return pid_record

    def finish_recording(self,
                         pid_record: int) -> None:
        # Ctrl+C to stop recording
        cmd_python = f"import os; import signal; os.kill({pid_record}, signal.SIGINT)"
        cmd = f'bash -c "python3 -c {cmd_python}"'
        exec_docker_container(
            name_container=Config.NAME_CONTAINER_ADS,
            cmd=cmd
        )
        get_logger().info("Saving ros2bag for replaying.")
        # Post-process ros2bag file
        cmd = " ".join([
            'ros2',
            'bag',
            'reindex',
            f"{DIR_HOME}/shared/logs/bag",
        ])
        call_ros2(cmd=cmd, timeout_cmd=-1, matching_str='')
        get_logger().info("Compressing ros2bag file.")
        # Compress ros2bag file
        cmd = " ".join([
            'ros2',
            'bag',
            'convert',
            '--input',
            f"{DIR_HOME}/shared/logs/bag",
            '--output-options',
            f'{DIR_HOME}/data/output_options.yaml'
        ])
        call_ros2(cmd=cmd, timeout_cmd=-1, matching_str='')

    def restart(self) -> None:
        os.system(f"docker kill {Config.NAME_CONTAINER_ADS} > /dev/null 2>&1")
        os.system(
            f"docker restart {Config.NAME_CONTAINER_ADS} > /dev/null 2>&1")

    def __intialize(self,
                    start: Dict[str, float]) -> None:
        q = quaternion_from_euler(0.0, 0.0, math.radians(-start['yaw']))
        ps = [start['x'], -start['y'], start['z'], q[0], q[1], q[2], q[3]]
        cmd = " ".join([
            'ros2',
            'topic',
            'pub',
            '--once',
            '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            f"'{{ header: {{ stamp: now, frame_id: 'map'}}, pose: {{ pose: {{ position: {{x: {ps[0]}, y: {ps[1]}, z: {ps[2]} }}, orientation: {{x: {ps[3]}, y: {ps[4]}, z: {ps[5]}, w: {ps[6]} }} }} }} }}'",
            "-1"
        ])
        call_ros2(cmd=cmd, matching_str='', name_container=Config.NAME_CONTAINER_ADS_PLANNER)

    def plan_trajectory(self,
                        mission: Dict[str, Dict[str, float]]) -> List[carla.Transform]:
        start = mission['start']
        # Set starting point
        self.__intialize(start)
        # Set goal point
        self.set_goal(mission=mission, is_planning=True)
        # Check if routing was successful
        cmd = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '/api/routing/state'
        ])
        res = call_ros2(cmd=cmd, matching_str='', return_output=True, name_container=Config.NAME_CONTAINER_ADS_PLANNER)
        ''' `res` is as below
        stamp:
            sec: 1740289719
            nanosec: 535594927
        state: 2
        '''
        try:
            _res = str(res).split('\n')     # ['stamp:', '  sec: 1740238389', '  nanosec: 523597216', 'state: 1', '---', '']
            _sec = int(_res[1].split(':')[-1].strip())
            _nanosec = int(_res[2].split(':')[-1].strip())
            if _sec == self.__sec and _nanosec == self.__nanosec:
                # Planner emits the same last state
                get_logger().info("Planner stopped. Restarting...")
                self.restart_planner()
                raise PlanningAgain("Planner stopped.")
                # raise TryMutationAgain
            self.__sec = _sec
            self.__nanosec = _nanosec
            _state = _res[3].split(':')[-1].strip()
            if _state != '2':   # Routing failed
                raise PlanningAgain("Routing failed")
                # raise TryMutationAgain
        except Exception as _:
            if res == False:
                self.restart_planner()
            raise PlanningAgain("Exception")
        # Get planned route
        cmd = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '--field',
            'points',
            '/planning/scenario_planning/lane_driving/behavior_planning/path'
        ])
        res = call_ros2(cmd=cmd, matching_str='', return_output=True, name_container=Config.NAME_CONTAINER_ADS_PLANNER, timeout=20)
        if res == False:
            self.restart_planner()
            raise TryMutationAgain
        pattern = r'Point\(x=(-?\d*\.?\d+(?:[eE][-+]?\d+)?), y=(-?\d*\.?\d+(?:[eE][-+]?\d+)?), z=(-?\d*\.?\d+(?:[eE][-+]?\d+)?)\), orientation=geometry_msgs.msg.Quaternion\(x=(-?\d*\.?\d+(?:[eE][-+]?\d+)?), y=(-?\d*\.?\d+(?:[eE][-+]?\d+)?), z=(-?\d*\.?\d+(?:[eE][-+]?\d+)?), w=(-?\d*\.?\d+(?:[eE][-+]?\d+)?)\)'
        positions = re.findall(pattern, str(res))#[20:]
        points = []
        # Convert data string into carla.Transform
        for position in positions:
            x, y, z = position[:3]
            _q = position[3:]
            q = euler_from_quaternion(*map(float, _q))
            points.append(carla.Transform(
                location=carla.Location(x=float(x), y=-float(y), z=float(z)),   # y should be negated
                rotation=carla.Rotation(
                    roll=math.degrees(q[0]),
                    pitch=math.degrees(q[1]),
                    yaw=-math.degrees(q[2]) # Should be negated?
                )
            ))
        # Remove first n points before the start point
        start = get_carla_transform(mission['start'])
        for i in range(len(points)):
            # Ignore z-axis
            dist = ((points[i].location.x - start.location.x)**2 + (points[i].location.y - start.location.y)**2)**0.5
            if dist < 1.1:
                points = points[i+1:]
                break
        dest = get_carla_transform(mission['dest'])
        if points[-1].location.distance(dest.location) > 5.0 or len(points) < 3:
            # Planning has some errors: the end of the trajectory is not the destination
            raise PlanningAgain("End point is not the destination.")
        return points

    def restart_planner(self, wait: bool=True) -> None:
        os.system(f"docker kill {Config.NAME_CONTAINER_ADS_PLANNER} > /dev/null 2>&1")
        os.system(
            f"docker restart {Config.NAME_CONTAINER_ADS_PLANNER} > /dev/null 2>&1")
        self.start_planner(wait)

    def __wait_to_be_loaded_planner(self, timeout: int) -> bool:
        cmd = " ".join([
            'ros2',
            'topic',
            'echo',
            '--once',
            '/system/component_state_monitor/component/autonomous/localization'
        ])
        cond = bool(call_ros2(
            cmd=cmd, matching_str='available: false', timeout=timeout, name_container=Config.NAME_CONTAINER_ADS_PLANNER))
        return cond

    def start_planner(self, wait: bool=True) -> None:
        if wait:
            get_logger().info("Loading Autoware Planner...")
        res = exec_docker_container(
            name_container=Config.NAME_CONTAINER_ADS_PLANNER,
            cmd=f'bash -c "source scripts/env.sh && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/carla_map/{self.map} vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit &"'
        )
        if wait:
            self.__wait_to_be_loaded_planner(120)
            get_logger().info("Autoware Planner is loaded.")
