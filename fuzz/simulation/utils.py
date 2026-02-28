from enum import IntEnum
import json
import math
from pathlib import Path
import shutil
import time
import weakref
from typing import Any, Dict, List

import networkx as nx
import numpy as np
import carla

from fuzz.ads import ADS, Autoware
from fuzz.carla.utils import get_carla_location, get_carla_rotation, get_carla_vector3D
from fuzz.commons.constants import *
from fuzz.commons.utils import exec_docker_container
from fuzz.data.record import Scenario, Record


def init_logging_dir() -> None:
    clean_logging_dir()
    if not Config.PATH_SIMULATION_LOG.is_dir():
        Config.PATH_SIMULATION_LOG.mkdir(parents=True)
        

def init_log_dir_for_seed(path_log: Path,
                          seed: List[Path]) -> Path:
    if isinstance(path_log, str):
        path_log = Path(path_log)
    p = path_log / seed[0].stem
    p.mkdir(exist_ok=True)
    return p


def clean_logging_dir() -> None:
    for filename in (Config.PATH_SIMULATION / 'images').glob('*.jpg'):
        filename.unlink()
    for filename in Config.PATH_SIMULATION_LOG.glob('*'):
        if filename.is_file():
            filename.unlink()
        else:
            shutil.rmtree(filename)
    for filename in Config.PATH_CARLA_AUTOWARE.glob('*.xml'):
        filename.unlink()


def copy_simulation_result(p: Path,
                           scenario: Scenario,
                           ads: ADS) -> None:
    # Create directory
    p.mkdir(exist_ok=True)
    # Save scenario
    scenario.dump(p)
    # Copy videos and state
    for f in [NAME_VIDEO_FRONT, NAME_VIDEO_TOP, NAME_FILE_STATE]:
        if (Config.PATH_SIMULATION_LOG / f).is_file():
            shutil.copy(Config.PATH_SIMULATION_LOG / f, p / f)
    if ads.id == 'autoware':
        # Copy Autoware log
        res = exec_docker_container(name_container=Config.NAME_CONTAINER_ADS,
                                    cmd=f'bash -c "ls {DIR_HOME}/.ros/log/**/{NAME_FILE_LAUNCH_LOG} "')
        launch_log = sorted(res.split("\n"))[-1]
        if isinstance(ads, Autoware):
            _ = exec_docker_container(name_container=Config.NAME_CONTAINER_ADS,
                                    cmd=f'bash -c "cp {launch_log} {DIR_HOME}/shared/logs/{NAME_FILE_LAUNCH_LOG}"')
        else:   # Old version
            _ = exec_docker_container(name_container=Config.NAME_CONTAINER_ADS,
                                    cmd=f'bash -c "cp {launch_log} {DIR_HOME}/autoware_carla_launch/external/zenoh_carla_bridge/carla_agent/logs/{NAME_FILE_LAUNCH_LOG}"')
        shutil.copy(
            f'{Config.PATH_SIMULATION_LOG}/{NAME_FILE_LAUNCH_LOG}', p / NAME_FILE_LAUNCH_LOG)
    # if any violation is made, save a category of the misbehavior as a file
    any_violation = (Config.PATH_SIMULATION_LOG / NAME_FILE_VIOLATION).is_file()
    if any_violation and (Config.PATH_SIMULATION_LOG / NAME_FILE_VIOLATION).open().read() == 'timeout':
        any_violation = False
    if any_violation:
        shutil.copy(Config.PATH_SIMULATION_LOG / NAME_FILE_VIOLATION,
                    p / NAME_FILE_VIOLATION)
    # if any violation is made, save ros bag file
    if ads.id == 'autoware':
        if any_violation and (Config.PATH_SIMULATION_LOG / NAME_FILE_BAG_COMPRESSED).is_dir():
            shutil.copytree(Config.PATH_SIMULATION_LOG /
                            NAME_FILE_BAG_COMPRESSED, p / NAME_FILE_BAG_COMPRESSED)
            shutil.rmtree(Config.PATH_SIMULATION_LOG / NAME_FILE_BAG_COMPRESSED)
        elif any_violation and (Config.PATH_SIMULATION_LOG / NAME_FILE_BAG).is_dir():
            shutil.copytree(Config.PATH_SIMULATION_LOG / NAME_FILE_BAG, p / NAME_FILE_BAG)
            shutil.rmtree(Config.PATH_SIMULATION_LOG / NAME_FILE_BAG)
    elif ads.id == 'apollo':
        # if any_violation:
        _p = Config.PATH_APOLLO.glob(NAME_FILE_RECORD)
        for __p in _p:
            shutil.copy(__p, p / __p.name)
            os.unlink(__p)

def dist_to_wp(wps: List[carla.Waypoint],
               wp_t: carla.Waypoint) -> float:
    if wps[0] == wp_t:
        return 0.0
    dist = 0.0
    wp_prev = wps[0]
    for wp in wps[1:]:
        dist += wp_prev.transform.location.distance(wp.transform.location)
        if wp == wp_t:
            break
        wp_prev = wp
    return dist


def distance_2d(loc1: carla.Location,
                loc2: carla.Location) -> float:
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)


def is_fp(record: Record,
          timeout_stalling: int) -> bool:
    fp = False
    speeds = [s['speed'] for s in record.state.state]
    # 1. Stalling from the starting point
    if record.state.result['stalling']:
        # Condition 1: The running time is < 23 seconds
        cond1 = len(speeds) < (timeout_stalling + 3) * FPS
        fp = cond1
        if fp:
            return True
    # 2. Collision by NPCs
    if record.state.result['collision'] and record.state.result['collision_with'] is not None:
        _speeds = speeds[-6:-2]
        # Condition 1: NPC hit behind the ego vehicle
        _s = record.state.state[-1]
        cond1 = False
        _dist_min = 10000.0
        p_n = dict()
        p_ego = get_carla_location(_s['point'])
        dir_ego = get_carla_rotation(
            _s['point']
        ).get_forward_vector().make_unit_vector()
        for _n in _s['vehicles'] + _s['pedestrians']:
            p = get_carla_location((_n['point']))
            _dist = p_ego.distance(p)
            if _dist < _dist_min:
                _dist_min = _dist
                p_n = _n['point']
        dir_ego_o = get_carla_vector3D(
            p_n['x'] - p_ego.x,
            p_n['y'] - p_ego.y,
            p_n['z'] - p_ego.z
        ).make_unit_vector()
        angle = math.degrees(dir_ego.get_vector_angle(dir_ego_o))
        # print(angle)
        if 150 < angle < 210:
            return True
        # Condition 2: The ego vehicle can be moving, but should be slow enough
        if len(_speeds) > 0:
            # print(sum(_speeds) / len(_speeds))
            cond2 = (sum(_speeds) / len(_speeds)) < 9.0
        else:
            cond2 = True
        # cond2 = True
        # Condition 3: The NPC should be moving faster than the ego vehicle
        cond3 = False
        _idx = -3 if len(record.state.state) > 3 else len(record.state.state) - 1
        for _v in record.state.state[_idx]['vehicles']:
            # print(_v['speed'])
            if _v['speed'] > _s['speed']:
                cond3 = True
                break
        # fp = cond1 and cond2 and cond3
        fp = cond2 and cond3
        if fp:
            # print(f"fp: {record.path_mb}")
            return True
    # 3. Stalling by obstacle in front
    if record.state.result['stalling']:
        for s in record.state.state[-6:-2]:
            p = s['point']
            dir_ego = get_carla_rotation(
                p
            ).get_forward_vector().make_unit_vector()
            p_ego = get_carla_location(p)
            for _v in s['vehicles'] + s['pedestrians']:
                p_obj = get_carla_location(_v['point'])
                dir_ego_o = get_carla_vector3D(
                    p_obj.x - p_ego.x,
                    p_obj.y - p_ego.y,
                    p_obj.z - p_ego.z
                ).make_unit_vector()
                angle = math.degrees(dir_ego.get_vector_angle(dir_ego_o))
                # Near, front and stopped
                # Condition 1: Near the ego vehicle
                cond1 = p_ego.distance(p_obj) < 15 
                # Condition 2: The object is in front of the ego vehicle
                cond2 = (angle % 360 < 50 or angle % 360 > 310)
                # Condition 3: The object is stopped
                cond3 = _v['speed'] < 1
                fp = cond1 and cond2 and cond3
                if fp:
                    print(f"fp: {record.path_mb}")
                    return True
    return False


class CollisionSensor(object):

    def __init__(self, parent_actor, sensor_name='collision', trans = None):
        self.sensor = None
        self._parent = parent_actor
        self.collided: bool = False
        self.collided_with: str = ''

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        bp.set_attribute('role_name', sensor_name)

        if trans == None:
            trans = carla.Transform()

        self.sensor = world.spawn_actor(
            bp,
            trans,
            attach_to=self._parent
        )

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if event.other_actor.type_id == 'static.road':
            return
        self.collided = True
        self.collided_with = event.other_actor.type_id
        if not self:
            return
        
    def check_collision(self) -> bool:
        return self.collided
    
    def check_collision_with(self) -> str:
        return self.collided_with
        

class LaneInvasionSensor(object):

    def __init__(self, parent_actor, sensor_name='lane_invasion', trans = None):
        self.sensor = None
        self.lines_invaded: List[carla.LaneMarking] = []

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor

            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            bp.set_attribute('role_name', sensor_name)

            if trans == None:
                trans = carla.Transform()

            self.sensor = world.spawn_actor(
                bp,
                trans,
                attach_to=self._parent
            )

            weak_self = weakref.ref(self)
            self.sensor.listen(
                lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lines_invaded = list(
            map(lambda _l: str(_l.type), event.crossed_lane_markings))

    def check_lane_invasion(self) -> bool:
        return len(self.lines_invaded) > 0



def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2

        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


class RoadOption(IntEnum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.

    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


class GlobalRoutePlanner(object):
    """
    This class provides a very high level route plan.
    """

    def __init__(self, wmap, sampling_resolution):
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap
        self._topology = None
        self._graph = None
        self._id_map = None
        self._road_id_to_edge = None

        self._intersection_end_node = -1
        self._previous_decision = RoadOption.VOID

        # Build the graph
        self._build_topology()
        self._build_graph()
        self._find_loose_ends()
        self._lane_change_link()

    def trace_route(self, origin, destination):
        """
        This method returns list of (carla.Waypoint, RoadOption)
        from origin to destination
        """
        route_trace = []
        route = self._path_search(origin, destination)
        current_waypoint = self._wmap.get_waypoint(origin)
        destination_waypoint = self._wmap.get_waypoint(destination)

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            edge = self._graph.edges[route[i], route[i+1]]
            path = []

            if edge['type'] != RoadOption.LANEFOLLOW and edge['type'] != RoadOption.VOID:
                route_trace.append((current_waypoint, road_option))
                exit_wp = edge['exit_waypoint']
                n1, n2 = self._road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]
                next_edge = self._graph.edges[n1, n2]
                if next_edge['path']:
                    closest_index = self._find_closest_in_list(
                        current_waypoint, next_edge['path'])
                    closest_index = min(
                        len(next_edge['path'])-1, closest_index+5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_trace.append((current_waypoint, road_option))

            else:
                path = path + [edge['entry_waypoint']] + \
                    edge['path'] + [edge['exit_waypoint']]
                closest_index = self._find_closest_in_list(
                    current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    if len(route)-i <= 2 and waypoint.transform.location.distance(destination) < 2*self._sampling_resolution:
                        break
                    elif len(route)-i <= 2 and current_waypoint.road_id == destination_waypoint.road_id and current_waypoint.section_id == destination_waypoint.section_id and current_waypoint.lane_id == destination_waypoint.lane_id:
                        destination_index = self._find_closest_in_list(
                            destination_waypoint, path)
                        if closest_index > destination_index:
                            break

        return route_trace

    def _build_topology(self):
        """
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects with the following attributes

        - entry (carla.Waypoint): waypoint of entry point of road segment
        - entryxyz (tuple): (x,y,z) of entry point of road segment
        - exit (carla.Waypoint): waypoint of exit point of road segment
        - exitxyz (tuple): (x,y,z) of exit point of road segment
        - path (list of carla.Waypoint):  list of waypoints between entry to exit, separated by the resolution
        """
        self._topology = []
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round(
                [l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict = dict()
            seg_dict['entry'], seg_dict['exit'] = wp1, wp2
            seg_dict['entryxyz'], seg_dict['exitxyz'] = (
                x1, y1, z1), (x2, y2, z2)
            seg_dict['path'] = []
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:
                    seg_dict['path'].append(w)
                    next_ws = w.next(self._sampling_resolution)
                    if len(next_ws) == 0:
                        break
                    w = next_ws[0]
            else:
                next_wps = wp1.next(self._sampling_resolution)
                if len(next_wps) == 0:
                    continue
                seg_dict['path'].append(next_wps[0])
            self._topology.append(seg_dict)

    def _build_graph(self):
        """
        This function builds a networkx graph representation of topology, creating several class attributes:
        - graph (networkx.DiGraph): networkx graph representing the world map, with:
            Node properties:
                vertex: (x,y,z) position in world map
            Edge properties:
                entry_vector: unit vector along tangent at entry point
                exit_vector: unit vector along tangent at exit point
                net_vector: unit vector of the chord from entry to exit
                intersection: boolean indicating if the edge belongs to an  intersection
        - id_map (dictionary): mapping from (x,y,z) to node id
        - road_id_to_edge (dictionary): map from road id to edge in the graph
        """

        self._graph = nx.DiGraph()
        self._id_map = dict()  # Map with structure {(x,y,z): id, ... }
        # Map with structure {road_id: {lane_id: edge, ... }, ... }
        self._road_id_to_edge = dict()

        for segment in self._topology:
            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in self._id_map:
                    new_id = len(self._id_map)
                    self._id_map[vertex] = new_id
                    self._graph.add_node(new_id, vertex=vertex)
            n1 = self._id_map[entry_xyz]
            n2 = self._id_map[exit_xyz]
            if road_id not in self._road_id_to_edge:
                self._road_id_to_edge[road_id] = dict()
            if section_id not in self._road_id_to_edge[road_id]:
                self._road_id_to_edge[road_id][section_id] = dict()
            self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)

            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()

            # Adding edge with attributes
            self._graph.add_edge(
                n1, n2,
                length=len(path) + 1, path=path,
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=np.array(
                    [entry_carla_vector.x, entry_carla_vector.y, entry_carla_vector.z]),
                exit_vector=np.array(
                    [exit_carla_vector.x, exit_carla_vector.y, exit_carla_vector.z]),
                net_vector=vector(entry_wp.transform.location,
                                  exit_wp.transform.location),
                intersection=intersection, type=RoadOption.LANEFOLLOW)

    def _find_loose_ends(self):
        """
        This method finds road segments that have an unconnected end, and
        adds them to the internal graph representation
        """
        count_loose_ends = 0
        hop_resolution = self._sampling_resolution
        for segment in self._topology:
            end_wp = segment['exit']
            exit_xyz = segment['exitxyz']
            road_id, section_id, lane_id = end_wp.road_id, end_wp.section_id, end_wp.lane_id
            if road_id in self._road_id_to_edge \
                    and section_id in self._road_id_to_edge[road_id] \
                    and lane_id in self._road_id_to_edge[road_id][section_id]:
                pass
            else:
                count_loose_ends += 1
                if road_id not in self._road_id_to_edge:
                    self._road_id_to_edge[road_id] = dict()
                if section_id not in self._road_id_to_edge[road_id]:
                    self._road_id_to_edge[road_id][section_id] = dict()
                n1 = self._id_map[exit_xyz]
                n2 = -1*count_loose_ends
                self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)
                next_wp = end_wp.next(hop_resolution)
                path = []
                while next_wp is not None and next_wp \
                        and next_wp[0].road_id == road_id \
                        and next_wp[0].section_id == section_id \
                        and next_wp[0].lane_id == lane_id:
                    path.append(next_wp[0])
                    next_wp = next_wp[0].next(hop_resolution)
                if path:
                    n2_xyz = (path[-1].transform.location.x,
                              path[-1].transform.location.y,
                              path[-1].transform.location.z)
                    self._graph.add_node(n2, vertex=n2_xyz)
                    self._graph.add_edge(
                        n1, n2,
                        length=len(path) + 1, path=path,
                        entry_waypoint=end_wp, exit_waypoint=path[-1],
                        entry_vector=None, exit_vector=None, net_vector=None,
                        intersection=end_wp.is_junction, type=RoadOption.LANEFOLLOW)

    def _lane_change_link(self):
        """
        This method places zero cost links in the topology graph
        representing availability of lane changes.
        """

        for segment in self._topology:
            left_found, right_found = False, False

            for waypoint in segment['path']:
                if not segment['entry'].is_junction:
                    next_waypoint, next_road_option, next_segment = None, None, None

                    if waypoint.right_lane_marking and waypoint.right_lane_marking.lane_change & carla.LaneChange.Right and not right_found:
                        next_waypoint = waypoint.get_right_lane()
                        if next_waypoint is not None \
                                and next_waypoint.lane_type == carla.LaneType.Driving \
                                and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANERIGHT
                            next_segment = self._localize(
                                next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']
                                                 ], next_segment[0], entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint, intersection=False, exit_vector=None,
                                    path=[], length=0, type=next_road_option, change_waypoint=next_waypoint)
                                right_found = True
                    if waypoint.left_lane_marking and waypoint.left_lane_marking.lane_change & carla.LaneChange.Left and not left_found:
                        next_waypoint = waypoint.get_left_lane()
                        if next_waypoint is not None \
                                and next_waypoint.lane_type == carla.LaneType.Driving \
                                and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANELEFT
                            next_segment = self._localize(
                                next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']
                                                 ], next_segment[0], entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint, intersection=False, exit_vector=None,
                                    path=[], length=0, type=next_road_option, change_waypoint=next_waypoint)
                                left_found = True
                if left_found and right_found:
                    break

    def _localize(self, location):
        """
        This function finds the road segment that a given location
        is part of, returning the edge it belongs to
        """
        waypoint = self._wmap.get_waypoint(location)
        edge = None
        try:
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            pass
        return edge

    def _distance_heuristic(self, n1, n2):
        """
        Distance heuristic calculator for path searching
        in self._graph
        """
        l1 = np.array(self._graph.nodes[n1]['vertex'])
        l2 = np.array(self._graph.nodes[n2]['vertex'])
        return np.linalg.norm(l1-l2)

    def _path_search(self, origin, destination):
        """
        This function finds the shortest path connecting origin and destination
        using A* search with distance heuristic.
        origin      :   carla.Location object of start position
        destination :   carla.Location object of of end position
        return      :   path as list of node ids (as int) of the graph self._graph
        connecting origin and destination
        """
        start, end = self._localize(origin), self._localize(destination)

        route = nx.astar_path(
            self._graph, source=start[0], target=end[0],
            heuristic=self._distance_heuristic, weight='length')
        route.append(end[1])
        return route

    def _successive_last_intersection_edge(self, index, route):
        """
        This method returns the last successive intersection edge
        from a starting index on the route.
        This helps moving past tiny intersection edges to calculate
        proper turn decisions.
        """

        last_intersection_edge = None
        last_node = None
        for node1, node2 in [(route[i], route[i+1]) for i in range(index, len(route)-1)]:
            candidate_edge = self._graph.edges[node1, node2]
            if node1 == route[index]:
                last_intersection_edge = candidate_edge
            if candidate_edge['type'] == RoadOption.LANEFOLLOW and candidate_edge['intersection']:
                last_intersection_edge = candidate_edge
                last_node = node2
            else:
                break

        return last_node, last_intersection_edge

    def _turn_decision(self, index, route, threshold=math.radians(35)):
        """
        This method returns the turn decision (RoadOption) for pair of edges
        around current index of route list
        """

        decision = None
        previous_node = route[index-1]
        current_node = route[index]
        next_node = route[index+1]
        next_edge = self._graph.edges[current_node, next_node]
        if index > 0:
            if self._previous_decision != RoadOption.VOID \
                    and self._intersection_end_node > 0 \
                    and self._intersection_end_node != previous_node \
                    and next_edge['type'] == RoadOption.LANEFOLLOW \
                    and next_edge['intersection']:
                decision = self._previous_decision
            else:
                self._intersection_end_node = -1
                current_edge = self._graph.edges[previous_node, current_node]
                calculate_turn = current_edge['type'] == RoadOption.LANEFOLLOW and not current_edge[
                    'intersection'] and next_edge['type'] == RoadOption.LANEFOLLOW and next_edge['intersection']
                if calculate_turn:
                    last_node, tail_edge = self._successive_last_intersection_edge(
                        index, route)
                    self._intersection_end_node = last_node
                    if tail_edge is not None:
                        next_edge = tail_edge
                    cv, nv = current_edge['exit_vector'], next_edge['exit_vector']
                    if cv is None or nv is None:
                        return next_edge['type']
                    cross_list = []
                    for neighbor in self._graph.successors(current_node):
                        select_edge = self._graph.edges[current_node, neighbor]
                        if select_edge['type'] == RoadOption.LANEFOLLOW:
                            if neighbor != route[index+1]:
                                sv = select_edge['net_vector']
                                cross_list.append(np.cross(cv, sv)[2])
                    next_cross = np.cross(cv, nv)[2]
                    deviation = math.acos(np.clip(
                        np.dot(cv, nv)/(np.linalg.norm(cv)*np.linalg.norm(nv)), -1.0, 1.0))
                    if not cross_list:
                        cross_list.append(0)
                    if deviation < threshold:
                        decision = RoadOption.STRAIGHT
                    elif cross_list and next_cross < min(cross_list):
                        decision = RoadOption.LEFT
                    elif cross_list and next_cross > max(cross_list):
                        decision = RoadOption.RIGHT
                    elif next_cross < 0:
                        decision = RoadOption.LEFT
                    elif next_cross > 0:
                        decision = RoadOption.RIGHT
                else:
                    decision = next_edge['type']

        else:
            decision = next_edge['type']

        self._previous_decision = decision
        return decision

    def _find_closest_in_list(self, current_waypoint, waypoint_list):
        min_distance = float('inf')
        closest_index = -1
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(
                current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index
    

class DummyWorld:
    def __init__(self, world_obj, player):
        self.world = world_obj
        self.player = player


class State:

    def __init__(self, path_scenario: Path, _map: str) -> None:
        # Start time of the simulation
        self.time_start: float = time.time()
        # Path to save
        self.log_dir: Path = path_scenario
        # State
        self.state: List[Dict[str, Any]] = []
        # Map
        self.map: str = _map
        # Bounding boxes
        self.bounding_boxes: Dict[str, Dict[str, Dict[str, float]]] = {}
        # Violation informations
        self.result: Dict[str, Any] = {
            'collision': False,
            'collision_with': None,
            'stalling': False,
            'speeding': False,
            'spawn_timeout': False,
            'start_timeout': False,
            'normal': False,
            'timeout': False
        }
        # End time of the simulation
        self.time_end: float = 0.0

    def __str__(self) -> str:
        return str(self.__dict__)

    def update(self, state_t: Dict[str, Any]) -> None:
        self.state.append(state_t)

    def any_violation(self) -> bool:
        return any(self.result.values())

    def dump(self) -> None:
        if not (self.log_dir / 'state.json').is_file():
            self.time_end = time.time()
            with (self.log_dir / 'state.json').open('w') as f:
                del self.log_dir
                json.dump(self.__dict__, f)


def angle_from_center_view_fov(target, ego, fov=90):
    target_location = target.get_location()
    ego_location = ego.get_location()
    ego_orientation = ego.get_transform().rotation.yaw

    # hack: adjust to the front central camera's location
    # this needs to be changed when the camera's location / fov change
    dx = 1.3 * np.cos(np.deg2rad(ego_orientation - 90))

    ego_location = ego.get_location()
    ego_x = ego_location.x + dx
    ego_y = ego_location.y

    target_vector = np.array([target_location.x - ego_x, target_location.y - ego_y])
    norm_target = np.linalg.norm(target_vector)

    if norm_target < 0.001:
        return 0

    forward_vector = np.array(
        [
            math.cos(math.radians(ego_orientation)),
            math.sin(math.radians(ego_orientation)),
        ]
    )

    try:
        d_angle = np.abs(
            math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))
        )
    except:
        print(
            "\n" * 3,
            "np.dot(forward_vector, target_vector)",
            np.dot(forward_vector, target_vector),
            norm_target,
            "\n" * 3,
        )
        d_angle = 0
    # d_angle_norm == 0 when target within fov
    d_angle_norm = np.clip((d_angle - fov / 2) / (180 - fov / 2), 0, 1)

    return d_angle_norm


def get_bbox(vehicle):
    current_tra = vehicle.get_transform()
    current_loc = current_tra.location

    heading_vec = current_tra.get_forward_vector()
    heading_vec.z = 0
    heading_vec = heading_vec / math.sqrt(
        math.pow(heading_vec.x, 2) + math.pow(heading_vec.y, 2)
    )
    perpendicular_vec = carla.Vector3D(-heading_vec.y, heading_vec.x, 0)

    extent = vehicle.bounding_box.extent
    x_boundary_vector = heading_vec * extent.x
    y_boundary_vector = perpendicular_vec * extent.y

    bbox = [
        current_loc + carla.Location(x_boundary_vector - y_boundary_vector),
        current_loc + carla.Location(x_boundary_vector + y_boundary_vector),
        current_loc + carla.Location(-1 * x_boundary_vector - y_boundary_vector),
        current_loc + carla.Location(-1 * x_boundary_vector + y_boundary_vector),
    ]

    return bbox


def norm_2d(loc_1, loc_2):
    return np.sqrt((loc_1.x - loc_2.x) ** 2 + (loc_1.y - loc_2.y) ** 2)
