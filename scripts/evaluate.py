import argparse
from copy import deepcopy
from collections import defaultdict
from datetime import datetime
from itertools import chain, combinations
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from matplotlib import pyplot as plt

from fuzz.behavior import Sequence, get_driving_pattern_sequences
from fuzz.commons.constants import *
from fuzz.data import Record, Scenario
from fuzz.exp import Exp

import numpy as np
import pandas as pd
from scipy.stats import mannwhitneyu


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, required=True)
    parser.add_argument('--timeout', type=float, default=1000)
    parser.add_argument('--check-behavior-again', '-c', action='store_true')
    args = parser.parse_args()
    return args


def a12(a: List[float], b: List[float]) -> float:
    A = np.asarray(list(a), float).ravel()
    B = np.asarray(list(b), float).ravel()
    if A.size == 0 or B.size == 0:
        raise ValueError("Empty sample.")
    gt = (A[:, None] > B[None, :]).sum()
    eq = (A[:, None] == B[None, :]).sum()
    return float((gt + 0.5 * eq) / (A.size * B.size))


def pairwise_mwu_vd_two_sided(
    data: Dict[str, List[int]],
    pairs: Optional[List[Tuple[str, str]]] = None,
) -> pd.DataFrame:
    keys = sorted(data)
    arr = {k: np.asarray(list(data[k]), float).ravel() for k in keys}
    if any(v.size == 0 for v in arr.values()):
        raise ValueError("Empty list for at least one technique.")

    if pairs is None:
        pairs = [(keys[i], keys[j]) for i in range(len(keys)) for j in range(i + 1, len(keys))]

    rows = []
    for A, B in pairs:
        if A == 'prunario':
            u, p = mannwhitneyu(arr[A], arr[B], alternative="two-sided", method="auto")
            rows.append(
                dict(A=A, B=B, nA=arr[A].size, nB=arr[B].size, U=float(u), p=float(p), A12=a12(arr[A], arr[B]))
            )
    return pd.DataFrame(rows).sort_values(["p", "A", "B"]).reset_index(drop=True)


def pooled_and_by_town(
    data_by_town: Dict[str, Dict[str, List[int]]],
    pairs: Optional[List[Tuple[str, str]]] = None,
) -> Tuple[pd.DataFrame, Dict[str, pd.DataFrame]]:
    pooled: Dict[str, List[int]] = {}
    for techmap in data_by_town.values():
        for tech, vals in techmap.items():
            pooled.setdefault(tech, []).extend(list(vals))

    pooled_df = pairwise_mwu_vd_two_sided(pooled, pairs)
    per_town = {town: pairwise_mwu_vd_two_sided(techmap, pairs) for town, techmap in data_by_town.items()}
    return pooled_df, per_town


# def __aggregate_violations(log_timess: List[List[float]], log_mbss: List[List[int]]) -> Tuple[List[float], List[int]]:
def __aggregate_violations(log_mbss: List[List[Tuple[float, int]]], timeout: float) -> Tuple[List[float], List[float]]:
    log_times_all = [0.0]
    log_mbs_all = [0]
    mbs_curr = log_mbs_all[-1]
    while any([len(log_mbs) > 0 for log_mbs in log_mbss]):
        _min = min([log_mbs[0][0] for log_mbs in log_mbss if len(log_mbs) > 0])
        log_times_all.append(_min)
        for i, log_mbs in enumerate(log_mbss):
            if len(log_mbs) <= 0:
                continue
            if log_mbs[0][0] == _min:
                mbs_curr += 1
                log_mbs_all.append(mbs_curr)
                # print(log_mbss[i])
                # log_mbs_all.append(sum([log_mbs[0][1] for log_mbs in log_mbss]))
                # log_timess[i] = log_times[1:]
                log_mbss[i] = log_mbss[i][1:]
                # print(log_mbss[i], log_mbs_all[-1])
                # exit()
                break
    # print(log_times_all, log_mbs_all)
    log_mbs_all = [_s / len(log_mbss) for _s in log_mbs_all]
    log_times_all.append(timeout)
    log_mbs_all.append(log_mbs_all[-1])
    return log_times_all, log_mbs_all


def evaluate_online(p: Path,
                    timeout: float,
                    town: str,
                    check_behavior_again: bool = False) -> Tuple[Dict[str, Any], List[int], int, int, Exp, Set[Sequence], Dict[str, int], Dict[str, int], int, List[Any]]:#, float]:
    path_time = p.parent / 'time.json'
    if 'drivefuzz' in str(p) or 'samota' in str(p):
        path_time = p / 'time.json'
    with path_time.open() as f:
        times = json.load(f)
    time_start = times['start']
    exp = Exp(Path(''))
    cnt_all = 0
    cnt_uniq = 0
    cnt_inter = 0
    for_unique = []
    # cc.load_world(town)
    time_all = 0.0
    log_times = {
        'run_total': [0.0],
        'pruning': 0.0,
        'num_pruned': 0,
        'simulation': 0.0,
        'unique_time': 0.0
    }
    cnt_v = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    cnt_v_all = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    cnt_v_real_all = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    cnt_mbs = 0
    log_mbs = []
    bs_v_unique = Exp(Path('')) #set()
    logs = sorted(p.glob(r'[0-9]*_[0-9]*'),
                  key=lambda _p: list(map(int, str(_p.name).split('_'))))
    if 'drivefuzz' in str(p):
        logs = []
        for seed in sorted(p.glob(f'{town}_*')):
            logs += sorted(seed.glob(r'[0-9]*_[0-9]*'),
                           key=lambda _p: list(map(int, str(_p.name).split('_'))))

    # for run in tqdm(logs):
    for run in logs:
        if (run / 'routing_failure').is_file():
            continue
        if 'drivefuzz' in str(p):
            time_seed = times[run.parent.name]
        elif 'samota' in str(p):
            time_seed = times[town]
        else:
            time_seed = times[p.name.split('_')[0]]
        if run.name not in time_seed:
            # print(f"Run {run} not in time_seed")
            continue
        time_run = time_seed[run.name]
        # if time_start == 0.0:
        #     time_start = time_run['start']
        # if 'end' in time_run:
        #     time_all = (time_run['end'] - time_start) / 3600
        # else:
        #     time_all = (time_run['simulation']['end'] - time_start) / 3600
        if 'end' in time_run:
            # print(time_real, time_all)
            # if time_real > 12:
            #     print(12)
            if 'baseline' in str(p) or 'pruning' in str(p):
                time_all = (time_run['end'] - time_start) / 3600
            else:
                # time_real = (time_run['end'] - time_start) / 3600
                time_all += (time_run['end'] - time_run['start']) / 3600
        else:
            if 'baseline' in str(p) or 'pruning' in str(p):
                time_all = (time_run['feedback']['end'] - time_start) / 3600
            else:
                time_all += (time_run['simulation']
                             ['end'] - time_run['start']) / 3600
        log_times['run_total'].append(time_all)
        # time_pruning += (time_run['pruning']['end'] - time_run['pruning']['start'])
        # times_pruning.append(time_pruning)
        # nums_pruning.append(time_run['pruning']['num_pruned'])
        if time_all > timeout:
            # print(f'12: {run}')
            log_times['run_total'][-1] = timeout  # 12.0
            # log_mbs.append(log_mbs[-1])
            # print("HERERE")
            # print(log_times)
            # print(log_mbs)
            return log_times, log_mbs, cnt_uniq, exp, bs_v_unique, get_unique_violations([bs_v_unique]), cnt_v, cnt_v_all, cnt_inter, cnt_v_real_all, for_unique

        scenario = Scenario(run)
        if not (run / NAME_FILE_BEHAVIOR).exists():
            # scenario = Scenario(run)
            r = Record(run)
            bs = get_driving_pattern_sequences(scenario.mission, r, ego_only=True)[0]['ego']
            # bs = get_driving_pattern_sequences(r, ego_only=True)[0][0]
            bs.dump(run, name=NAME_FILE_BEHAVIOR)
        else:
            bs = Sequence.load(run, name=NAME_FILE_BEHAVIOR)
        if (run / NAME_FILE_BEHAVIOR_HAT).is_file():
            bs_hat = Sequence.load(run, name=NAME_FILE_BEHAVIOR_HAT)
        else:
            bs_hat = bs
        if 'collision' in bs.violation:
            bs.violation = 'collision'
        do_ignore = exp.redundant(bs)
        if bs.any_interaction():
            cnt_inter += 1
        if not do_ignore:
            cnt_uniq += 1
            log_times['unique_time'] += (time_run['simulation']['end_driving'] -
                                         time_run['simulation']['start_driving']) / 3600
            # if bs.any_interaction():
            #     cnt_inter += 1
            # print(run)
            # print(bs, bs.violation)
        else:
            # print(bs, run, bs.violation)
            if not (run / 'ignore').exists():
                (run / 'ignore').touch()
        log_times['simulation'] += (time_run['simulation']['end_driving'] -
                                    time_run['simulation']['start_driving']) / 3600
        if 'pruning' in time_run:
            log_times['pruning'] += (time_run['pruning']['end'] -
                                    time_run['pruning']['start']) / 3600
        exp.append(run, scenario, bs, bs_hat)
        for_unique.append((
            run,
            (time_run['simulation']['end_driving'] - time_run['simulation']['start_driving']) / 3600,
            bs,
            (run / 'fp').exists()
        ))
        # if not do_ignore and (run / 'fp').exists():
        #     print(run)
        if not (run / 'fp').exists():
            bs_v_unique.append(run, scenario, bs, bs)
        if (run / 'violation.txt').exists() and not do_ignore and not (run / 'fp').exists():
            cnt_mbs += 1
            # bs_v_unique.append(run, scenario, bs, bs)
            log_mbs.append((log_times['run_total'][-1], cnt_mbs))
            __violation = (run / 'violation.txt').open().read().strip()
            if 'collision' in __violation:
                cnt_v['collision'] += 1
            elif 'stalling' in __violation:
                cnt_v['stalling'] += 1
            elif 'speeding' in __violation:
                cnt_v['speeding'] += 1
            elif 'lane_invasion' in __violation:
                cnt_v['lane_invasion'] += 1
            else:
                cnt_v['other'] += 1
        if (run / 'violation.txt').exists() and not (run / 'fp').exists():
            # bs_v_unique.add(bs)
            __violation = (run / 'violation.txt').open().read().strip()
            if 'collision' in __violation:
                cnt_v_all['collision'] += 1
            elif 'stalling' in __violation:
                cnt_v_all['stalling'] += 1
            elif 'speeding' in __violation:
                cnt_v_all['speeding'] += 1
            elif 'lane_invasion' in __violation:
                cnt_v_all['lane_invasion'] += 1
            else:
                cnt_v_all['other'] += 1
        if (run / 'violation.txt').exists():
            __violation = (run / 'violation.txt').open().read().strip()
            if 'collision' in __violation:
                cnt_v_real_all['collision'] += 1
            elif 'stalling' in __violation:
                cnt_v_real_all['stalling'] += 1
            elif 'speeding' in __violation:
                cnt_v_real_all['speeding'] += 1
            elif 'lane_invasion' in __violation:
                cnt_v_real_all['lane_invasion'] += 1
            else:
                cnt_v_real_all['other'] += 1
        cnt_all += 1
    return log_times, log_mbs, cnt_uniq, exp, bs_v_unique, get_unique_violations([bs_v_unique]), cnt_v, cnt_v_all, cnt_inter, cnt_v_real_all, for_unique


def to_record(p: Path) -> Record:
    return Record(p, other=True)



def evaluate_scenariofuzz(p: Path,
                          timeout: int,
                          check_behavior_again: bool = False) -> Tuple[Dict[str, Any], List[int], int, int, Exp, Set[Sequence], Dict[str, int], Dict[str, int], int, List[Any]]:#, float]:
    # cc.load_world(town)
    p_eval = p / 'for_eval'
    exp = Exp(Path(''))
    if not p_eval.is_dir():
        p_eval.mkdir()
    path_time = p.parent / 'time.json'
    with path_time.open() as f:
        times = json.load(f)
    log_times = {
        'run_total': [0.0],
        'pruning': 0.0,
        'num_pruned': 0,
        'simulation': 0.0,
        'unique_time': 0.0
    }
    cnt_v = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    cnt_v_all = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    cnt_v_real_all = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    for_unique = []
    # log_times = [0.0]
    # log_mbs = [0]
    log_mbs = []
    time_start = 0.0
    time_all = 0.0
    cnt_mbs = 0
    cnt_uniq = 0
    cnt_inter = 0
    cnt_all = 0
    offset = 0.0
    _prev = 0.0
    bs_v_unique = Exp(Path('')) #set()
    # log_times_unique_all = 0.0
    for cnt_campaign, v in times.items():
        if cnt_campaign == 'time_ran':
            continue
        for i in range(1, 4):
            for j in range(1, 4):
                # time_all += (v2[str(i)]['end'] - v2[str(i)]['start']) / 3600
                _id = f"{cnt_campaign}_{i}_{j}"
                p_behavior = p_eval / f"{_id}_{NAME_FILE_BEHAVIOR}"
                p_ignore = p_eval / f"{_id}_ignore"
                p_fp = p_eval / f"{_id}_fp"
                p_violation = p_eval / f"{_id}_violation.txt"
                _log = list((p / 'queue').glob(f'{_id}_*.json'))

                if len(_log) > 0:
                    if time_start == 0.0:
                        time_start = _log[0].stem.split('_')[-1]
                    _diff = (float(_log[0].stem.split(
                        '_')[-1]) - float(time_start)) / 3600 - offset
                    if _diff - time_all > 1:
                        offset += (_diff - time_all)
                        _diff = (float(_log[0].stem.split(
                            '_')[-1]) - float(time_start)) / 3600 - offset
                        # time_start = float(_log[0].stem.split('_')[-1])
                    # else:
                    #     offset = _diff
                    time_all = _diff
                    if time_all > timeout:
                        # log_times.append(12.0)
                        # log_mbs.append(log_mbs[-1])
                        return log_times, log_mbs, cnt_uniq, exp, bs_v_unique, get_unique_violations([bs_v_unique]), cnt_v, cnt_v_all, cnt_inter, cnt_v_real_all, for_unique
                    r = to_record(_log[0])
                    with _log[0].open() as f:
                        # if is_fp(r, timeout_stalling=20):
                        #     if not p_fp.is_file():
                        #         p_fp.touch()
                        # Check duplication
                        js = json.load(f)
                        if not p_behavior.is_file():
                            # bs = get_driving_pattern_sequence(r, p_violation.name)
                            mission = {
                                'start': {
                                    'x': js['ego_car']['sp_x'],
                                    'y': js['ego_car']['sp_y'],
                                    'z': js['ego_car']['sp_z'],
                                    'roll': js['ego_car']['sp_roll'],
                                    'pitch': js['ego_car']['sp_pitch'],
                                    'yaw': js['ego_car']['sp_yaw']
                                },
                                'dest': {
                                    'x': js['ego_car']['dp_x'],
                                    'y': js['ego_car']['dp_y'],
                                    'z': js['ego_car']['dp_z'],
                                    'roll': 0.0,
                                    'pitch': 0.0,
                                    'yaw': 0.0
                                }
                            }
                            bs = get_driving_pattern_sequences(mission, r, ego_only=True)[0]['ego']
                            bs.dump(path=p_eval, name=p_behavior.name)
                        else:
                            bs = Sequence.load(p_eval, p_behavior.name)
                        if 'collision' in bs.violation:
                            bs.violation = 'collision'
                        do_ignore = exp.redundant(bs)
                        _s_u = datetime.strptime(
                            js['exec_scenario_finish_time'], '%Y-%m-%d %H:%M:%S')
                        _e_u = datetime.strptime(
                            js['exec_end_time'], '%Y-%m-%d %H:%M:%S')
                        if not do_ignore:
                            # print(bs, bs.violation, _log[0])
                            cnt_uniq += 1
                            log_times['unique_time'] += (_e_u -
                                                         _s_u).total_seconds() / 3600
                            if bs.any_interaction():
                                cnt_inter += 1
                            # log_times_unique_all += (_e_u -
                            #                          _s_u).total_seconds() / 3600
                        else:
                            if not p_ignore.is_file():
                                p_ignore.touch()
                        # if not do_ignore and p_fp.is_file():
                        #     print(_log[0])
                        log_times['simulation'] += (_e_u - _s_u).total_seconds() / 3600
                        # if js['events']['stuck']:
                        #     print(_log[0])
                        any_violation = js['events']['crash'] or js['events']['stuck'] or js['events']['speeding'] or js['events']['lane_invasion']
                        # if not do_ignore:
                        #     print(bs, any_violation, _log[0])
                        exp.append(Path(_id), Scenario(Path(), empty=True), bs, bs)
                        for_unique.append((
                            Path(_id),
                            (_e_u - _s_u).total_seconds() / 3600,
                            bs,
                            p_fp.is_file()
                        ))
                        if not p_fp.is_file():
                            bs_v_unique.append(Path(_id), Scenario(Path(), empty=True), bs, bs)
                        if any_violation and not p_fp.is_file() and not do_ignore: #p_ignore.is_file():
                            # print(bs, bs.violation, _log[0])
                            cnt_mbs += 1
                            # bs_v_unique.add(bs)
                            # bs_v_unique.append(Path(_id), Scenario(Path(), empty=True), bs, bs)
                            log_mbs.append((time_all, cnt_mbs))
                            if js['events']['crash']:
                                cnt_v['collision'] += 1
                            elif js['events']['stuck']:
                                cnt_v['stalling'] += 1
                            elif js['events']['speeding']:
                                cnt_v['speeding'] += 1
                            elif js['events']['lane_invasion']:
                                cnt_v['lane_invasion'] += 1
                            else:
                                cnt_v['other'] += 1
                        if any_violation and not p_fp.is_file():
                            # bs_v_unique.add(bs)
                            if js['events']['crash']:
                                cnt_v_all['collision'] += 1
                            elif js['events']['stuck']:
                                cnt_v_all['stalling'] += 1
                            elif js['events']['speeding']:
                                cnt_v_all['speeding'] += 1
                            elif js['events']['lane_invasion']:
                                cnt_v_all['lane_invasion'] += 1
                            else:
                                cnt_v_all['other'] += 1
                        if any_violation:
                            if js['events']['crash']:
                                cnt_v_real_all['collision'] += 1
                            elif js['events']['stuck']:
                                cnt_v_real_all['stalling'] += 1
                            elif js['events']['speeding']:
                                cnt_v_real_all['speeding'] += 1
                            elif js['events']['lane_invasion']:
                                cnt_v_real_all['lane_invasion'] += 1
                            else:
                                cnt_v_real_all['other'] += 1
                        # if any_violation and not p_fp.is_file() and not do_ignore: #p_ignore.is_file():
                        #     cw = js['events']['crash_with']
                        #     if 'walker' in cw:
                        #         cw = 'walker'
                        #     elif 'vehicle' in cw:
                        #         cw = 'vehicle'
                        #     else:
                        #         cw = 'other'
                        #     print(f"{_id}: {bs}, {'fp' if p_fp.is_file() else ('stalling' if js['events']['stuck'] else f'collision({cw})')}, {_log[0]}")
                    log_times['run_total'].append(time_all)
                    cnt_all += 1
    # print(log_times, log_mbs)
    # log_times.append(12.0)
    # log_mbs.append(log_mbs[-1])
    return log_times, log_mbs, cnt_uniq, exp, bs_v_unique, get_unique_violations([bs_v_unique]), cnt_v, cnt_v_all, cnt_inter, cnt_v_real_all, for_unique
    # print(v2['end'] - v2['start'])
    # for cnt_cycle in
    # for run in sorted((p / 'queue').glob(r'[0-9]*_[0-9]*.json'), key=lambda _p: list(map(int, str(_p.name).split('_')[:3]))):
    #     with run.open() as f:
    #         js = json.load(f)
    #         # print(js.keys())
    #         print(f"Cycle~mutstart: {(parse(js['cycle_start_time']) - parse(js['mutation_start_time']))}")
    #         print(f"Mutation~runend: {(parse(js['mutation_start_time']) - parse(js['mutation_start_time']))}")
    #         print(parse(js['campaign_start_time']))
    #         # print(datetime(js['campaign_start_time']))
    #         exit()


def get_unique_violations(exps: List[Exp]) -> Set[Sequence]:
    return

    def is_set_unique(set_v: Set[Sequence]) -> bool:
        for v1 in set_v:
            for v2 in set_v:
                if v1 == v2:
                    continue
                if v1.redundant(v2):
                    return False
        return True
    v_unique = set()
    # Construct set
    for exp in exps:
        for _, bs, _ in exp.get_experienced():
            v_unique.add(bs)
    # Check powerset
    v_unique_poset = set(chain.from_iterable(combinations(v_unique, r) for r in range(len(v_unique)+1)))
    # Leave unique subsets only
    v_unique_poset_unique = set()
    for v in v_unique_poset:
        if len(v) <= 1:
            v_unique_poset_unique.add(v)
        else:
            if is_set_unique(v):
                v_unique_poset_unique.add(v)
    return max(v_unique_poset_unique, key=lambda _v: len(_v))


def collect_unique_violations(exp: Exp) -> Set[Sequence]:
    v_unique = set()
    bss = list(_bs for _, _bs, _ in exp.get_experienced() if _bs.violation != '')
    for i, _bs in enumerate(bss):
        # bs_str = str(_bs) + _bs.violation
        # if bs_str in v_unique:
        #     continue
        # v_unique.add(_bs)
        # '''
        _bss_diff = deepcopy(bss[i:])
        _bss_diff.remove(_bs)
        if any(_bs.redundant(_bs_exp) for _bs_exp in _bss_diff):
            continue
        v_unique.add(_bs)
        # '''
    # print(f"Unique here: {len(v_unique)}")
    # for _bs in v_unique:
    #     print(_bs, _bs.violation)
    # exit()
    return v_unique


def get_uniques(for_unique: List[Tuple[Path, float, Sequence, bool]]) -> Tuple[int, float]:
    uniques = []
    for _id1, _time1, bs1, _fp1 in for_unique:
        for _id2, _time2, bs2, fp2 in for_unique:
            if _id1 == _id2:
                continue
            if bs1.redundant(bs2):
                break
        else:
            uniques.append((_id1, _time1, bs1, _fp1))
    time_unique = sum([_time for _id, _time, bs, _fp in uniques])
    return len(uniques), time_unique


def get_unique_v(for_unique: List[Tuple[Path, float, Sequence, bool]]) -> Tuple[Dict[str, int], Dict[str, int], Dict[str, int]]:
    uniques = []
    for _id1, _time1, bs1, _fp1 in for_unique:
        for _id2, _time2, bs2, fp2 in for_unique:
            if _id1 == _id2:
                continue
            if bs1.redundant(bs2):
                break
        else:
            uniques.append((_id1, _time1, bs1, _fp1))
    scenarios_v = [(_id, _time, bs, _fp) for _id, _time, bs, _fp in uniques if bs.violation != '' and not _fp]
    stats = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    for _id, _time, bs, _fp in scenarios_v:
        if _fp or bs.violation == '':
            continue
        if 'collision' in bs.violation:
            stats['collision'] += 1
        elif 'stalling' in bs.violation:
            stats['stalling'] += 1
        elif 'speeding' in bs.violation:
            stats['speeding'] += 1
        elif 'lane_invasion' in bs.violation:
            stats['lane_invasion'] += 1
        else:
            stats['other'] += 1
    stats_dedup = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    for _id, _time, bs, _fp in for_unique:
        if _fp or bs.violation == '':
            continue
        if 'collision' in bs.violation:
            stats_dedup['collision'] += 1
        elif 'stalling' in bs.violation:
            stats_dedup['stalling'] += 1
        elif 'speeding' in bs.violation:
            stats_dedup['speeding'] += 1
        elif 'lane_invasion' in bs.violation:
            stats_dedup['lane_invasion'] += 1
        else:
            stats_dedup['other'] += 1
    stats_all = {
        'collision': 0,
        'stalling': 0,
        'speeding': 0,
        'lane_invasion': 0,
        'other': 0
    }
    for _id, _time, bs, _fp in for_unique:
        if bs.violation == '':
            continue
        if 'collision' in bs.violation:
            stats_all['collision'] += 1
        elif 'stalling' in bs.violation:
            stats_all['stalling'] += 1
        elif 'speeding' in bs.violation:
            stats_all['speeding'] += 1
        elif 'lane_invasion' in bs.violation:
            stats_all['lane_invasion'] += 1
        else:
            stats_all['other'] += 1
    return stats, stats_dedup, stats_all


args = parse_args()


_paths = {
    "Town01": {
        "prunario": [
            'prunario_Town01_1/Town01_1',
            'prunario_Town01_2/Town01_1',
            'prunario_Town01_3/Town01_1',
        ],
        "scenariofuzz": [
            "scenariofuzz_Town01_1/autoware",
            "scenariofuzz_Town01_2/autoware",
            "scenariofuzz_Town01_3/autoware",
        ],
        "drivefuzz": [
            "drivefuzz_Town01_1",
            "drivefuzz_Town01_2",
            "drivefuzz_Town01_3"
        ],
        "samota": [
            'samota_Town01_1',
            'samota_Town01_2',
            'samota_Town01_3'
        ],
        'field': [
            'field_Town01_1/Town01_1',
            'field_Town01_2/Town01_1',
            'field_Town01_3/Town01_1'
        ],
        'basic': [
            'basic_Town01_1/Town01_1',
            'basic_Town01_2/Town01_1',
            'basic_Town01_3/Town01_1'
        ]
    },
    "Town03": {
        "prunario": [
            'prunario_Town03_1/Town03_1',
            'prunario_Town03_2/Town03_1',
            'prunario_Town03_3/Town03_1'
        ],
        "scenariofuzz": [
            "scenariofuzz_Town03_1/autoware",
            "scenariofuzz_Town03_2/autoware",
            "scenariofuzz_Town03_3/autoware"
        ],
        "drivefuzz": [
            "drivefuzz_Town03_1",
            "drivefuzz_Town03_2",
            "drivefuzz_Town03_3",
        ],
        "samota": [
            'samota_Town03_1',
            'samota_Town03_2',
            'samota_Town03_3'
        ],
        'field': [
            'field_Town03_1/Town03_1',
            'field_Town03_2/Town03_1',
            'field_Town03_3/Town03_1'
        ],
        'basic': [
            'basic_Town03_1/Town03_1',
            'basic_Town03_2/Town03_1',
            'basic_Town03_3/Town03_1'
        ]
    },
    "Town04": {
        "prunario": [
            'prunario_Town04_1/Town04_11',
            'prunario_Town04_2/Town04_11',
            'prunario_Town04_3/Town04_11'
        ],
        "scenariofuzz": [
            "scenariofuzz_Town04_1/autoware",
            "scenariofuzz_Town04_2/autoware",
            "scenariofuzz_Town04_3/autoware",
        ],
        "drivefuzz": [
            "drivefuzz_Town04_1",
            "drivefuzz_Town04_2",
            "drivefuzz_Town04_3"
        ],
        "samota": [
            'samota_Town04_1',
            'samota_Town04_2',
            'samota_Town04_3'
        ],
        'field': [
            'field_Town04_1/Town04_11',
            'field_Town04_2/Town04_11',
            'field_Town04_3/Town04_11'
        ],
        'basic': [
            'basic_Town04_1/Town04_11',
            'basic_Town04_2/Town04_11',
            'basic_Town04_3/Town04_11'
        ]
    },
    "Town05": {
        "prunario": [
            'prunario_Town05_1/Town05_11',
            'prunario_Town05_2/Town05_11',
            'prunario_Town05_3/Town05_11'
        ],
        "scenariofuzz": [
            "scenariofuzz_Town05_1/autoware",
            "scenariofuzz_Town05_2/autoware",
            "scenariofuzz_Town05_3/autoware",
        ],
        "drivefuzz": [
            "drivefuzz_Town05_1",
            "drivefuzz_Town05_2",
            "drivefuzz_Town05_3"
        ],
        "samota": [
            'samota_Town05_1',
            'samota_Town05_2',
            'samota_Town05_3'
        ],
        'field': [
            'field_Town05_1/Town05_11',
            'field_Town05_2/Town05_11',
            'field_Town05_3/Town05_11'
        ],
        'basic': [
            'basic_Town05_1/Town05_11',
            'basic_Town05_2/Town05_11',
            'basic_Town05_3/Town05_11'
        ]
    }
}

colors = {
    'prunario': 'b',
    'drivefuzz': 'g',
    'scenariofuzz': 'y',
    'samota': 'purple'
}

exp_total = {
    'prunario': [],
    'scenariofuzz': [],
    'drivefuzz': [],
    'samota': [],
    'field': [],
    'basic': [],
}

paths = {}
path_root = Path(args.path)
for _map, v in _paths.items():
    paths[_map] = {}
    for fuzzer, res in v.items():
        paths[_map][fuzzer] = [path_root / r for r in res]
# print(paths)
execs_all = {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []}
uniqs_all = {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []}
exect_all = {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []}
uniqt_all = {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []}
for_stats_all = {
    'Town01': {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []},
    'Town03': {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []},
    'Town04': {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []},
    'Town05': {'prunario': [], 'scenariofuzz': [], 'drivefuzz': [], 'samota': [], 'field': [], 'basic': []}
}

print("###### TABLE 2 ######")

for town in paths.keys():
    print(f"{town}:")
    for k in paths[town].keys():
        # if 'baseline' in paths[town]:
        paths_baseline = paths[town][k]
        log_timess = []
        log_mbss = []
        avg_all_uniq_baseline = 0
        avg_all_all_baseline = 0
        avg_mbs_baseline = 0
        avg_times_unique_all_baseline = 0.0
        avg_times_pruning_all = 0.0
        avg_times_simulation_all = 0.0
        bs_v_unique_all = []
        v_unique = set()
        v_unique_all = Exp(Path(''))
        # cnt_unique_all = []
        cnt_inter_all = 0
        cnt_v_all = {
            'collision': 0,
            'stalling': 0,
            'speeding': 0,
            'lane_invasion': 0,
            'other': 0
        }
        cnt_v_total_all = {
            'collision': 0,
            'stalling': 0,
            'speeding': 0,
            'lane_invasion': 0,
            'other': 0
        }
        cnt_v_total_all_real = {
            'collision': 0,
            'stalling': 0,
            'speeding': 0,
            'lane_invasion': 0,
            'other': 0
        }
        avg_uniqs = 0
        avg_uniqt = 0
        for_unique_all = []
        for path_baseline in paths_baseline:
            times_baseline, mbs_baseline, cnt_uniq_baseline, exp, bs_v_unique, bs_v_unique_unique, cnt_v, cnt_v_total, cnt_inter, cnt_v_total_real, for_unique = evaluate_online(
                path_baseline, args.timeout, town, args.check_behavior_again) if k != 'scenariofuzz' else evaluate_scenariofuzz(path_baseline, args.timeout, args.check_behavior_again)
            avg_all_uniq_baseline += cnt_uniq_baseline
            cnt_inter_all += cnt_inter
            avg_all_all_baseline += len(exp)
            avg_mbs_baseline += mbs_baseline[-1][1] if len(
                mbs_baseline) > 0 else 0
            log_timess.append(times_baseline['run_total'])
            log_mbss.append(mbs_baseline)
            avg_times_unique_all_baseline += times_baseline['unique_time'] #times_unique_all_baseline
            avg_times_pruning_all += times_baseline['pruning']
            avg_times_simulation_all += times_baseline['simulation']
            bs_v_unique_all.append(bs_v_unique)
            for _k, _v in cnt_v_total.items():
                cnt_v_total_all[_k] += _v
            for _k, _v in cnt_v_total_real.items():
                cnt_v_total_all_real[_k] += _v
            v_unique = collect_unique_violations(bs_v_unique)
            for_stats_all[town][k].append(len(v_unique))
            for_unique_all += for_unique
            exp_total[k] += for_unique
            uniqs, uniqt = get_uniques(for_unique)

            avg_uniqs += uniqs
            avg_uniqt += uniqt

            for _bs in v_unique:
                if v_unique_all.redundant(_bs):
                    continue
                v_unique_all.append(Path(''), Scenario(Path(''), empty=True), _bs, _bs)
        log_times_all_baseline, log_mbs_all_baseline = __aggregate_violations(
            log_mbss, args.timeout)
        # plt.plot(log_times_all_baseline, log_mbs_all_baseline,
        #          color=colors[k], label=k.lower(), linestyle='-', marker='s')
        
        
        avg_uniqs = avg_uniqs / len(paths_baseline)
        avg_uniqt = avg_uniqt / len(paths_baseline)
        time_sim_avg = avg_times_simulation_all / avg_all_all_baseline
        avg_all_all_baseline = avg_all_all_baseline / len(paths_baseline)
        avg_times_simulation_all = avg_times_simulation_all / len(paths_baseline)
        stats_uniq, stats_dedup, stats_all = get_unique_v(for_unique_all)
        execs_all[k].append(avg_all_all_baseline)
        uniqs_all[k].append(avg_uniqs)
        exect_all[k].append(avg_times_simulation_all*60)
        uniqt_all[k].append(avg_uniqt*60)

        
        h_t, m_t = divmod(avg_times_simulation_all * 60, 60)
        h_u, m_u = divmod(avg_uniqt * 60, 60)
        h_p, m_p = divmod(avg_times_pruning_all * 60, 60)
        _res = collect_unique_violations(v_unique_all)
        for _bs in _res:
            if 'collision' in _bs.violation:
                cnt_v_all['collision'] += 1
            elif 'stalling' in _bs.violation:
                cnt_v_all['stalling'] += 1
            elif 'speeding' in _bs.violation:
                cnt_v_all['speeding'] += 1
            elif 'lane_invasion' in _bs.violation:
                cnt_v_all['lane_invasion'] += 1
            else:
                cnt_v_all['other'] += 1
        
        if k in ['field', 'basic']:
            continue
        print(f"\t{k:20s}: Total ({sum(stats_dedup.values())}) -\tC: {stats_dedup['collision']},\tS: {stats_dedup['stalling']},\tLI: {stats_dedup['lane_invasion']},\tSp: {stats_dedup['speeding']},\tO: {stats_dedup['other']} (# FP: {sum(stats_all.values()) - sum(stats_dedup.values())})")

pooled_df, per_town_dfs = pooled_and_by_town(for_stats_all, pairs=list(combinations(['prunario', 'scenariofuzz', 'drivefuzz', 'samota'], 2)))
print("=== POOLED (all towns, Table 2) ===")
print(pooled_df.to_string(index=False, float_format=lambda x: f"{x:.4f}"))

seq_final = defaultdict(list)
print("Sum")
for k, v in exp_total.items():
    seq_final[k] += [(str(_bs), _bs.violation) for _id, _time, _bs, _fp in v if not _fp and _bs.violation != '']
    if k in ['field', 'basic']:
        continue
    print(f"\t{k}: {len(seq_final[k])}")

res = {
    'prunario': Exp(Path('')),
    'scenariofuzz': Exp(Path('')),
    'drivefuzz': Exp(Path('')),
    'samota': Exp(Path('')),
    'field': Exp(Path('')),
    'basic': Exp(Path('')),
}
for _k, _log in seq_final.items():
    for i, (_bs_str, _v) in enumerate(_log):
        _bs = Sequence.from_str(_bs_str)
        _bs.violation = _v
        # res[_k].append(Path(''), _bs, _bs)
        res[_k].append(Path(f'{i}'), Scenario(Path(f'{i}'), empty=True), _bs, _bs)

_venn = {
    'prunario': set((str(_s), _s.violation) for _s in collect_unique_violations(res['prunario'])),
    'scenariofuzz': set((str(_s), _s.violation) for _s in collect_unique_violations(res['scenariofuzz'])),
    'drivefuzz': set((str(_s), _s.violation) for _s in collect_unique_violations(res['drivefuzz'])),
    'samota': set((str(_s), _s.violation) for _s in collect_unique_violations(res['samota'])),
    'field': set((str(_s), _s.violation) for _s in collect_unique_violations(res['field'])),
    'basic': set((str(_s), _s.violation) for _s in collect_unique_violations(res['basic'])),
}

print("Dedup")
for k in res:
    if k in ['field', 'basic']:
        continue
    print(f"\t{k}: {len(_venn[k])}")

print("###### TABLE 3 ######")
for k, v in execs_all.items():
    execs = sum(v) / len(v)
    uniqs = sum(uniqs_all[k]) / len(uniqs_all[k])
    ues = uniqs / execs * 100
    exect = sum(exect_all[k]) / len(exect_all[k])
    uniqt = sum(uniqt_all[k]) / len(uniqt_all[k])
    uet = uniqt / exect * 100
    if k in ['field', 'basic']:
        continue
    print(f"{k:20s}: ExecS: {execs:.1f}, UniqS: {uniqs:.1f} (UniqS/ExecS): {ues:.1f}%, ExecT: {exect:.0f}m, UniqT: {uniqt:.0f}m (UniqT/ExecT): {uet:.1f}%")
    
print("###### TABLE 4 ######")
for k, v in execs_all.items():
    execs = sum(v) / len(v)
    uniqs = sum(uniqs_all[k]) / len(uniqs_all[k])
    ues = uniqs / execs * 100
    exect = sum(exect_all[k]) / len(exect_all[k])
    uniqt = sum(uniqt_all[k]) / len(uniqt_all[k])
    uet = uniqt / exect * 100
    if k not in ['prunario', 'field', 'basic']:
        continue
    print(f"{k:20s}: ExecS: {execs:.1f}, UniqS: {uniqs:.1f} (UniqS/ExecS): {ues:.1f}%, ExecT: {exect:.0f}m, UniqT: {uniqt:.0f}m (UniqT/ExecT): {uet:.1f}%")
    


import json
from pathlib import Path

from matplotlib import pyplot as plt


def elementwise_sum(lists):
    return [sum(values) for values in zip(*lists)]


def get_vs_for_experiment(path: Path, _map: str):
    with (path.parent / 'time.json').open('r') as f:
        times = json.load(f)
    time_start = times['start']
    logs = sorted(path.glob(r'[0-9]*_[0-9]*'),
            key=lambda _p: list(map(int, str(_p.name).split('_'))))
    cnt_v = 0
    time_ran = 0.0
    _logs = []
    for run in logs:
        time_run = times[_map][run.name]['start']
        time_ran = time_run - time_start
        if (run / 'violation.txt').is_file() and not (run / 'fp').is_file():
            cnt_v += 1
            _logs.append((time_ran))#, cnt_v))
    interval = 30 * 60
    total_duration = 8 * 60 * 60

    res = []
    for i in range(1, total_duration // interval + 1):
        t = i * interval
        if i == 16:
            count = sum(1 for v in _logs)
        else:
            count = sum(1 for v in _logs if v <= t)
        res.append(count)
    return res


print("###### Figure 5 ######")

_paths = {
    "prunario": {
        "Town01": [
            'prunario_Town01_1/Town01_1',
            'prunario_Town01_2/Town01_1',
            'prunario_Town01_3/Town01_1',
        ],
        "Town03": [
            'prunario_Town03_1/Town03_1',
            'prunario_Town03_2/Town03_1',
            'prunario_Town03_3/Town03_1'
        ],
        "Town04": [
            'prunario_Town04_1/Town04_11',
            'prunario_Town04_2/Town04_11',
            'prunario_Town04_3/Town04_11'
        ],
        "Town05": [
            'prunario_Town05_1/Town05_11',
            'prunario_Town05_2/Town05_11',
            'prunario_Town05_3/Town05_11'
        ]
    },
    "field": {
        "Town01": [
            'field_Town01_1/Town01_1',
            'field_Town01_2/Town01_1',
            'field_Town01_3/Town01_1'
        ],
        'Town03': [
            'field_Town03_1/Town03_1',
            'field_Town03_2/Town03_1',
            'field_Town03_3/Town03_1'
        ],
        'Town04': [
            'field_Town04_1/Town04_11',
            'field_Town04_2/Town04_11',
            'field_Town04_3/Town04_11'
        ],
        'Town05': [
            'field_Town05_1/Town05_11',
            'field_Town05_2/Town05_11',
            'field_Town05_3/Town05_11',
        ]
    },
    "basic": {
        "Town01": [
            'basic_Town01_1/Town01_1',
            'basic_Town01_2/Town01_1',
            'basic_Town01_3/Town01_1'
        ],
        "Town03": [
            'basic_Town03_1/Town03_1',
            'basic_Town03_2/Town03_1',
            'basic_Town03_3/Town03_1'
        ],
        'Town04': [
            'basic_Town04_1/Town04_11',
            'basic_Town04_2/Town04_11',
            'basic_Town04_3/Town04_11'
        ],
        "Town05": [
            'basic_Town05_1/Town05_11',
            'basic_Town05_2/Town05_11',
            'basic_Town05_3/Town05_11',
        ]
    }
}


INTERVAL = 30 * 60 # 30 minutes
time_ran = 0.0


colors = {
    'basic': 'red',
    'field': 'green',
    'prunario': 'blue',
}

markers = {
    'basic': '^',
    'field': 'o',
    'prunario': 's',
}

label = {
    'basic': 'Basic',
    'field': 'Field',
    'prunario': 'Prunario',
}
plt.tick_params(axis='both', labelsize=12)
for target in _paths:
    log_vs = []
    num_v = 0
    res = elementwise_sum([get_vs_for_experiment(path_root / path, _map) for _map in _paths[target] for path in _paths[target][_map]])
    res_final = elementwise_sum([res])
    _x = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8.0]
    plt.plot(_x, [0] + res_final, label=label[target] + f" (Total: {res_final[-1]})", color=colors[target], marker=markers[target], markersize=6, linewidth=1, markerfacecolor='none')
plt.legend(fontsize=13)
plt.xlabel('Fuzzing time (hour)', fontsize=14)
plt.ylabel('Number of violations', fontsize=14)
plt.savefig('figure5.pdf', bbox_inches='tight', pad_inches=0)
print("Figure 5 is saved as figure5.pdf")

pooled_df, per_town_dfs = pooled_and_by_town(for_stats_all, pairs=list(combinations(['prunario', 'field', 'basic'], 2)))
print("=== POOLED (all towns, RQ2) ===")
print(pooled_df.to_string(index=False, float_format=lambda x: f"{x:.4f}"))
