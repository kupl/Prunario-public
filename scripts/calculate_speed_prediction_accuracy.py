import json
from pathlib import Path

from fuzz.behavior import Sequence, get_driving_pattern_sequences
from fuzz.carla import cc
from fuzz.commons.constants import *
from fuzz.data import Record, Scenario

from tqdm import tqdm


def convert_path(p: Path) -> Path:
    parent = p.parent
    name = p.name

    parts = name.split('_')
    tail = '_'.join(parts[-2:])

    parent = Path('dataset_path')
    if 'town01' in str(p):
        name = 'Town01'
        part = 'Town01_1'
    elif 'town03' in str(p):
        name = 'Town03'
        part = 'Town03_1'
    elif 'town04' in str(p):
        name = 'Town04'
        part = 'Town04_11'
    else:
        name = 'Town05'
        part = 'Town05_11'
    new_path = parent / f'prunario_{name}_{parts[5]}' / part / tail
    return new_path


path_root = Path('speeds/speeds')

data_ego = {
    'speeds_pred': [],
    'speeds_real': []
}
data_npc = {
    'speeds_pred': [],
    'speeds_real': []
}

for path_log in tqdm(sorted(path_root.glob('*'))):
    if 'town01' in str(path_log):
        _map = 'Town01'
    elif 'town03' in str(path_log):
        _map = 'Town03'
    elif 'town04' in str(path_log):
        _map = 'Town04'
    else:
        _map = 'Town05'
    cc.load_world(_map)
    if not (path_log / 'ego_speeds_raw.json').is_file() or not (path_log / 'npc_speeds_raw.json').is_file():
        continue
    js_ego = json.load((path_log / 'ego_speeds_raw.json').open())
    js_npc = json.load((path_log / 'npc_speeds_raw.json').open())
    run = convert_path(path_log)
    scenario = Scenario(run)
    record = Record(run)
    # Ego
    bs, idx_itr = get_driving_pattern_sequences(scenario.mission, record)
    idx = min(idx_itr.values())
    bs = Sequence.load(run)
    if len(bs.details) == 1:
        idx_ego = bs.details[0][1]
    else:
        idx_itr = bs.get_idx_first_interaction()
        idx_ego = bs.details[idx_itr][0] + K
    if (run / NAME_FILE_BEHAVIOR_HAT).is_file():
        bs_hat = Sequence.load(run, name=NAME_FILE_BEHAVIOR_HAT)
        idx_itr = bs_hat.get_idx_first_interaction()
        idx_ego = min(idx_ego, bs_hat.details[idx_itr][0] + K)
    pred_ego = js_ego['speeds_predicted_raw']
    real_ego = js_ego['speeds_real_raw']
    len_min_ego = min(len(pred_ego), len(real_ego), idx_ego)
    pred_ego = pred_ego[:len_min_ego]
    real_ego = real_ego[:len_min_ego]
    data_ego['speeds_pred'].extend(pred_ego)
    data_ego['speeds_real'].extend(real_ego)
    # NPC
    pred_npc = js_npc['speeds_predicted_raw']
    real_npc = js_npc['speeds_real_raw']
    for bp in pred_npc:
        if bp not in real_npc:
            continue
        pred_npc_bp = pred_npc[bp]
        real_npc_bp = real_npc[bp]
        len_min = min(len(pred_npc_bp), len(real_npc_bp), idx)
        pred_npc_bp = pred_npc_bp[:len_min]
        real_npc_bp = real_npc_bp[:len_min]
        data_npc['speeds_pred'].extend(pred_npc_bp)
        data_npc['speeds_real'].extend(real_npc_bp)
    for npc in scenario.npcs['vehicles'] + scenario.npcs['pedestrians']:
        if npc['type_motion'] != DYNAMIC:
            data_npc['speeds_pred'].extend([0.0] * len(record.state.state))
            data_npc['speeds_real'].extend([0.0] * len(record.state.state))

print("==== Ego ====")
mae_ego = sum(abs(a - b)
              for a, b in zip(data_ego['speeds_pred'], data_ego['speeds_real'])) / len(data_ego['speeds_pred'])
print(
    f"MAE: {mae_ego:.4f} km/h over {len(data_ego['speeds_pred'])} samples (avg cnt: {len(data_ego['speeds_pred']) / 12})")
print("==== NPC ====")
mae_npc = sum(abs(a - b)
              for a, b in zip(data_npc['speeds_pred'], data_npc['speeds_real'])) / len(data_npc['speeds_pred'])
print(
    f"MAE: {mae_npc:.4f} km/h over {len(data_npc['speeds_pred'])} samples (avg cnt: {len(data_npc['speeds_pred']) / 12})")
