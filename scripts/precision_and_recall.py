import argparse
import json
from pathlib import Path


argparser = argparse.ArgumentParser()
argparser.add_argument('--path', type=str, required=True)
args = argparser.parse_args()

_paths = {
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
_paths = {k: [Path(args.path) / p for p in v] for k, v in _paths.items()}

for technique in ['field', 'prunario']:
    if technique == 'field':
        REPEAT = 1
    else: # prunario
        REPEAT = 3
    all_logs = []
    for _map, paths in _paths.items():
        for path in paths:
            for i in range(REPEAT):
                _path = Path(__file__).parent.parent / 'pnr_results'
                name_file = f'{_map}_{Path(path).parent.name}_{technique}_{i+1}.json'
                all_logs += json.load(open(_path / name_file, 'r'))
    tps = [_r for _r in all_logs if _r.get('result') == 'tp']
    fps = [_r for _r in all_logs if _r.get('result') == 'fp']
    tns = [_r for _r in all_logs if _r.get('result') == 'tn']
    fns = [_r for _r in all_logs if _r.get('result') == 'fn']
    print(technique)
    print(f"\tPrecision: {len(tps) / (len(tps) + len(fps)) * 100 if (len(tps) + len(fps)) > 0 else 0:.1f}%")
    print(f"\tRecall: {len(tps) / (len(tps) + len(fns)) * 100 if (len(tps) + len(fns)) > 0 else 0:.1f}%")
