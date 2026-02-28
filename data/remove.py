import argparse
import os
from pathlib import Path
import shutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Parse arguments")
    parser.add_argument("--target", type=str, default="notree",
                        help="Path to the directory to remove files from.")
    args = parser.parse_args()
    return args

args = parse_args()

p = Path('res.txt')
to_remove = []
with p.open('r') as f:
    lines = f.readlines()
    for line in lines:
        if 'Import:' in line:
            continue
        if line.strip().startswith('Only in '):
            _, _, _path, name = line.strip().split(' ')
            _path = Path(_path.split(':')[0])
            to_remove.append(_path / name)
            # print(line.strip().split(' ')[-1])

for a in to_remove:
    if a.is_file():
        a.unlink()
    elif a.is_dir():
        # os.rmdir(a)
        shutil.rmtree(a)
    else:
        print(f"{a} is not a file or directory")
        continue
    print(f"Removed {a}")
