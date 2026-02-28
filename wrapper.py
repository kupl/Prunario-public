import os
from pathlib import Path
import sys
import time


while (True):
    args = ' '.join(sys.argv[1:])
    CARLA_PATH = Path(__file__).parent / 'data' / 'PythonAPI' / 'carla'
    cmd = f'export PYTHONPATH={CARLA_PATH}:$PYTHONPATH; python3 main.py {args}'
    os.system(cmd)
    time.sleep(10)
