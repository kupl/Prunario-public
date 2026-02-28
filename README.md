# Prunario

This repository contains the implementation of Prunario.


## Installation

To setup Prunario, please refer [INSTALL.md](INSTALL.md).


## Overall Structure of the Implementation

The core functionalities of Prunario and their corresponding implementations are as follows:

- `main.py`: implements the main testing algorithm (**Algorithm 1**).
- `fuzz/behavior/`
  - `behavior.py`: implements the pattern-based transformation procedure (**Section 3.1.1 Abstracting Simulation Results**).
- `fuzz/exp/`
  - `exp.py`: implements redundancy checking procedures (**Section 3.1.1, 3.2.3**).
- `fuzz/model/`:
  - `model.py`: implements the training procedures for speed-prediction model (**Algorithm 3**).
  - `utils.py`: implements feature extraction procedures (**Table 1**).
- `fuzz/simulator/`
  - `simulator.py`: implements physical simulation procedures (**line 11 in Algorithm 1**).
  - `simulator_abs.py`: implements simulation prediction procedures (**Algorithm 2**).
  - `planner_v.py`: implements record prediction procedures (**Algorithm 4, 5**).

## Run Prunario

- Please ensure that all procedures in [INSTALL.md](INSTALL.md) have been completed.
- Run the following command to run the testing loop:
  - We run wrapper.py instead of main.py to prevent the procedure from exiting due to a simulator error. The wrapper automatically restarts the simulator during execution.
  ```bash
  # Test Autoware in Town01 with Prunario
  python wrapper.py --path_seed seeds/seed_1 --path_log logs --gpu 0 --offscreen --town 1 --pruning
  # Test Autoware in Town03 with Prunario
  python wrapper.py --path_seed seeds/seed_3 --path_log logs --gpu 0 --offscreen --town 3 --pruning
  ```

- After running the command above, you can check the results under `logs/`.

- If you want to run Field and Basic (Section 5.2) without pruning, you can run the following command:
  ```bash
  # Test Autoware in Town01 with Field (static pruning)
  python wrapper.py --path_seed seeds/seed_1 --path_log logs --gpu 0 --offscreen --town 1 --naive
  # Test Autoware in Town01 with Basic
  python wrapper.py --path_seed seeds/seed_1 --path_log logs --gpu 0 --offscreen --town 1
  ```

## Reproduce the results in Section 5

### 0. Download
Download all files from [figshare](https://figshare.com/s/a6467432dff248a8a85d?file=56392964) and locate each files at the project root.

### 1. Table 2, 3, 4 and Figure 5
```bash
mkdir dataset_path
tar -xzvf dataset.tar.gz -C dataset_path
export PYTHONPATH=.
python scripts/evaluate.py --path dataset_path
```

### 2. Results in the paragraph "Precision and Recall of Pruning" (Section 5.2)
```bash
mkdir pnr_results
tar -xvzf pnr_results.tar.gz -C pnr_results
export PYTHONPATH=.
python scripts/precision_and_recall.py --path pnr_results
```