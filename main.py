import argparse
import json
from pathlib import Path
import time

from fuzz.commons.constants import *
from fuzz.commons.exceptions import PlannerException, ExtractionException, DDSimulateFail
from fuzz.commons.utils import construct_seed, get_logger, run_carla_with_docker, Timer, get_carla_host_ip, is_container_running


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='argument parser')

    parser.add_argument('--path_seed', '-s',
                        help='Path to seed directory. default: seed')
    parser.add_argument('--path_log', '-l', default='logs',
                        help='Path to save record. default: logs')
    parser.add_argument('--ads', default='autoware',
                        help="Testing target. 'autoware'")
    parser.add_argument('--offscreen', action='store_true',
                        help='Run carla without screen. If False, environment variable DISPLAY should be set with following command: "export DISPLAY=:1; xhost +".')
    parser.add_argument('--dry_run', action='store_true',
                        help='Run one scenario. Read the scenario from path_seed and save the result to path_log.')
    parser.add_argument('--pruning', action='store_true',
                        help='Testing with pruning')
    parser.add_argument('--naive', action='store_true',
                        help='Testing with naive pruning')
    parser.add_argument('--fps', default=FPS, type=int)
    parser.add_argument('--num_mutation_max', '-nm', default=NUM_MUTATION_MAX,
                        type=int, help='Number of mutation')
    parser.add_argument('--gpu', default=0, type=int,
                        help='GPU index to run')
    parser.add_argument('--version', default='20241212', type=str,
                        help='Autoware version')
    parser.add_argument('--start_delay', default=START_DELAY_AUTOWARE, type=float,
                        help='Set how long the zero-speed is maintained before driving.')
    parser.add_argument('--town', type=int, required=True,
                        help="Carla's Town index")
    parser.add_argument('--timeout', default=TIMEOUT_FUZZING, type=int, help='Timeout for the fuzzing (in seconds)')
    parser.add_argument('--repeat', default=3, type=int, help='Number of repeat')
    parser.add_argument('--mute', action='store_true', help='Mute trace logging while driving')

    args = parser.parse_args()

    if args.path_seed is None:
        args.path_seed = Path(__file__).resolve().parent / 'seed'
    else:
        args.path_seed = Path(args.path_seed)

    if args.path_log is None:
        args.path_log = Path(__file__).resolve().parent / 'logs'
    else:
        args.path_log = Path(args.path_log)
    
    if args.pruning and args.naive:
        print("Cannot use both --pruning and --naive at once. Choose one of them.")
        exit()

    args.ads = args.ads.lower()
    return args


def print_args(args: argparse.Namespace) -> None:
    if args.dry_run:
        _str = "Dry run"
    _ads = args.ads.lower()
    _pruning = args.pruning
    _field = args.naive
    _str = f'Fuzz {STR_RED(_ads)} ' + \
        f'with {STR_RED("Prunario" if _pruning else "Field" if _field else "Basic")} ' + \
        f'on {STR_RED(f"Town0{args.town}")} ' + \
        f'for {args.timeout} seconds ({STR_RED(args.timeout/3600)} hours, {STR_RED(args.repeat)} repeats) ' + \
        f'on GPU {STR_RED(args.gpu)}.'
    get_logger().info(_str)


def run_with_seed(seeds: List[Path],
                  ads,
                  args: argparse.Namespace,
                  name_map: str) -> None:
    # Initialize log directory for the seed
    for _i in range(1, args.repeat+1):
        _path_log = Path(f"{str(args.path_log)}_{_i}")
        if (_path_log / 'done').is_file():
            get_logger().info(f"{_path_log} is already done.")
            continue
        if not _path_log.is_dir():
            _path_log.mkdir(parents=True)
        with (_path_log / 'args.json').open('w') as f:
            _args = {**args.__dict__}
            _args['path_log'] = str(_path_log)
            _args['path_seed'] = str(args.path_seed)
            json.dump(_args, f, indent=4)
        if ads.id == 'autoware':
            ads.restart_planner()
        timer = Timer(
            path_log=_path_log,
            pruning=args.pruning,
            timeout=args.timeout
        )
        timer.start_fuzz()
        timer.dump()
        timer.start_seed(name_map)
        path_log = init_log_dir_for_seed(_path_log, seeds)
        path_data = path_log / 'data'
        path_data.mkdir(exist_ok=True)
        path_speeds = path_log / 'speeds'
        path_speeds.mkdir(exist_ok=True)
        if args.pruning or args.naive:
            path_pruned = path_log / 'mutants_pruned'
            path_pruned.mkdir(parents=True, exist_ok=True)
            dataset = dict()
            models = dict()
            if args.pruning:
                for target in ['ego', 'npc']:# + TYPES_VEHICLE:
                    models[target] = ModelML(
                        role=target,
                        _type='dt'
                    )
                    path_dataset = path_data / f'{target}.json'
                    if path_dataset.is_file():
                        _d = [(tuple(_l[0]), _l[-1])
                                for _l in json.load(path_dataset.open())]
                        dataset[target] = set(_d)
                        get_logger().info(
                            f"Train model for {target} with {len(dataset[target])} data")
                        _x, _y = zip(*dataset[target])
                        models[target].train(_x, _y)
                    else:
                        dataset[target] = set()
                        get_logger().info(f"No data for {target}")
        path_next_mutant = path_log / 'next_mutant'
        path_next_mutant.mkdir(parents=True, exist_ok=True)
        # Initialize seed pool
        pool = SeedPool(path_log)
        if len(pool) <= 0:  # First time to run
            # Construct seed scenarios and push to the seed pool
            for seed in reversed(seeds):
                _scenario_seed = Scenario(seed)
                pool.push(_scenario_seed, 0.0)
            loaded = False
        else:
            loaded = True
        set_exp = Exp(path_log)
        # Generate and run each mutant
        _iter = 1
        scenario_ran = []
        # Collect previously ran scenarios
        for _run in path_log.glob(r'[0-9]*_[0-9]*'):
            scenario_ran.append(Scenario(_run))
        # Until the pool is empty and timeout
        while not pool.is_empty() or loaded:
            if timer.is_timeout():
                get_logger().info("Timeout reached.")
                timer.end_seed()
                timer.dump()
                if not (path_log.parent / 'done').is_file():
                    (path_log.parent / 'done').touch()
                break
            get_logger().info(f"Seed Pool({len(pool)}): {pool}")
            if loaded:
                scenario_best, score_best = Scenario(
                    Path(pool.seed_curr)), pool.score_curr
                loaded = False
                _iter = pool.num_pop
            else:
                scenario_best, score_best = pool.pop()
            j = 0
            get_logger().info(f"Current scenario: {scenario_best.path_scenario}")
            # Generate mutants from the seed
            if args.dry_run:
                mutants = iter([scenario_best])
            else:
                mutants = generate_mutants(
                    seed=scenario_best,
                    data_generation=False,
                    ads=ads
                )
            while True:
                if timer.is_timeout():
                    get_logger().info("Timeout reached.")
                    timer.end_seed()
                    timer.dump()
                    if not (path_log.parent / 'done').is_file():
                        (path_log.parent / 'done').touch()
                    break
                if j >= args.num_mutation_max:
                    break
                run_name = f'{_iter}_{j+1}'
                _path_run = path_log / run_name
                _path_state = _path_run / NAME_FILE_STATE
                _num_pruned = 0
                _num_next_mutant = len(
                    list(path_next_mutant.glob(f'{run_name}_*.json')))
                if not _path_state.is_file():
                    timer.start_run(run_name)
                    timer.start_pruning()
                    dd_failed = False
                    while True:
                        if timer.is_timeout():
                            get_logger().info("Timeout reached.")
                            timer.end_seed()
                            timer.dump()
                            if not (path_log.parent / 'done').is_file():
                                (path_log.parent / 'done').touch()
                            break
                        mutant = next(mutants)
                        if mutant in scenario_ran:
                            continue
                        if args.pruning:# or args.baseline:
                            # For w/ pruning
                            try:
                                if args.pruning:
                                    bs_hat = run_abs_real(
                                        mutant,
                                        models,
                                        ads,
                                        args.start_delay,
                                        path_run=_path_run
                                    )
                                # elif args.baseline:
                                #     bs_hat = run_abs_baseline(mutant)
                            except DDSimulateFail as _:
                                get_logger().info("Run the mutant due to the failure of simulation prediction.")
                                dd_failed = True
                                break
                            except PlannerException as _:
                                continue
                            except (RuntimeError, IndexError, ExtractionException) as e:
                                import traceback
                                traceback.print_exc()
                                continue
                            # If there exists any equivalent behavior sequence, skip to the next mutant
                            if set_exp.ddredundant(bs_hat['ego']):
                                _num_pruned += 1
                                get_logger().info(f"Prune {run_name}_{_num_pruned}")
                                # Save pruned mutant
                                mutant.dump(
                                    path_pruned, name=f'{run_name}_{_num_pruned}_{time.time()}.json')
                                continue
                            else:
                                # If the mutant is new, run the mutant
                                break
                        elif args.naive:
                            if set_exp.ddredundant_autofuzz(mutant):
                                _num_pruned += 1
                                get_logger().info(f"Prune (naive) {run_name}_{_num_pruned}")
                                # Save pruned mutant
                                mutant.dump(
                                    path_pruned, name=f'{run_name}_{_num_pruned}_{time.time()}.json')
                                continue
                            else:
                                # If the mutant is new, run the mutant
                                break
                        else:
                            # For w/o pruning (baseline)
                            break
                    mutant.path_scenario = _path_run
                    timer.end_pruning(_num_pruned)
                    # Run the scenario
                    if args.dry_run:
                        get_logger().info(f"Run simulation - Dry Run")
                    else:
                        get_logger().info(f"Run simulation - {run_name}")
                    # Run the simulation with carla
                    try:
                        record = simulator.run(
                            path_run=_path_run,
                            scenario=mutant,
                            ads=ads,
                            timer=timer,
                            id_gpu=args.gpu,
                            mute=args.mute
                        )
                    except RunNextMutant as _:
                        mutant.dump(path_next_mutant,
                                    name=f'{run_name}_{_num_next_mutant}.json')
                        _num_next_mutant += 1
                        continue
                    except KeyboardInterrupt as _:
                        # Keyboard interrupt while running
                        get_logger().info("Exited by user")
                        # import traceback
                        # traceback.print_exc()
                        exit()
                    finally:
                        simulator.clean_up(ads)
                        # simulator.clean_up(args.target)
                    # Compute feedback score
                    timer.start_feedback()
                    feedback = get_feedback(_path_run, record.state)
                    timer.end_feedback()
                    feedback.dump(_path_run)
                    # bs = get_driving_pattern_sequence(record)
                    bs, idxs_first = get_driving_pattern_sequences(mutant.mission, record)
                    for _id in bs:
                        if _id == 'ego':
                            bs[_id].dump(_path_run, name=NAME_FILE_BEHAVIOR)
                        else:
                            bs[_id].dump(_path_run, name=f'behavior_{_id}.json')
                    # bs.dump(_path_run, name=NAME_FILE_BEHAVIOR)
                    if args.pruning:# or args.baseline:
                        # if len(bs_hat) <= 0:
                        #     # bs.dump(_path_run, name=NAME_FILE_BEHAVIOR_HAT)
                        #     with (_path_run / 'ego_first').open('w') as f:
                        #         f.write('ego_first')
                        # else:
                        if not dd_failed:
                            # bs_hat.dump(_path_run, name=NAME_FILE_BEHAVIOR_HAT)
                            for _id in bs_hat:
                                if _id == 'ego':
                                    bs_hat[_id].dump(_path_run, name=NAME_FILE_BEHAVIOR_HAT)
                                else:
                                    bs_hat[_id].dump(_path_run, name=f'behavior_hat_{_id}.json')
                        else:
                            with (_path_run / 'dd_failed').open('w') as f:
                                f.write('dd_failed')
                        # Extract data from the record
                        path_models = _path_run / 'models'
                        path_models.mkdir()
                        dataset_new = extract_speed_features(mutant, record, until=idxs_first)
                        # Train and save the model
                        for target in ['ego', 'npc']: # + TYPES_VEHICLE:
                            # Save the model used for the prediction
                            models[target].save(path_models)
                            dataset[target].update(dataset_new[target])
                            if len(dataset[target]) <= 0:
                                continue
                            # Save data
                            json.dump(list(dataset[target]), open(
                                path_data / f'{target}.json', 'w'))
                            # Train model
                            _x, _y = zip(*dataset[target])
                            models[target].train(_x, _y)
                            get_logger().info(f"Trained model for {target} with {len(dataset[target])} data")
                    # Check if the record is new from the experienced set
                    if not set_exp.redundant(bs['ego']):
                        if record.any_violation:
                            # if ADS made any misbehavior
                            get_logger().info(
                                "\x1b[31;20m" + f'{record.violation} detected'.upper() + "\x1b[0m")
                        else:
                            get_logger().info(
                                "\x1b[31;20m" + f'Arrived to the destination' + "\x1b[0m")
                            # Add to the seed pool only if the scenario is normal.
                            get_logger().info(
                                f"Feedback score: {feedback.score:.3f} (Best: {score_best})")
                            # if feedback.score > score_best:
                            pool.push(mutant, feedback.score)
                    else:
                        get_logger().info("Already experienced behavior sequence. Ignore it.")
                        with (_path_run / 'ignore').open('w') as f:
                            f.write('ignore')
                    # Add to the experienced set
                    if not args.dry_run:
                        # if (args.pruning or args.baseline) and not dd_failed:
                        if args.pruning and not dd_failed:
                            set_exp.append(_path_run, mutant, bs['ego'], bs_hat['ego'])
                        else:
                            set_exp.append(_path_run, mutant, bs['ego'], bs['ego'])
                    timer.end_run()
                    timer.dump()
                    pool.dump()
                    set_exp.dump()
                    scenario_ran.append(mutant)
                    if args.dry_run:
                        get_logger().info("Dry run done.")
                        exit()
                j += 1
            _iter += 1
        else:
            get_logger().info("Seed pool is empty. Try next seed.")
            return


if __name__ == '__main__':
    args = parse_args()
    print_args(args)
    Config.VERSION_CARLA = '0.9.15'
    Config.PORT_CARLA = PORT_CARLA + args.gpu*3
    Config.PORT_TM = PORT_TM + args.gpu*3
    # if args.ads == 'autoware':
    Config.NAME_CONTAINER_ADS = f'autoware_universe_4a3de49_{args.gpu}'
    Config.NAME_CONTAINER_ADS_PLANNER = f'autoware_universe_planner_{args.gpu}'
    Config.NAME_CONTAINER_CARLA = f'carla-{NAME_USER}-{Config.VERSION_CARLA}_{args.gpu}'
    if not is_container_running(Config.NAME_CONTAINER_CARLA):
        run_carla_with_docker(
            version=Config.VERSION_CARLA,
            port=Config.PORT_CARLA,
            offscreen=args.offscreen,
            id_gpu=args.gpu
        )
    Config.PATH_SIMULATION = PROJECT_ROOT / 'shared' / f'autoware_universe_{args.gpu}'
    Config.HOST_CARLA = get_carla_host_ip()
    Config.PATH_SIMULATION_LOG = Config.PATH_SIMULATION / 'logs'
    Config.PATH_LOCK = Config.PATH_SIMULATION_LOG / 'lock'
    Config.PATH_DATA = PROJECT_ROOT / 'data'
    Config.PATH_CARLA_AUTOWARE = Config.PATH_SIMULATION / 'carla_autoware'
    Config.TIMEOUT_STALLING = 20
    # construct seed pool (scenario pool)
    get_logger().info(
        f"Construct seed from the seed directory {args.path_seed}")
    while True:
        try:
            from fuzz.carla import cc
            break
        except RuntimeError as e:
            if 'while waiting for the simulator, make sure' in str(e):
                print(e)
                run_carla_with_docker(
                    version=Config.VERSION_CARLA,
                    port=Config.PORT_CARLA,
                    offscreen=args.offscreen,
                    id_gpu=args.gpu
                )
    from fuzz.ads import Autoware#, AutowareOld
    from fuzz.behavior import get_driving_pattern_sequences
    from fuzz.commons.exceptions import RunNextMutant
    from fuzz.data import Scenario
    from fuzz.exp import Exp
    from fuzz.feedback import get_feedback
    from fuzz.selection import SeedPool
    from fuzz.simulation.simulator_abs import run_abs_real
    from fuzz.simulation.utils import init_log_dir_for_seed
    from fuzz.model import ModelML, extract_speed_features
    from fuzz.mutation import generate_mutants
    import fuzz.simulation.simulator as simulator
    if args.ads == 'autoware':
        _name_map = f'Town0{args.town}' if args.town < 10 else f'Town{args.town}HD'
        ads = Autoware(_map=_name_map, version=args.version)
    else:
        raise ValueError("Invalid target")
    simulator.clean_up(ads)
    # for each seed
    for seed in construct_seed(args.path_seed, args.dry_run):
        name_map = seed[0].stem.split('_')[0]
        cc.load_world(name_map)
        run_with_seed(seed, ads, args, name_map)
