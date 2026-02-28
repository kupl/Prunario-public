import carla
from pathlib import Path
from typing import Dict, List

from fuzz.commons.constants.simulation import FPS

K: int = 20

TIMEOUT_FUZZING: int = 8 * 60 * 60 # 8 hours

TIMEOUT: int = 180
# PATH_SIMULATION: Path = PROJECT_ROOT / 'shared'
# PATH_SIMULATION_LOG: Path = PATH_SIMULATION / 'logs'
# PATH_LOCK: Path = PATH_SIMULATION_LOG / 'lock'
# PATH_DATA: Path = PROJECT_ROOT / 'data'
# PATH_CARLA_AUTOWARE: Path = PATH_SIMULATION / 'carla_autoware'


NEAR_POINT_MUTATION_THRESHOLD: int = 30
MISSION_MUTATION_NO_LIMIT: int = -1

POLLING_INTERVAL: float = 0.05
LOAD_TIMEOUT: int = 180
IGNORE_LAST: int = 1
DISTANCE_LOOKAHEAD: float = 50.0 #100.0
INTERVAL_LOOKAHEAD: float = 10.0 #10.0

THRESHOLD_STOP: float = 1 # 0.1
THRESHOLD_INTER: float = 3.0 #5.0

MIN_YAW: float = 0.08 #0.06 # 0.1 #08 #0.5
MAX_YAW: float = 20.0
OFFSET_YAW: float = 0.05

SIZE_CONTEXT: int = 4

NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD: float = 2.300000

CHANCES_RUN_AGAIN: int = 3

THRESHOLD_INTERESTING: int = 1

NUM_SELECTION_MAX: int = 5
NUM_MUTATION_MAX: int = 10

POLICY_DRYRUN: str = 'dryrun'
POLICY_RANDOM: str = 'random'
POLICY_HEURISTICS: str = 'heuristics'
POLICY_BETTER_HEURISTICS: str = 'better_heuristics'

MAP_COVERAGE_DISTANCE_BETWEEN_WAYPOINTS: int = 5
NUM_SCENARIOS: int = 30

# Mutation
DISTANCE_FORWARD: float = 10.0
DISTANCE_BACKWARD: float = 10.0
DURATION_MINIMUM: float = 0.5 * FPS
RANGE_PERTURB: float = 10.0
NEAR_DISTANCE: float = 50.0
FAR_DISTANCE: float = 10.0
OBSTACLE_NEAR_LIMIT: float = 30.0
NUM_TRIAL_MUTATION: int = 30
DYNAMIC: str = "dynamic"
LINEAR: str = "linear"
STATIC: str = "static"
TYPES_VEHICLE = [
    # car
    # "vehicle.audi.a2",
    # "vehicle.ford.mustang",
    "vehicle.mercedes.coupe",
    # "vehicle.micro.microlino",
    # Truck
    "vehicle.carlamotors.carlacola",
    # Van
    "vehicle.volkswagen.t2",
    # bicycle
    "vehicle.gazelle.omafiets",
    # motorcycle
    "vehicle.yamaha.yzf"
]
TYPES_PEDESTRIAN = [
    'walker.pedestrian.0011',   # Girl
    'walker.pedestrian.0001',   # Woman
    'walker.pedestrian.0012',   # Boy
    'walker.pedestrian.0003',   # Man
]
MAPPING_VEHICLE = {
    "vehicle.mercedes.coupe": 0,
    "vehicle.carlamotors.carlacola": 1,
    "vehicle.volkswagen.t2": 2,
    "vehicle.gazelle.omafiets": 3,
    "vehicle.yamaha.yzf": 4
}

# Feedback
F_SLOW: float = 4.0
F_LIMIT_DIST: float = 100.0 #30.0
NAME_FEATURES: List[str] = [
    'dist_min_to_vehicles',
    'dist_min_to_pedestrians',
    'map_coverage',
    'dist_max_from_lane_center',
    'fast_accel',
    'npc_crossing',
    'hard_brake',
    'trajectory_shape'
]

LEFT: int = 1
RIGHT: int = 2
STRAIGHT: int = 3
TRAJECTORY_SHAPES_ENCODING_MAP: Dict[int, str] = {
    LEFT: 'LEFT',
    RIGHT: 'RIGHT',
    STRAIGHT: 'STRAIGHT'
}

START_DELAY_AUTOWARE: float = 3.5

PATTERNS_STR = {

}

NAME_FILE_BEHAVIOR: str = 'behavior.json' #'behavior.txt'
# NAME_FILE_BEHAVIOR_ORIGINAL: str = 'behavior_original.txt'
NAME_FILE_BEHAVIOR_HAT: str = 'behavior_hat.json' #'behavior_hat.txt'


BB: Dict[str, Dict[str, Dict[str, float]]] = {
    "vehicle.audi.a2": {
        "location": {
            "x": 2.108439730363898e-06,
            "y": 0.0003394893719814718,
            "z": 0.7818071842193604
        },
        "extent": {
            "x": 1.852684736251831,
            "y": 0.8943392634391785,
            "z": 0.7745251059532166
        }
    },
    "vehicle.audi.etron": {
        "location": {
            "x": -0.00359594845212996,
            "y": 2.6822089438383045e-08,
            "z": 0.8205030560493469
        },
        "extent": {
            "x": 2.427854299545288,
            "y": 1.0163782835006714,
            "z": 0.8246796727180481
        }
    },
    "vehicle.audi.tt": {
        "location": {
            "x": -0.0003713149926625192,
            "y": 1.6853212514433835e-07,
            "z": 0.6926480531692505
        },
        "extent": {
            "x": 2.0906050205230713,
            "y": 0.9970585703849792,
            "z": 0.6926480531692505
        }
    },
    "vehicle.bh.crossbike": {
        "location": {
            "x": 0.0,
            "y": 7.629394360719743e-08,
            "z": 0.8082212805747986
        },
        "extent": {
            "x": 0.7546613812446594,
            "y": 0.4329703152179718,
            "z": 0.8061768412590027
        }
    },
    "vehicle.bmw.grandtourer": {
        "location": {
            "x": 8.846446917232242e-07,
            "y": -0.0005331939901225269,
            "z": 0.7651039958000183
        },
        "extent": {
            "x": 2.3055028915405273,
            "y": 1.1208566427230835,
            "z": 0.8336379528045654
        }
    },
    "vehicle.carlamotors.carlacola": {
        "location": {
            "x": 6.403671903854047e-08,
            "y": 1.8299344901606673e-06,
            "z": 1.23372220993042
        },
        "extent": {
            "x": 2.601919174194336,
            "y": 1.3134948015213013,
            "z": 1.2337223291397095
        }
    },
    "vehicle.carlamotors.european_hgv": {
        "location": {
            "x": 0.012722599320113659,
            "y": 0.005304763093590736,
            "z": 1.7306151390075684
        },
        "extent": {
            "x": 3.967855215072632,
            "y": 1.4455441236495972,
            "z": 1.7309716939926147
        }
    },
    "vehicle.carlamotors.firetruck": {
        "location": {
            "x": -0.25344118475914,
            "y": 0.005304698832333088,
            "z": 1.9133497476577759
        },
        "extent": {
            "x": 4.234020709991455,
            "y": 1.4455441236495972,
            "z": 1.9137061834335327
        }
    },
    "vehicle.chevrolet.impala": {
        "location": {
            "x": 5.7747958635445684e-05,
            "y": 0.0017898466903716326,
            "z": 0.7014925479888916
        },
        "extent": {
            "x": 2.6787397861480713,
            "y": 1.0166014432907104,
            "z": 0.7053293585777283
        }
    },
    "vehicle.citroen.c3": {
        "location": {
            "x": -2.2158026524721208e-07,
            "y": 0.0023671784438192844,
            "z": 0.7619171738624573
        },
        "extent": {
            "x": 1.9938424825668335,
            "y": 0.9254241585731506,
            "z": 0.8085547685623169
        }
    },
    "vehicle.diamondback.century": {
        "location": {
            "x": 0.0,
            "y": 7.629394360719743e-08,
            "z": 0.8157447576522827
        },
        "extent": {
            "x": 0.8281218409538269,
            "y": 0.2912190854549408,
            "z": 0.8098834753036499
        }
    },
    "vehicle.dodge.charger_2020": {
        "location": {
            "x": -0.0053450604900717735,
            "y": 1.633167272530045e-07,
            "z": 0.7655780911445618
        },
        "extent": {
            "x": 2.5039126873016357,
            "y": 0.9408109784126282,
            "z": 0.7673624753952026
        }
    },
    "vehicle.dodge.charger_police": {
        "location": {
            "x": 0.023638982325792313,
            "y": 0.00060065503930673,
            "z": 0.7926706671714783
        },
        "extent": {
            "x": 2.487122058868408,
            "y": 1.0192005634307861,
            "z": 0.7710590958595276
        }
    },
    "vehicle.dodge.charger_police_2020": {
        "location": {
            "x": 0.10949568450450897,
            "y": 0.0,
            "z": 0.8174071311950684
        },
        "extent": {
            "x": 2.6187572479248047,
            "y": 0.9648797512054443,
            "z": 0.819191575050354
        }
    },
    "vehicle.ford.ambulance": {
        "location": {
            "x": -0.28856074810028076,
            "y": 0.0019410132663324475,
            "z": 1.2156093120574951
        },
        "extent": {
            "x": 3.18282151222229,
            "y": 1.1755871772766113,
            "z": 1.215687870979309
        }
    },
    "vehicle.ford.crown": {
        "location": {
            "x": 0.1854064017534256,
            "y": 2.47955313170678e-07,
            "z": 0.7684469223022461
        },
        "extent": {
            "x": 2.6828393936157227,
            "y": 0.9003620743751526,
            "z": 0.7874829769134521
        }
    },
    "vehicle.ford.mustang": {
        "location": {
            "x": 0.03214368596673012,
            "y": 1.2159347306806012e-07,
            "z": 0.650469958782196
        },
        "extent": {
            "x": 2.358762502670288,
            "y": 0.947413444519043,
            "z": 0.650469958782196
        }
    },
    "vehicle.gazelle.omafiets": {
        "location": {
            "x": -0.009999999776482582,
            "y": 7.629394360719743e-08,
            "z": 0.8946490287780762
        },
        "extent": {
            "x": 0.9217206835746765,
            "y": 0.3295213580131531,
            "z": 0.8879144191741943
        }
    },
    "vehicle.harley-davidson.low_rider": {
        "location": {
            "x": 0.0,
            "y": 7.629394360719743e-08,
            "z": 0.8246490359306335
        },
        "extent": {
            "x": 1.175087809562683,
            "y": 0.3831165134906769,
            "z": 0.8247470855712891
        }
    },
    "vehicle.jeep.wrangler_rubicon": {
        "location": {
            "x": 0.00021681956422980875,
            "y": 0.0010300345020368695,
            "z": 0.93227618932724
        },
        "extent": {
            "x": 1.9331103563308716,
            "y": 0.9525982737541199,
            "z": 0.9389679431915283
        }
    },
    "vehicle.kawasaki.ninja": {
        "location": {
            "x": 0.0,
            "y": 7.629394360719743e-08,
            "z": 0.7688607573509216
        },
        "extent": {
            "x": 1.021842122077942,
            "y": 0.3984561562538147,
            "z": 0.761595606803894
        }
    },
    "vehicle.lincoln.mkz_2017": {
        "location": {
            "x": 0.004045447334647179,
            "y": 1.2218951894737984e-07,
            "z": 0.7188605070114136
        },
        "extent": {
            "x": 2.4508416652679443,
            "y": 1.0641621351242065,
            "z": 0.7553732395172119
        }
    },
    "vehicle.lincoln.mkz_2020": {
        "location": {
            "x": -0.006292920559644699,
            "y": -2.5749204723979346e-07,
            "z": 0.748845636844635
        },
        "extent": {
            "x": 2.44619083404541,
            "y": 0.9183566570281982,
            "z": 0.7451388239860535
        }
    },
    "vehicle.mercedes.coupe": {
        "location": {
            "x": 2.212822437286377e-06,
            "y": -0.0007744769100099802,
            "z": 0.6667942404747009
        },
        "extent": {
            "x": 2.5133883953094482,
            "y": 1.0757731199264526,
            "z": 0.8253258466720581
        }
    },
    "vehicle.mercedes.coupe_2020": {
        "location": {
            "x": -0.0025943678338080645,
            "y": -2.1070241018605884e-07,
            "z": 0.7203620076179504
        },
        "extent": {
            "x": 2.3368194103240967,
            "y": 0.9059062600135803,
            "z": 0.7209736704826355
        }
    },
    "vehicle.mercedes.sprinter": {
        "location": {
            "x": -0.010540497489273548,
            "y": 0.004136416129767895,
            "z": 1.2882417440414429
        },
        "extent": {
            "x": 2.957595109939575,
            "y": 0.9942164421081543,
            "z": 1.2803276777267456
        }
    },
    "vehicle.micro.microlino": {
        "location": {
            "x": -2.0918621430610074e-06,
            "y": -5.977785986033268e-05,
            "z": 0.6771554350852966
        },
        "extent": {
            "x": 1.1036475896835327,
            "y": 0.7404598593711853,
            "z": 0.6880123615264893
        }
    },
    "vehicle.mini.cooper_s": {
        "location": {
            "x": -2.0918621430610074e-06,
            "y": -5.977785986033268e-05,
            "z": 0.6930100917816162
        },
        "extent": {
            "x": 1.9029000997543335,
            "y": 0.9851378202438354,
            "z": 0.7375150918960571
        }
    },
    "vehicle.mini.cooper_s_2021": {
        "location": {
            "x": -0.031254496425390244,
            "y": -6.985663958403165e-07,
            "z": 0.8910346627235413
        },
        "extent": {
            "x": 2.2763495445251465,
            "y": 1.0485360622406006,
            "z": 0.8835831880569458
        }
    },
    "vehicle.mitsubishi.fusorosa": {
        "location": {
            "x": -0.4452227056026459,
            "y": -0.12088702619075775,
            "z": 2.127350330352783
        },
        "extent": {
            "x": 5.136342525482178,
            "y": 1.9720759391784668,
            "z": 2.1264240741729736
        }
    },
    "vehicle.nissan.micra": {
        "location": {
            "x": -2.8334557100606617e-07,
            "y": 0.0009692307212390006,
            "z": 0.7812460660934448
        },
        "extent": {
            "x": 1.8166879415512085,
            "y": 0.9225568771362305,
            "z": 0.7506412863731384
        }
    },
    "vehicle.nissan.patrol": {
        "location": {
            "x": -0.05737816169857979,
            "y": 0.00015922545571811497,
            "z": 0.9351239800453186
        },
        "extent": {
            "x": 2.3022549152374268,
            "y": 0.9657964706420898,
            "z": 0.9274230599403381
        }
    },
    "vehicle.nissan.patrol_2021": {
        "location": {
            "x": 0.02838161401450634,
            "y": -1.4781952017983713e-07,
            "z": 1.0167162418365479
        },
        "extent": {
            "x": 2.782914400100708,
            "y": 1.0749834775924683,
            "z": 1.0225735902786255
        }
    },
    "vehicle.seat.leon": {
        "location": {
            "x": -0.00015224098751787096,
            "y": -0.0015581287443637848,
            "z": 0.7396332621574402
        },
        "extent": {
            "x": 2.0964150428771973,
            "y": 0.9080929160118103,
            "z": 0.7369155883789062
        }
    },
    "vehicle.tesla.cybertruck": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.049095630645752
        },
        "extent": {
            "x": 3.1367766857147217,
            "y": 1.1947870254516602,
            "z": 1.049095630645752
        }
    },
    "vehicle.tesla.model3": {
        "location": {
            "x": 0.02922196500003338,
            "y": -2.3603438137342891e-07,
            "z": 0.7358605265617371
        },
        "extent": {
            "x": 2.3958897590637207,
            "y": 1.081725001335144,
            "z": 0.7438300251960754
        }
    },
    "vehicle.toyota.prius": {
        "location": {
            "x": 0.001989735523238778,
            "y": -0.00038415222661569715,
            "z": 0.724169135093689
        },
        "extent": {
            "x": 2.256761312484741,
            "y": 1.0034072399139404,
            "z": 0.7624167203903198
        }
    },
    "vehicle.vespa.zx125": {
        "location": {
            "x": 0.009999999776482582,
            "y": 7.629394360719743e-08,
            "z": 0.7946490049362183
        },
        "extent": {
            "x": 0.9085533022880554,
            "y": 0.4329703152179718,
            "z": 0.7950454354286194
        }
    },
    "vehicle.volkswagen.t2": {
        "location": {
            "x": 0.0013296699617058039,
            "y": -0.0005375533364713192,
            "z": 1.0144715309143066
        },
        "extent": {
            "x": 2.2402184009552,
            "y": 1.034657597541809,
            "z": 1.0188959836959839
        }
    },
    "vehicle.volkswagen.t2_2021": {
        "location": {
            "x": 0.11667035520076752,
            "y": -0.0003524398780427873,
            "z": 0.9912433624267578
        },
        "extent": {
            "x": 2.2210919857025146,
            "y": 0.88728266954422,
            "z": 0.9936033487319946
        }
    },
    "vehicle.yamaha.yzf": {
        "location": {
            "x": 0.0,
            "y": 7.629394360719743e-08,
            "z": 0.7746490240097046
        },
        "extent": {
            "x": 1.0953842401504517,
            "y": 0.4329586327075958,
            "z": 0.7651907205581665
        }
    },
    "walker.pedestrian.0001": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0002": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0003": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0004": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0005": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0006": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0007": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0008": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0009": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.25,
            "y": 0.25,
            "z": 0.550000011920929
        }
    },
    "walker.pedestrian.0010": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.25,
            "y": 0.25,
            "z": 0.550000011920929
        }
    },
    "walker.pedestrian.0011": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.25,
            "y": 0.25,
            "z": 0.550000011920929
        }
    },
    "walker.pedestrian.0012": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.6499999761581421
        }
    },
    "walker.pedestrian.0013": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.6499999761581421
        }
    },
    "walker.pedestrian.0014": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.6499999761581421
        }
    },
    "walker.pedestrian.0015": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0016": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0017": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0018": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0019": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0020": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0021": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0022": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0023": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0024": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0025": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0026": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0027": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0028": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0029": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0030": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0031": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0032": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0033": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0034": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0035": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0036": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0037": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0038": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0039": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0040": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0041": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0042": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0043": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0044": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0045": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0046": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0047": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0048": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.25,
            "y": 0.25,
            "z": 0.550000011920929
        }
    },
    "walker.pedestrian.0049": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.25,
            "y": 0.25,
            "z": 0.550000011920929
        }
    },
    "walker.pedestrian.0050": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    },
    "walker.pedestrian.0051": {
        "location": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "extent": {
            "x": 0.18767888844013214,
            "y": 0.18767888844013214,
            "z": 0.9300000071525574
        }
    }
}


CARLA_OBSTACLES = [
    carla.CityObjectLabel.Buildings,
    carla.CityObjectLabel.Walls,
    carla.CityObjectLabel.Fences,
    carla.CityObjectLabel.GuardRail,
    carla.CityObjectLabel.Vegetation,
    carla.CityObjectLabel.Poles,
    carla.CityObjectLabel.TrafficLight,
    carla.CityObjectLabel.TrafficSigns,
    carla.CityObjectLabel.Static,
    carla.CityObjectLabel.Other,
    carla.CityObjectLabel.Car,
    carla.CityObjectLabel.Truck,
    carla.CityObjectLabel.Bus,
    carla.CityObjectLabel.Train,
    carla.CityObjectLabel.Motorcycle,
    carla.CityObjectLabel.Bicycle
]


STR_RED = lambda x: f"\033[91m{x}\033[0m"

XYZRANGE: Dict[str, Dict[str, float]] = {
    "Town01": {
        "x": abs(-2.4200193881988525 - 396.6376037597656),
        "y": abs(-2.286572217941284 - 330.9415283203125),
        "z": abs(0.29999998211860657 - 0.5342468023300171)
    },
    "Town03": {
        "x": abs(-149.06358337402344 - 245.8651123046875),
        "y": abs(-208.17323303222656 - 207.50567626953125),
        "z": abs(0.27524489164352417 - 8.605596542358398)
    },
    "Town04": {
        "x": abs(-515.3493041992188 - 413.5663146972656),
        "y": abs(-396.1575622558594 - 340.2911376953125),
        "z": abs(0.2819424271583557 - 11.668004989624023)
    },
    "Town05": {
        "x": abs(-275.6298522949219 - 211.03936767578125),
        "y": abs(-207.7470245361328 - 208.752197265625),
        "z": abs(0.29999998211860657 - 0.6799139380455017)
    }
}
