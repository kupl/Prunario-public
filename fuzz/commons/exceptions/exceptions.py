from fuzz.commons.constants.fuzzing import NUM_TRIAL_MUTATION

### Testing Exceptions ###


class TestingException(Exception):
    pass


class ExceptionSimulation(TestingException):
    pass


class RunAgain(ExceptionSimulation):
    pass


class RunNextMutant(ExceptionSimulation):
    pass


class Exit(ExceptionSimulation):
    pass
### Testing Exceptions ###


### Cluster Exceptions ###
class ClusterException(Exception):
    pass


class DSLException(ClusterException):
    pass


class InvalidTokenException(DSLException):

    def __init__(self, token: str, feature: str) -> None:
        super().__init__(f'Invalid token "{token}" is provided to {feature}.')


class CarlaClientException(ClusterException):
    pass


class LoadWorldException(CarlaClientException):

    def __init__(self, _map: str) -> None:
        super().__init__(f'Error while loading world {_map}')


class InvalidClusteringMethod(ClusterException):

    def __init__(self, method_clustering: str) -> None:
        super().__init__(
            f'Invalid clustering method "{method_clustering}" is provided.')


class RecordException(ClusterException):
    pass


class NoRecordException(RecordException):

    def __init__(self, path: str) -> None:
        super().__init__(f'No record found from the path "{path}"!')


class RecordLoadingException(RecordException):

    def __init__(self, path: str) -> None:
        super().__init__(f'Cannot read state from "{path}"!')


class ReasoningException(ClusterException):
    pass


class EmptyReasonException(ReasoningException):

    def __init__(self, path_mb: str) -> None:
        super().__init__(f'Reason for {path_mb} is empty! ' +
                         'Some negative examples cound be in the positive examples. ' +
                         'Please check the labels.')
### Cluster Exceptions ###


### Planner Exceptions ###

class PlannerException(Exception):
    pass


class PlannerTimeoutException(PlannerException):

    def __init__(self, timeout: int) -> None:
        super().__init__(f'Planner Timeout({timeout}s)')

class PlanningAgain(PlannerException):

    def __init__(self, s: str) -> None:
        super().__init__(f'Planning failed: {s}')

class ModelNotTrained(PlannerException):
    pass

class DDSimulateFail(PlannerException):
     pass

### Planner Exceptions ###


### Mutation Exceptions ###
class TryMutationAgain(TestingException):

    def __init__(self) -> None:
        super().__init__(f'Mutation trial({NUM_TRIAL_MUTATION}) is over')


class InvalidMotionTypeException(TestingException):

    def __init__(self, mutation: str) -> None:
        super().__init__(f'Invalid mutation "{mutation}" is provided.')


class InvalidTargetException(TestingException):

    def __init__(self, target: str) -> None:
        super().__init__(f'Invalid target "{target}" is provided.')
### Mutation Exceptions ###


### Prediction Exceptions ###
class PredictionException(Exception):
    pass

class ExtractionException(PredictionException):
    pass
### Prediction Exceptions ###


class ROS2Timeout(Exception):
    pass