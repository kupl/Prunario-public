from abc import abstractmethod
from typing import Any, Dict, List

from fuzz.data import Scenario


class ADS:

    def __init__(self) -> None:
        self.id: str = ''
        self.bp: str = ''
        self.version: str = ''

    @abstractmethod
    def connect(self,
                scenario: Scenario,
                timeout: int,
                id_gpu: int) -> bool:
        pass
    
    @abstractmethod
    def set_goal(self, mission: Dict[str, Dict[str, float]]) -> bool:
        pass
    
    @abstractmethod
    def engage(self) -> bool:
        pass
    
    @abstractmethod
    def start_recording(self) -> int:
        pass
    
    @abstractmethod
    def finish_recording(self, pid_record: int) -> bool:
        pass

    @abstractmethod
    def restart(self) -> None:
        pass

    @abstractmethod
    def plan_trajectory(self, mission: Dict[str, Dict[str, float]]) -> List[Any]:
        pass