from abc import ABC, abstractmethod
from core.types.shared_data.shared_state import SharedState
from adapters.platform import PlatformAdapter
from core.pipeline import Pipeline

class Drone(ABC):
    """
    Drone parent class. Defines common fields and methods that all drones should have. 
    The Drone class should also be the place where all threads are handled. 
    """

    pipeline: Pipeline
    shared_data: SharedState
    adapter: PlatformAdapter
    hover_thrust: int
    takeoff_thrust: int
    
    
    @abstractmethod
    def setup(self, adapter: PlatformAdapter, pipeline: Pipeline) -> None: 
        ...
        
    @abstractmethod
    def setup(self, adapter, pipeline: Pipeline) -> None: 
        ...
        
    @abstractmethod
    def start_processing(self) -> None: 
        ...
        
    @abstractmethod
    def take_control(self) -> None:
        ...
        
    @abstractmethod
    def arm(self) -> None: 
        ...
        
    @abstractmethod
    def offboard(self) -> None: 
        ...
        
    @abstractmethod           
    def disarm(self) -> None: 
        ...
        
    @abstractmethod
    def hover(self) -> None: 
        ...
        
    @abstractmethod
    def bump_and_run(self) -> None: 
        ...
    