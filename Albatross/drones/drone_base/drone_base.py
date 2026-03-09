from abc import ABC, abstractmethod
from core.types.telemetry.shared_state import SharedState
from adapters.adapter_base.platform_adapter import PlatformAdapter
from core.types.module.pipeline_type import Pipeline

import threading



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
    
    def __init__(self):
        # This is used by th runner to stop all loops in the case of catastrophic failure. 
        self.stop_evt = threading.Event() 
        
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
    