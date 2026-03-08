from abc import ABC, abstractmethod
from core.types.telemetry.shared_state import SharedState
from adapters.adapter_base.platform_adapter import PlatformAdapter
from core.types.module.module_type import Module

import threading



class Drone(ABC):
    """
    Drone parent class. Defines common fields and methods that all drones should have. 
    The Drone class should also be the place where all threads are handled. 
    """

    modules: list[Module]
    shared_data: SharedState
    adapter: PlatformAdapter
    hover_thrust: int
    takeoff_thrust: int
    send_thread: threading # Continous Sending of Telemetry
    pump_thread: threading # Continous Reading of Telemetry
    print_thread: threading 
    
    def __init__(self):
        # This is used by th runner to stop all loops in the case of catastrophic failure. 
        self.stop_evt = threading.Event()
        
        
    @abstractmethod
    def setup(self, adapter, shared_state, modules) -> None: 
        ...
        
    @abstractmethod
    def neutral_command(self, adapter, shared_state, modules) -> None: 
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
    def cooldown(self) -> None: 
        ...
        
    @abstractmethod
    def activate_control(self) -> None:
        ...

    @abstractmethod
    def bump_and_run(self) -> None: 
        ...
    
    @abstractmethod
    def send_init(self) -> None: 
        ...
        
    @abstractmethod
    def print_init(self) -> None: 
        ...
        
    
    @abstractmethod
    def pump_init(self) -> None: 
        ...
        
        
    @abstractmethod
    def send_start(self) -> None: 
        ...
        
    @abstractmethod
    def print_start(self) -> None: 
        ...
        
    
    @abstractmethod
    def pump_start(self) -> None: 
        ...