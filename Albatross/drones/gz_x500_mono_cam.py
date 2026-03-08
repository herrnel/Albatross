from drone_base.drone_base import Drone
from core.types.command.action_message import Command
import time


class gz_x500_mono_cam(Drone):
    """
    Every drone may contain its own information. This class will allow us to have specific
    Information about this drone and expose the core functionality to the Runner. 
    Drone -> Modules -> SharedData -> Adapter
    """
    
    def setup(self, adapter, shared_state, modules) -> None: 
        self.adapter = adapter
        self.shared_data = shared_state
        self.modules = modules
        self.hover_thrust = .87
        
    def neutral_command(self, adapter, shared_state, modules) -> None: 
        ...
    
    def arm(self) -> None: 
        self.adapter.request_arm()
        
                
    def disarm(self) -> None: 
        self.adapter.request_disarm()
    
    def cooldown(self) -> None: 
        self.adapter.set_command(
            Command(
                roll=0.0,
                pitch=0.0,
                yaw_angle=0.0,
                yaw_rate=0.0,
                thrust=self.hover_thrust,
                t=time.perf_counter(),
            )
        )
        

    def bump_and_run(self) -> None: 
        ...
    
    def send_init(self) -> None: 
        ...
        
    def print_init(self) -> None: 
        ...
        
    
    def pump_init(self) -> None: 
        ...
        
        
    def send_start(self) -> None: 
        ...
        
    def print_start(self) -> None: 
        ...
        
    
    def pump_start
    