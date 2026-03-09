from drone_base.drone_base import Drone
from core.types.command.action_message import Command
from core.types.module.pipeline_type import Pipeline
import time
import threading

class gz_x500_mono_cam(Drone):
    """
    Every drone may contain its own information. This class will allow us to have specific
    Information about this drone and expose the core functionality to the Runner. 
    Drone -> Modules -> SharedData -> Adapter
    """
    
    def setup(self, adapter, shared_state, pipeline: Pipeline) -> None: 
        self.adapter = adapter
        self.shared_data = shared_state
        self.pipeline = pipeline
        self.hover_thrust = .87
        self.takeoff_thrust = .97
          
    def neutral_command(self) -> None: 
        self.shared_data.set_command(
            Command(
                roll=0.0,
                pitch=0.0,
                yaw_angle=0.0,
                yaw_rate=0.0,
                thrust=self.hover_thrust,
                t=time.perf_counter(),
            )
        )
    
    def arm(self) -> None: 
        self.adapter.request_arm()

    def offboard(self) -> None: 
        self.adapter.request_offboard()
                
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
        self.shared_data.set_command(
            Command(
                roll=0.0,
                pitch=0.0,
                yaw_angle=0.0,
                yaw_rate=0.0,
                thrust=self.takeoff_thrust,
                t=time.perf_counter(),
            )
        )
        
    def activate_control(self) -> None:

    
    