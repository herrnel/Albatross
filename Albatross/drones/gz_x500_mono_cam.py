import time
from drones.drone_base.drone_base import Drone
from core import Pipeline
from adapters import PlatformAdapter
from core.types import Command


class gz_x500_mono_cam(Drone):
    """
    Every drone may contain its own information. This class will allow us to store specific
    information about each drone and expose the core drone functionality to any orchestrator. 
    Drones should primarly interact with Adapters, Pipelines, and ShareState. 
    """
    
    def setup(self, adapter: PlatformAdapter, pipeline: Pipeline) -> None: 
        self.adapter: PlatformAdapter = adapter
        self.pipeline: Pipeline = pipeline
        # Thrust tuning
        self.hover_thrust = 0.74
        self.takeoff_thrust = .87
           
        
    # Core functionality
    
    def start_processing(self) -> None: 
        self.pipeline.start_processing()
    
    def take_control(self) -> None:
        self.pipeline.take_control()
    
    def arm(self) -> None: 
        """
        Arm the drone using an adapter. 
        """
        self.adapter.request_arm()

    def offboard(self) -> None: 
        """
        Send a request to the simulator or controller offboard access using an adapter. 
        """
        self.adapter.request_offboard()
                
    def disarm(self) -> None: 
        """
        Disarm the drone using an adapter.
        """
        self.adapter.request_disarm()
    
    def hover(self) -> None: 
        """
        Direct command to get the drone to stop and hover using an adapter.
        """
        self.pipeline.shared_state.set_command(
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
        """
        Direct command to the drone to take off and fly forward using an adapter. 
        """
        self.pipeline.shared_state.set_command(
            Command(
                roll=0.0,
                pitch=0.0,
                yaw_angle=0.0,
                yaw_rate=0.0,
                thrust=self.takeoff_thrust,
                t=time.perf_counter(),
            )
        )
        
    
    