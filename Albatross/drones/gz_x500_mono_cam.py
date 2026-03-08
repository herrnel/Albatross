from drone_base.drone_base import Drone
from core.types.command.action_message import Command
import time
import threading

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
        print("[phase] active: 500hz controller updating command")
        ctrl_thread = threading.Thread(
            target=self.modules.control_module.control_loop,
            args=(self.shared_data, self.stop_evt, 500.0, "scripted"),
            daemon=True,
        )
        ctrl_thread.start()
    
    def send_init(self) -> None: 
        self.send_thread = threading.Thread(target=command_loop, args=(mav, shared, stop_evt, t0, 50.0), daemon=True)
    
    def pump_init(self) -> None: 
        self.pump_thread = threading.Thread(target=pump_loop, args=(mav, shared, stop_evt), daemon=True)
        
    def print_init(self) -> None: 
        self.print_thread = threading.Thread(target=hb_print_loop, args=(shared, stop_evt), daemon=True)
        
    def send_start(self) -> None: 
        self.send_thread.start()
        
    def pump_start(self) -> None: 
        self.pump_thread.start()
    
    def print_start(self) -> None: 
        self.print_thread.start()
        
    