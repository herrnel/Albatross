import time
import threading
from adapters import PlatformAdapter
from core.utilities import monotonic_sleep_until
from core.types import Module, SharedState


class Pipeline:
    """
    Purpose is to initialize, organize, store, and sychronize the pipeline, control, and message threads. 
    """
    modules: dict[str, Module]

    def __init__(self, modules: list[Module], adapter: PlatformAdapter, shared_state: SharedState):
        self.modules = {}
        self.threads = {}
        self.modules_list = modules
        self.adapter = adapter
        self.shared_state = shared_state
        self.stop_evt = threading.Event() # stop_evt is a thread-safe shutdown signal. It’s how multiple loops know when to stop running.
        self.t0 = time.time() # Timer for tracking Hz

        # Store modules in a dictionary for easier access. 
        for module in modules:
            self.modules[module.name] = module
            setattr(self, module.name + "_module", module)
        
    def start_processing(self) -> None:
        for name, module in self.modules.items():
            
            if name != "control" : # We want to initiate the control module later
                print(f"[Phase] Starting {name} Module")
                self.threads[name] = module.create_thread(self.stop_evt)
            
    def take_control(self) -> None:
        print(f"[Phase] Starting control Module!")
        self.ctrl_thread = threading.Thread(
            target=self.control_module.control_loop,
            args=(self.stop_evt, 500.0),
            daemon=True,
        )
        self.ctrl_thread.start()
        
    # --- Core Send Thread Logic --- 
    def send_init(self) -> None: 
        self.threads["send"] = threading.Thread(target=self.command_loop, args=(50.0,), daemon=True)

    def send_start(self) -> None: 
        self.threads["send"].start()
    
    
    def command_loop(
        self,
        send_hz: float = 50.0,  # Use a higher rate to avoid any “offboard loss” sensitivity
    ):
        """
        Dedicated setpoint stream loop. Inbetween new commands the previous one should be ran.
        PX4 offboard generally needs continuous streaming.
        """
        dt = 1.0 / send_hz
        next_t = time.perf_counter()
        last_print = 0.0

        while not self.stop_evt.is_set():
            cmd = self.shared_state.get_command()

            self.adapter.send_attitude_target(
                t0=self.t0,
                roll=cmd.roll,
                pitch=cmd.pitch,
                yaw_angle=cmd.yaw_angle,
                yaw_rate=cmd.yaw_rate,
                thrust=cmd.thrust,
            )

            # Print out a hearbeat and get the cmd being currently streamed. 
            now_wall = time.time()
            if now_wall - last_print > 1.0:
                hb = self.shared_state.get_heartbeat()
                armed_str = "unknown"
                if hb is not None:
                    armed_str = str(self.adapter.is_armed_from_heartbeat(hb))

                print(
                    "[stream] "
                    f"roll={cmd.roll:.3f} "
                    f"pitch={cmd.pitch:.3f} "
                    f"yaw_angle={cmd.yaw_angle:.3f} "
                    f"yaw_rate={cmd.yaw_rate:.3f} "
                    f"thrust={cmd.thrust:.3f} "
                    f"armed={armed_str}"
                )
                last_print = now_wall

            next_t += dt
            monotonic_sleep_until(next_t)
        
    
    # --- Core Pump Thread Logic ---
    def pump_init(self) -> None: 
        self.threads["pump"] = threading.Thread(target=self.pump_loop, daemon=True)
        
    def pump_start(self) -> None: 
        self.threads["pump"].start()
        
    def pump_loop(self):
        """
        Dedicated IO loop: drain telemetry as fast as practical.
        """
        while not self.stop_evt.is_set():
            drained = self.adapter.pump_sensors(max_msgs=500)

            # tiny sleep so we don't busy-spin at 100% CPU
            if drained == 0:
                time.sleep(0.0005)

    # --- Core Print Thread Logic  ---
    def print_init(self) -> None: 
        self.threads["print"] = threading.Thread(target=self.hb_print_loop, daemon=True)
         
    def print_start(self) -> None: 
        self.threads["print"].start()
    
    def hb_print_loop(self):
        while not self.stop_evt.is_set():
            hb = self.shared_state.get_heartbeat()
            pos = self.shared_state.get_local_pos()

            if hb is not None:
                print(
                    f"[hb] armed={self.adapter.is_armed_from_heartbeat(hb)} "
                    f"base_mode={hb.base_mode} custom_mode={hb.custom_mode}"
                )
            if pos is not None:
                print(f"[pos] x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}")

            time.sleep(1.0)
        