import time
import threading
from adapters.platform.adapter_base.platform_adapter import PlatformAdapter
from core.types.shared_data.shared_state import SharedState
from core.types.command.command_type import Command
from core.modules.module_base.module_type import ModuleBase


from colorama import Fore, Back, Style, init


class Pipeline:
    """
    Purpose is to initialize, organize, store, and sychronize the pipeline, control, and message threads. 
    """
    modules: dict[str, ModuleBase]

    def __init__(self, modules: list[ModuleBase], adapter: PlatformAdapter, shared_state: SharedState):
        self.modules = {}
        self.threads = {}
        self.modules_list = modules
        self.adapter = adapter
        self.shared_state = shared_state
        self.stop_evt = threading.Event() # stop_evt is a thread-safe shutdown signal. It’s how multiple threads know when to stop running.
        self.t0 = time.time() # Timer for tracking Hz

        # Store modules in a dictionary for easier access. 
        for module in modules:
            self.modules[module.name] = module
            setattr(self, module.name + "_module", module)
        
    def start_processing(self) -> None:
        """
            This method initializes all of the **processing module** threads and the **safety module** thread. The exception is the `control module`
        """
        
        for name, module in self.modules.items():
            
            if name != "control" : # We want to initiate the control module later
                print(f"INFO  {Fore.LIGHTYELLOW_EX}[stack]{Style.RESET_ALL} {Fore.YELLOW}Running {name} module{Style.RESET_ALL}")
                
                # Create a thread for the module. Note that each module must implement the ModuleBase class
                self.threads[name] = module.create_thread(self.stop_evt) 
            
    def take_control(self) -> None:
        """
            This method creates the **Control Module** thread. By this point, all other processing modules and the safety module should be running. 
            Note: The control module does not compute `Commands` but rather an intermediary commands called `Actions` which is then consumed by the Safety Module which in turn publishes `Commands`.
            `Commands` are the actual commands sent to the platform. 
            To change the _Hz_ in which this **control module** runs you can change it in `main.py`
        """
        
        print(f"INFO  {Fore.LIGHTYELLOW_EX}[stack]{Style.RESET_ALL} {Fore.YELLOW}Running control module{Style.RESET_ALL}")
        self.ctrl_thread = self.control_module.create_thread(self.stop_evt)
        self.ctrl_thread.start()
        
    # -----------------------------
    # Operational Threads
    # -----------------------------
    
    # --- Core Send Heartbeat thread logic - Operational Thread -- 
    
    def send_heartbeat_init(self) -> None: 
        self.threads["send_heartbeat"] = threading.Thread(target=self.send_heartbeat_loop, kwargs={"heartbeat_hz": 2.0}, daemon=True) # Once this works, I’d change the default from 2.0 to 3.0 or 5.0 ust for a bit more margin

    def send_heartbeat_start(self) -> None: 
        self.threads["send_heartbeat"].start()
    
    
    def send_heartbeat_loop(
        self,
        heartbeat_hz: float = 2.0,
    ):
        """
        Dedicated client heartbeat TX loop.

        Responsibilities:
        - periodically send a MAVLink heartbeat through the adapter
        - keep publishing client heartbeat_tx state for monitoring/debugging
        - optionally print a low-rate status line

        Assumptions:
        - self.adapter.send_heartbeat() exists
        - self.shared_state.sensors.heartbeat_tx is updated by adapter.send_heartbeat()
        - self.stop_evt exists
        """
        dt_ns = int(1e9 / heartbeat_hz)
        next_ns = time.perf_counter_ns()
        last_print_ns = 0

        while not self.stop_evt.is_set():
            now_ns = time.perf_counter_ns()

            try:
                self.adapter.send_heartbeat()
            except Exception as e:
                print(f"{Fore.YELLOW}WARN{Style.RESET_ALL} [hb_tx] send failed: {e}")

            # Optional once-per-second debug print
            if (now_ns - last_print_ns) >= 1_000_000_000:
                hb_tx, hb_tx_t_ns, hb_tx_seq = self.shared_state.sensors.heartbeat_tx.get()
                hb_rx, hb_rx_t_ns, hb_rx_seq = self.shared_state.sensors.heartbeat_rx.get()

                tx_age_ms = self.shared_state.sensors.heartbeat_tx.age_ms(now_ns=now_ns)
                rx_age_ms = self.shared_state.sensors.heartbeat_rx.age_ms(now_ns=now_ns)

                print(
                    f"INFO  {Fore.LIGHTYELLOW_EX}[hb_tx]{Style.RESET_ALL} "
                    f"tx_seq={hb_tx_seq if hb_tx is not None else 'none'} "
                    f"tx_age_ms={tx_age_ms if tx_age_ms is not None else 'n/a'} "
                    f"rx_seq={hb_rx_seq if hb_rx is not None else 'none'} "
                    f"rx_age_ms={rx_age_ms if rx_age_ms is not None else 'n/a'}"
                )
                last_print_ns = now_ns

            next_ns += dt_ns
            self.sleep_until_ns(next_ns)
    
        
    # --- Core Send Command Thread Logic - Operational Thread --- 
    
    #------------
    # TODO: I need to make it clear here that the SafetyModule will most likely be the one computing the Commands in the SharedState. NOT a ControlModule Policy.
    #------------
    
    def send_command_init(self) -> None: 
        self.threads["send_command"] = threading.Thread(target=self.send_command_loop, args=(50.0,), daemon=True)

    def send_command_start(self) -> None: 
        self.threads["send_command"].start()
    
    
    def send_command_loop(
        self,
        send_hz: float = 50.0,
    ):
        """
        Dedicated outbound command stream loop.

        Behavior:
        - Continuously re-stream the latest available command.
        - If no command exists yet, keep sending a neutral/default command.
        - PX4/offboard-style control generally expects continuous streaming.

        Assumptions:
        - self.shared_state.control.command is a LastValueTopic[Command]
        - self.shared_state.sensors.heartbeat_rx is a LastValueTopic[HeartbeatData]
        - self.adapter.send_command(cmd) exists
        """
        dt_ns = int(1e9 / send_hz)
        next_ns = time.perf_counter_ns()
        last_print_ns = 0

        # Seed fallback command if nothing has been published yet.
        last_cmd = Command(
            roll=0.0,
            pitch=0.0,
            yaw_angle=0.0,
            yaw_rate=0.0,
            thrust=0.55,
            t=time.perf_counter_ns() / 1e9,
        )

        while not self.stop_evt.is_set():
            now_ns = time.perf_counter_ns()

            # Read latest command topic
            cmd, cmd_t_ns, cmd_seq = self.shared_state.control.command.get()

            # If nothing is available yet, keep reusing the last known command
            if cmd is not None:
                last_cmd = cmd

            # Send using the adapter boundary, not field-by-field here
            self.adapter.send_command(last_cmd)

            # Optional: print a low-rate stream summary
            if (now_ns - last_print_ns) >= 1_000_000_000:
                hb, hb_t_ns, hb_seq = self.shared_state.sensors.heartbeat_rx.get()
                status, status_t_ns, status_seq = self.shared_state.sensors.system_status.get()

                armed_str = "unknown"
                mode_str = "unknown"
                hb_age_ms = None
                cmd_age_ms = self.shared_state.control.command.age_ms(now_ns=now_ns)

                if status is not None:
                    armed_str = str(status.armed)
                    mode_str = str(status.mode)
                elif hb is not None:
                    armed_str = str(hb.armed)
                    mode_str = str(hb.mode)

                if hb is not None:
                    hb_age_ms = self.shared_state.sensors.heartbeat_rx.age_ms(now_ns=now_ns)

                print(
                    f"INFO  {Fore.LIGHTYELLOW_EX}[stream]{Style.RESET_ALL} "
                    f"cmd_seq={cmd_seq if cmd is not None else 'none'} "
                    f"roll={last_cmd.roll:.3f} "
                    f"pitch={last_cmd.pitch:.3f} "
                    f"yaw_angle={last_cmd.yaw_angle:.3f} "
                    f"yaw_rate={last_cmd.yaw_rate:.3f} "
                    f"thrust={last_cmd.thrust:.3f} "
                    f"cmd_age_ms={cmd_age_ms if cmd_age_ms is not None else 'n/a'} "
                    f"hb_age_ms={hb_age_ms if hb_age_ms is not None else 'n/a'} "
                    f"armed={armed_str} "
                    f"mode={mode_str}"
                )
                last_print_ns = now_ns

            next_ns += dt_ns
            self.sleep_until_ns(next_ns)
        
    
    # --- Core Pump Thread Logic - Operational Thread ---
    
    def pump_sensors_init(self) -> None: 
        self.threads["pump"] = threading.Thread(target=self.pump_sensors_loop, daemon=True)
        
    def pump_sensors_start(self) -> None: 
        self.threads["pump"].start()
        
    def pump_sensors_loop(self):
        """
        Dedicated IO loop: drain telemetry as fast as practical.
        """
        while not self.stop_evt.is_set():
            drained = self.adapter.pump_sensors(max_msgs=500)

            # tiny sleep so we don't busy-spin at 100% CPU
            if drained == 0:
                time.sleep(0.0005)
                
    # -----------------------------
    # Printing Threads
    # -----------------------------

    # --- Drone Heartbeat/Status/Position - Print Thread  ---
    def hb_print_init(self) -> None: 
        self.threads["hb_print"] = threading.Thread(target=self.hb_print_loop, daemon=True)
         
    def hb_print_start(self) -> None: 
        self.threads["hb_print"].start()
    
    def hb_print_loop(self):
        while not self.stop_evt.is_set():
            now_ns = time.perf_counter_ns()

            hb, hb_t_ns, hb_seq = self.shared_state.sensors.heartbeat_rx.get()
            status, _, _ = self.shared_state.sensors.system_status.get()
            pos, pos_t_ns, pos_seq = self.shared_state.sensors.local_pos.get()

            if hb is not None:
                hb_age_ms = self.shared_state.sensors.heartbeat_rx.age_ms(now_ns=now_ns)
                print(
                    f"INFO  {Fore.LIGHTYELLOW_EX}[hb_rx]{Style.RESET_ALL} age_ms={hb_age_ms:.1f} "
                    f"armed={hb.armed} "
                    f"mode=({hb.mode}) "
                    f"base_mode={hb.base_mode} "
                    f"custom_mode={hb.custom_mode}"
                )
                
            if status is not None:
                print(
                    f"INFO  {Fore.LIGHTYELLOW_EX}[status]{Style.RESET_ALL} armed={status.armed} "
                    f"offboard={status.offboard_enabled} "
                    f"mode={status.mode} "
                    f"failsafe={status.failsafe}"
                )

            if pos is not None:
                pos_age_ms = self.shared_state.sensors.local_pos.age_ms(now_ns=now_ns)
                print(
                    f"INFO  {Fore.LIGHTYELLOW_EX}[pos]{Style.RESET_ALL} age_ms={pos_age_ms:.1f} "
                    f"x={pos.x_m:.2f} y={pos.y_m:.2f} z={pos.z_m:.2f}"
                )

            time.sleep(1.0)

    # --- Every Module's Operational Health - Print Thread ---
    
    def moh_print_init(self) -> None: 
        self.threads["moh_print"] = threading.Thread(target=self.moh_print_loop, daemon=True)
         
    def moh_print_start(self) -> None: 
        self.threads["moh_print"].start()
    
    def moh_print_loop(self): # In the future it might be nice to add a paramter to limit which modules get their Health Printed out. 
        while not self.stop_evt.is_set():
            health_map, _, _ = self.shared_state.health.module_health.get()
            now_ns = time.perf_counter_ns()

            if health_map:
                for name, health in health_map.items():
                    age_ms = None
                    if health.last_tick_time_ns is not None:
                        age_ms = (now_ns - health.last_tick_time_ns) / 1_000_000.0
                    print(f"INFO  {Fore.LIGHTYELLOW_EX}[moh]{Style.RESET_ALL} {name}: status={health.status}, age_ms={age_ms}, info={health.info}")

            time.sleep(1.0)
                
    # -----------------------------
    # Helpers
    # -----------------------------
    
    def sleep_until_ns(self, target_ns: int) -> None:
        while True:
            now_ns = time.perf_counter_ns()
            remaining_ns = target_ns - now_ns
            if remaining_ns <= 0:
                return

            # Coarse sleep first, then short yield
            if remaining_ns > 2_000_000:  # > 2 ms
                time.sleep((remaining_ns - 1_000_000) / 1e9)
            else:
                time.sleep(0)