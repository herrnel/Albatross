import time

from adapters.adapter_base.platform_adapter import PlatformAdapter
from drones.drone_base.drone_base import Drone
from core.types.telemetry.shared_state import SharedState




class Runner:
    """
    Here make an attempt to have the runner loop run at 500hz but its not necessary. 
    This should 
    """
    def __init__(self, drone: Drone,  adapter: PlatformAdapter, modules: list):
        self.drone = drone
        self.adapter = adapter
        self.modules = modules
        self._running = False

    def run(self, rate_hz: float = 500.0):
        period = 1.0 / rate_hz
        next_t = time.perf_counter()
        
        # 1. Start up connection using adapter
        self.adapter.start()
        
        # 2. Create a shared state object that is thread safe from multiple modules reading and writing to it. 
        # This shared object will be the drone essentially for this project. It will be the abstraction of the 
        # Drone we care about. 
        self.shared = SharedState()
        # Initial the drone which wraps the SharedState and holds a drones configuration i.e weight etc. 
        # Initialize the drone using the adapter, SharedState and modules
        self.drone.setup(self.adapter, self.shared, self.modules)
        t0 = time.time() # Timer for tracking Hz
        
        
        # Seed an initial neutral command so sender can start immediately
        self.drone.neutral_command()
        
        # 3. Initialize sensor reading i.e inititializng the pump_sensor loop to start reading independently
        # These could probably exists in the drone and should be stored by the adapter. like Drone.pump_init(), Drone.send_init() Drone.print_init(). 
        # pump_thread = threading.Thread(target=pump_loop, args=(mav, shared, stop_evt), daemon=True)
        # send_thread = threading.Thread(target=command_loop, args=(mav, shared, stop_evt, t0, 50.0), daemon=True)
        # print_thread = threading.Thread(target=hb_print_loop, args=(shared, stop_evt), daemon=True)
    
        self.drone.pump_init()
        self.drone.send_init()
        self.drone.print_init()

        try:
            
            self.drone.pump_start()
            self.drone.send_start()
            self.drone.print_start()
            
            
            # Neutral commands, arming, and offboard requests may be unique to Mavlink and whould probably be moved to the adapter under one command.
            # Warmup: neutral streaming before arm/offboard
            
            print("[phase] warmup: neutral streaming")
            time.sleep(2.0)
            
            print("[cmd] ARM (while streaming)")
            self.drone.arm()
            time.sleep(0.5)
            
            print("[cmd] OFFBOARD (while streaming)")
            self.drone.offboard
   
            # 4. Initialize modules for neutral streaming, this should tell the modules that we are not in racing mode
            # we need the drones properllers to probably be spinning but nothing enough for take off. Everyting except the control module.
            for m in self.modules:
                # We pass the drone which contains the shared data to each of the modules which they will all simulatnously
                # use together. This start should 
                m.start(self.drone)
                
                
            # 5. Next should be the takeoff bump this should be a consistent adapter command but with different implementaions 
            # Depending on the drone due to weight and motor difference. 
            print("[phase] takeoff bump: level attitude, higher thrust")
            self.drone.bump_and_run()
            time.sleep(1.5)
            
            # 6. Initiate Active Control at 500Hz
            print("[phase] active: 500hz controller updating command")
            self.drone.start() # This should essentailly start the control module
                
            # 7. Initiate cooldown to neutral 
            print("[phase] cooldown: neutral")
            self.drone.cooldown()
            
            # 8. Requrest Disarm
            print("[cmd] DISARM")
            self.drone.disarm()
            
        finally: 
            self.drone.stop_evt.set()
            time.sleep(0.3)

            # best effort neutral stream one last time
            try:
                for _ in range(10):
                    # This is giving us direct access to an adapter command to regain control. 
                    self.drone.adapter.send_attitude_target(
                        t0,
                        roll=0.0,
                        pitch=0.0,
                        yaw_angle=0.0,
                        yaw_rate=0.0,
                        thrust=self.drone.hover_thrust,
                    )
                    time.sleep(0.02)
            except Exception:
                pass

            
            

        # self._running = True
        # try:
        #     while self._running:
        #         # 1) ingest and update bus
        #         self.adapter.pump_sensors(self.bus)

        #         # 2) apply latest action (if any)
        #         act = self.bus.action_latest.get()
        #         if act is not None:
        #             self.adapter.apply_action(act)

        #         # fixed-rate scheduling
        #         now = time.perf_counter()
        #         if now < next_t:
        #             time.sleep(next_t - now)
        #         next_t += period
        # finally:
        #     for m in self.modules:
        #         m.stop()
        #     self.adapter.stop()