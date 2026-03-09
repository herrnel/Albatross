import time

from adapters.adapter_base.platform_adapter import PlatformAdapter
from drones.drone_base.drone_base import Drone
from core.types.module.pipeline_type import Pipeline

class Runner:
    """
    Here make an attempt to have the runner loop run at 500hz but its not necessary. 
    This should 
    """
    def __init__(self, drone: Drone,  adapter: PlatformAdapter, pipline: Pipeline):
        self.drone = drone
        self.adapter = adapter
        self.pipeline = pipline 
        self._running = False

    def run(self, rate_hz: float = 500.0):        
        # 1. Start up connection to Sim/Drone using adapter
        self.drone.adapter.connect()
        
        # Initial the drone which wraps the SharedState and holds a drones configuration i.e weight etc. 
        # Initialize the drone using the adapter, SharedState and modules
        self.drone.setup(self.adapter, self.pipeline)
        
        # Seed an initial neutral command so sender can start immediately
        self.drone.hover()
        
        # 3. Initialize sensor reading i.e inititializng the pump_sensor loop to start reading independently
        # These could probably exists in the drone and should be stored by the adapter. like Drone.pump_init(), Drone.send_init() Drone.print_init(). 
    
        self.drone.pipeline.pump_init()
        self.drone.pipeline.send_init()
        self.drone.pipeline.print_init()

        try:
            
            # Start primary threads
            self.drone.pipeline.pump_start() # Start collecting telemetry
            self.drone.pipeline.send_start() # Start sending commands (if available)
            self.drone.pipeline.print_start() # Start logging flight data
            
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
            self.drone.start_processing()
                
                
            # 5. Next should be the takeoff bump this should be a consistent adapter command but with different implementaions 
            # Depending on the drone due to weight and motor difference. 
            print("[phase] takeoff bump: level attitude, higher thrust")
            self.drone.bump_and_run()
            time.sleep(1.5)
            
            # 6. Initiate Active Control at 500Hz
            print("[phase] active: 500hz controller updating command")
            self.drone.take_control() # This should essentailly start the control module
            time.sleep(3.0)
            

            # 7. Initiate cooldown to neutral 
            print("[phase] cooldown: neutral")
            self.drone.hover()
            time.sleep(1.0)
            
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
                    self.drone.hover()
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