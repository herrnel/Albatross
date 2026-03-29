import time
from core import Pipeline
from adapters.platform import PlatformAdapter
from drones import Drone

class Runner:
    """
    Here make an attempt to have the runner loop run at 500hz but its not necessary. 
    This should 
    """
        
    def setup(self, drone: Drone,  adapter: PlatformAdapter, pipline: Pipeline) -> None:
        self.drone = drone
        self.adapter = adapter
        self.pipeline = pipline 
        self._running = False

    def run(self):        
        # Start up connection to Sim/Drone using adapter
        self.drone.adapter.connect()
        
        # Seed an initial neutral command so command sender can start immediately
        self.drone.hover()
        
        # Prepare non-module threads.
        # These could probably exists in the drone and should be stored by the adapter. like Drone.pump_init(), Drone.send_init() Drone.print_init(). 
        self.drone.pipeline.pump_sensors_init()
        self.drone.pipeline.send_heartbeat_init()
        self.drone.pipeline.send_command_init()
        self.drone.pipeline.hb_print_init()
        self.drone.pipeline.moh_print_init()

        try:
            
            # Start primary threads
            self.drone.pipeline.pump_sensors_start() # Start collecting telemetry
            self.drone.pipeline.send_heartbeat_start() # Start holding a minimum 2hz heartbeat stream with sim. 
            self.drone.pipeline.send_command_start() # Start streaming commands (if available)
            self.drone.pipeline.hb_print_start() # Start printing drone heartbeat/status flight data
            self.drone.pipeline.moh_print_start() # Start printing each module's operation health. 
            
            # Neutral commands, arming, and offboard requests may be unique to Mavlink and whould probably be moved to the adapter under one command.
            # Warmup: neutral streaming before arm/offboard
            
            print("[phase] warmup: neutral streaming")
            time.sleep(2.0)
            
            print("[cmd] ARM (while streaming)")
            self.drone.arm()
            time.sleep(0.5)
            
            print("[cmd] OFFBOARD (while streaming)")
            self.drone.offboard()
   
            # 4. Initialize modules for neutral streaming, this should tell the modules that we are not in racing mode
            # we need the drones properllers to probably be spinning but nothing enough for take off. This activates everything except the control module.
            self.drone.start_processing()
                
                
            # 5. Next should be the takeoff bump this should be a consistent adapter command but with different implementaions 
            # Depending on the drone due to weight and motor difference. 
            print("[phase] takeoff bump: level attitude, higher thrust")
            self.drone.bump_and_run()
            time.sleep(3)
            
            # 6. Initiate Active Control at 500Hz
            print("[phase] active: 500hz controller updating command")
            self.drone.take_control() # This should essentailly start the control module
            time.sleep(3.0)
            

            # 7. Initiate cooldown to neutral 
            print("[phase] cooldown: neutral")
            self.drone.hover()
            time.sleep(1.0)
            
            
            # Wait while we are landed then:
            
            # 8. Requrest Disarm
            print("[cmd] DISARM")
            self.drone.disarm()
            
        finally: 
            self.drone.pipeline.stop_evt.set()
            time.sleep(0.3)

            # best effort neutral stream one last time
            try:
                for _ in range(10):
                    # This is giving us direct access to an adapter command to regain control. 
                    self.drone.hover()
                    time.sleep(0.02)
            except Exception:
                pass

        