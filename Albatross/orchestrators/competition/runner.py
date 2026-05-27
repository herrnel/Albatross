import time
from core import Pipeline
from adapters import PlatformAdapter
from adapters import VisionAdapter
from drones import Drone

from colorama import Fore, Back, Style, init

class Runner:
    """
    This is where all the "phases" for the drone flight is defined. 
    """
        
    def setup(self, drone: Drone,  adapter: PlatformAdapter, vision_adapter: VisionAdapter, pipline: Pipeline) -> None:
        self.drone = drone
        self.adapter = adapter
        self.vision_adapter = vision_adapter
        self.pipeline = pipline 
        self._running = False

    def run(self):  
        print(f"{Fore.MAGENTA}I feel the need — the need for speed!{Style.RESET_ALL}")
              
        # Start up connection to Sim/Drone using adapter
        self.drone.adapter.connect()
        
        # Start up connect to Sim/Drone camera using adapter
        self.drone.vision_adapter.connect()
        
        # Seed an initial neutral command so command sender can start immediately
        self.drone.hover(3)
        
        # Prepare non-module threads.
        # These could probably exists in the drone and should be stored by the adapter. like Drone.pump_init(), Drone.send_init() Drone.print_init().
        print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}preparing replay logger and non-module threads{Style.RESET_ALL}")
        # Start recording replay system logs
        self.drone.pipeline.logger_init()
        self.drone.pipeline.pump_sensors_init()
        self.drone.pipeline.vision_sensors_init()
        self.drone.pipeline.send_heartbeat_init()
        self.drone.pipeline.send_command_init()
        self.drone.pipeline.hb_print_init()
        self.drone.pipeline.moh_print_init()
        
        try:
            
            # Start recording replay system logs
            self.drone.pipeline.logger_start()
            
            # Start primary threads
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}running non-module threads{Style.RESET_ALL}")
            self.drone.pipeline.pump_sensors_start() # Start collecting telemetry
            self.drone.pipeline.vision_sensors_start()
            self.drone.pipeline.send_heartbeat_start() # Start holding a minimum 2hz heartbeat stream with sim. 
            self.drone.pipeline.send_command_start() # Start streaming commands (if available)
            self.drone.pipeline.hb_print_start() # Start printing drone heartbeat/status flight data
            self.drone.pipeline.moh_print_start() # Start printing each module's operation health. 
            
            # Neutral commands, arming, and offboard requests may be unique to Mavlink and whould probably be moved to the adapter under one command.
            # Warmup: neutral streaming before arm/offboard
            
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}warmup: pause for neutral streaming{Style.RESET_ALL}")
            time.sleep(2.0)
            
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}ARM Drone (while streaming){Style.RESET_ALL}")
            self.drone.arm()
            time.sleep(0.5)
            
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}Request OFFBOARD (while streaming){Style.RESET_ALL}")
            self.drone.offboard()
            
            print(f"{Fore.MAGENTA}Talk to me, Goose{Style.RESET_ALL}")
   
            # 4. Initialize modules for neutral streaming, this should tell the modules that we are not in racing mode
            # we need the drones properllers to probably be spinning but nothing enough for take off. This activates everything except the control module.
            self.drone.start_processing()
            
            print(f"{Fore.MAGENTA}Ready for take off!{Style.RESET_ALL}")
                
            # 5. Next should be the takeoff bump this should be a consistent adapter command but with different implementaions 
            # Depending on the drone due to weight and motor difference. 
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}Takeoff bump: level attitude, higher thrust{Style.RESET_ALL}")
            self.drone.bump_and_run(1.0)
            
            
            print(f"{Fore.MAGENTA}C'mon Mav! Do some of that pilot shit!{Style.RESET_ALL}")
            
            # 6. Initiate Active Control at 120Hz
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}Activate controller {Style.RESET_ALL}")
            self.drone.take_control() # This should essentailly start the control module
            time.sleep(1.0)
            

            # 7. Initiate cooldown to neutral 
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}cooldown: neutral{Style.RESET_ALL}")
            self.drone.pipeline.stop_evt.set() 
            self.drone.hover(1.0)
            
            
            # 8. Start descending until we have landed. 
            while not self.drone.has_landed():
                self.drone.descend() # I need to check that stop_evt isn't blocking this descent. 
            
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}Drone has landed{Style.RESET_ALL}")
            
            
            # Wait while we are landed then:
            
            # 9. Requrest Disarm - TODO should only be disarming once we have landed the drone. 
            print(f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} {Fore.YELLOW}DISARM Drone{Style.RESET_ALL}")
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

        