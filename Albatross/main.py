import argparse
from orchestrators import Runner
from drones import gz_x500_mono_cam
from adapters import GazeboPx4MavlinkAdapter
from core import Pipeline, ControlModule
# from modules.vision import VisionModule
# from modules.ekf import EkfModule

from core.types import SharedState


def extract_params():
    parser = argparse.ArgumentParser(description="Run Albatross Autonomy Stack")
    parser.add_argument("drone", type=str, 
                        help="Provide a real drone or sim/real drone name. This will help the automation stack adapt to the drone's characteristics.", 
                        choices=["gz_x500_mono_cam"])
    parser.add_argument("adapter",
                        type=str, 
                        help="Provide an adapter which could be a simulator and/or communication protocol", 
                        choices=[
                            "GazeboPx4", 
                            # "AirSim", # TODO
                            # "Issac" # TODO
                            ])
    parser.add_argument("runner",
                        type=str, 
                        help="Provide an adapter which could be a simulator and/or communication protocol", 
                        choices=[
                                "Runner",
                                # "xyz_Trainer"
                            ])
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-v", 
                        "--verbosity", 
                        help="increase output verbosity", 
                        action="count", default=0)
    group.add_argument("-q", "--quiet", action="store_true")
    
    args = parser.parse_args()
    if args.verbosity == 2:
        print("highest verbosity turned on")
    elif args.verbosity == 1:
        print("average verbosity turned on")
    else:
        print("")
        
    
    match args.drone:
        case "gz_x500_mono_cam":
            drone = gz_x500_mono_cam()
        case _:
            print("")
    
    
    match args.adapter:
        case "GazeboPx4":
            adapter = GazeboPx4MavlinkAdapter()
        case _:
            print("")
    
    match args.runner:
        case "Runner":
            runner = Runner()
        case _:
            print("")
    
    return (drone, adapter, runner)

def main():
    """
    This should be running as fast has computation allows. 
    """
    
    drone, adapter, runner = extract_params()
    
    # Create a shared state object that is thread safe from multiple modules reading and writing to it. 
    # This shared object will be the drone essentially for this project. It will be the abstraction of the 
    # Drone we care about. 
    shared_state = SharedState()
    
    # Choose Modules
    modules = [
        # VisionModule(shared_state, hz=80),
        # EkfModule(shared_state, hz=400),
        ControlModule(shared_state, hz=500)
    ]
    

    # Initialize the module manager 
    pipeline = Pipeline(modules, adapter, shared_state)
    
    # Setup drone 
    drone.setup(adapter, pipeline)

    # set up the flight orchestrator
    runner.setup(drone, pipeline, adapter)

    runner.run()


if __name__ == "__main__":
    main()