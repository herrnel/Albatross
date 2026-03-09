import argparse
from core.orchestrator.runner import Runner
from core.types.module.pipeline_type import Pipeline
from drones.gz_x500_mono_cam import gz_x500_mono_cam
from adapters.gazebo_px4_adapter.gazebo_px4_sim_adapter import GazeboPx4MavlinkAdapter
# from modules.vision import VisionModule
# from modules.ekf import EkfModule
from core.policy.control_module import ControlModule
from core.types.telemetry.shared_state import SharedState



def extract_params():
    parser = argparse.ArgumentParser(description="calculate X to the power of Y")
    parser.add_argument("drone", type=str, 
                        help="Provide a real drone or sim/real drone name. This will help adjust the automation stack to the drones characteristics", 
                        choices=["gz_x500_mono_cam"])
    parser.add_argument("adapter",
                        type=str, 
                        help="Provide an adapter which could be a simulator and/or communication protocol", 
                        choices=[
                            # "AirSim", # TODO
                            "GazeboPx4", 
                            # "Issac" # TODO
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
            
    return (drone, adapter)

def main():
    """
    This should be running as fast has computation allows. 
    """
    
    drone, adapter = extract_params()
    
    # Choose Modules
    modules = [
        # VisionModule(hz=80),
        # EkfModule(hz=400),
        ControlModule(hz=500)
    ]
    
    # 2. Create a shared state object that is thread safe from multiple modules reading and writing to it. 
    # This shared object will be the drone essentially for this project. It will be the abstraction of the 
    # Drone we care about. 
    shared_state = SharedState()

    pipeline = Pipeline(modules, adapter, shared_state)

    # Create an orchestrator
    runner = Runner(drone, pipeline, adapter)

    # run system
    runner.run()


if __name__ == "__main__":
    main()