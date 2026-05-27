import argparse
from orchestrators import Runner
from drones import gz_x500_mono_cam
from adapters import GazeboPx4MavlinkAdapter
from adapters import GazeboGstVisionAdapter
from core import Pipeline
from core.modules.dummy_observation_module import DummyObservationModule
from core.modules.control_module import ControlModule
from core.modules.safety_module import SafetyModule
# from modules.vision import VisionModule
# from modules.ekf import EkfModule

from core.types.shared_data.shared_state import SharedState


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
            vision_adapter = GazeboGstVisionAdapter()
        case _:
            print("")
    
    match args.runner:
        case "Runner":
            runner = Runner()
        case _:
            print("")
    
    return (drone, adapter, vision_adapter, runner)

def main():
    """
    This should be running as fast has computation allows. 
    """
    # Get flight configuration
    drone, adapter, vision_adapter, runner = extract_params()
    
    # Create a shared state object that is thread safe from multiple modules reading and writing to it. 
    shared_state = SharedState()
    
    
    # Choose Modules
    modules = [
        # GateDetectionModule
        # GateGeometryModule
        # StateEstimationModule
        # ObservationModule
        # PolicyModule
        # SafteyModule()
        DummyObservationModule(shared_state),
        ControlModule(shared_state),
        SafetyModule(shared_state),       # whatever simple version you have
    ]
    
    # Setup platform adapter
    adapter.setup(shared_state)
    
    # Setup Vision adapter
    vision_adapter.setup(
        shared_state=shared_state,
        udp_port=5601,
        camera_name="x500_mono_cam",
    )
    
    # Initialize the module manager 
    pipeline = Pipeline(modules, adapter, vision_adapter, shared_state, enable_logging=True)
    
    # Setup drone 
    drone.setup(adapter, vision_adapter, pipeline)

    # Setup the flight orchestrator
    runner.setup(drone, adapter, vision_adapter, pipeline)

    runner.run()


if __name__ == "__main__":
    main()