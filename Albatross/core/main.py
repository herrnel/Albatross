from core.types.orchestrator.runner import Runner
from core.types.message_bus.bus_types import Bus

from adapters.gazebo_px4_adapter import GazeboPx4SimAdapter
# from modules.vision import VisionModule
# from modules.ekf import EkfModule
from core.policy.control_module import ControlModule

import sys

def main():
    """
    This should be running as fast has computation allows. 
    """

    bus = Bus()

    if len(sys.argv) > 1:
        adapter_param = sys.argv[1]
        
        if "Gazebo" == adapter_param :
            adapter = GazeboPx4SimAdapter()
        else:
            print("Adapter parameter not recognized.")
        
    else:
        print("No arguments provided. Using GazeboPx4SimAdapter")
        adapter = GazeboPx4SimAdapter()

    # Choose Modules
    modules = [
        # VisionModule(bus, hz=80),
        # EkfModule(bus, hz=400),
        ControlModule(bus, hz=500)
    ]

    # Create Orchestrator
    runner = Runner(adapter, bus, modules)

    # run system
    runner.run()


if __name__ == "__main__":
    main()