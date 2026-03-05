from core.types.orchestrator.runner import Runner
from core.types.message_bus.bus_types import Bus

from adapters.gazebo_px4_adapter import GazeboPx4SimAdapter
# from modules.vision import VisionModule
# from modules.ekf import EkfModule
from modules.control import ControlModule

def main():

    bus = Bus()

    # choose platform
    adapter = GazeboPx4SimAdapter()

    # create modules
    modules = [
        # VisionModule(bus, hz=80),
        # EkfModule(bus, hz=400),
        ControlModule(bus, hz=500)
    ]

    # create orchestrator
    runner = Runner(adapter, bus, modules)

    # run system
    runner.run()


if __name__ == "__main__":
    main()