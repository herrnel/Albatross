import argparse

from drones import gz_x500_mono_cam
from adapters import GazeboPx4MavlinkAdapter
from adapters import GazeboGstVisionAdapter
from core import Pipeline
from core.types.shared_data.shared_state import SharedState
from orchestrators import FoundationRunner


def extract_params():
    parser = argparse.ArgumentParser(description="Run Albatross foundation flight test")
    parser.add_argument(
        "drone",
        type=str,
        choices=["gz_x500_mono_cam"],
    )
    parser.add_argument(
        "adapter",
        type=str,
        choices=["GazeboPx4"],
    )
    args = parser.parse_args()

    match args.drone:
        case "gz_x500_mono_cam":
            drone = gz_x500_mono_cam()
        case _:
            raise ValueError("Unsupported drone")

    match args.adapter:
        case "GazeboPx4":
            adapter = GazeboPx4MavlinkAdapter()
            vision_adapter = GazeboGstVisionAdapter()
        case _:
            raise ValueError("Unsupported adapter")

    runner = FoundationRunner()
    return drone, adapter, vision_adapter, runner


def main():
    drone, adapter, vision_adapter, runner = extract_params()

    shared_state = SharedState()

    # No modules for foundation test
    modules = []

    adapter.setup(shared_state)

    # We keep the vision adapter object around to match your setup shape,
    # but the foundation runner intentionally does not connect/start vision.
    vision_adapter.setup(
        shared_state=shared_state,
        udp_port=5601,
        camera_name="x500_mono_cam",
    )

    pipeline = Pipeline(
        modules,
        adapter,
        vision_adapter,
        shared_state,
        enable_logging=True,
    )

    drone.setup(adapter, vision_adapter, pipeline)
    runner.setup(drone, adapter, vision_adapter, pipeline)
    runner.run()


if __name__ == "__main__":
    main()