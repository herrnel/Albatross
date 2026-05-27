from core.types.shared_data.shared_state import SharedState
from core.pipeline import Pipeline
from replay.replay_runner import ReplayRunner

from core.modules.dummy_observation_module import DummyObservationModule
from core.modules.control_module import ControlModule
from core.modules.safety_module import SafetyModule


def main():
    shared_state = SharedState()

    modules = [
        DummyObservationModule(shared_state),
        ControlModule(shared_state),
        SafetyModule(shared_state),
    ]

    # No live adapters here
    pipeline = Pipeline(
        modules=modules,
        adapter=None,
        vision_adapter=None,
        shared_state=shared_state,
        enable_logging=False,
    )

    replay = ReplayRunner(
        shared_state=shared_state,
        pipeline=pipeline,
        run_dir="eval/runs/run_20260406_225532",
        mode="realtime",   # or "fast"
    )

    replay.run()


if __name__ == "__main__":
    main()