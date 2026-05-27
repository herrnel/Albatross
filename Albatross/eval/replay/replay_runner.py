from __future__ import annotations

import time

from replay.replay_loader import ReplayLoader
from adapters.platform.replay_platform_adapter.replay_platform_adapter import ReplayPlatformAdapter
from adapters.vision.replay_vision_adapter.replay_vision_adapter import ReplayVisionAdapter


class ReplayRunner:
    def __init__(self, shared_state, pipeline, run_dir: str, mode: str = "realtime"):
        self.shared_state = shared_state
        self.pipeline = pipeline
        self.loader = ReplayLoader(run_dir)
        self.platform_adapter = ReplayPlatformAdapter(shared_state)
        self.vision_adapter = ReplayVisionAdapter(shared_state, run_dir)
        self.mode = mode

    def run(self):
        records = list(self.loader.iter_replay_inputs())
        if not records:
            raise RuntimeError("No replay input records found.")

        t0_log_ns = records[0].t_ref_ns
        t0_wall_ns = time.perf_counter_ns()

        # Start modules, but not live adapters
        self.pipeline.start_processing()
        self.pipeline.take_control()

        for record in records:
            if self.mode == "realtime":
                dt_ns = record.t_ref_ns - t0_log_ns
                target_ns = t0_wall_ns + dt_ns
                while time.perf_counter_ns() < target_ns:
                    time.sleep(0)

            if record.topic.startswith("sensors.camera"):
                self.vision_adapter.publish_record(record)
            else:
                self.platform_adapter.publish_record(record)