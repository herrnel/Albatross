import time
from Albatross.core.types.archive.data import Action
from adapters.unified_px4_gz_adapter import UnifiedPx4GzAdapter


def sleep_to_rate(next_t: float, dt: float) -> float:
    now = time.time()
    if now < next_t:
        time.sleep(next_t - now)
    return next_t + dt


def main():
    adapter = UnifiedPx4GzAdapter()
    adapter.start()

    # Offboard warmup + arm (warmup streams neutral setpoints first)
    assert adapter.sender is not None
    adapter.sender.begin_offboard_and_arm(warmup_s=1.0, hz=50.0)

    hz = 100.0
    dt = 1.0 / hz
    next_t = time.time()

    # Simple state: wait for first camera frame, but keep streaming safe commands
    saw_first_frame = False

    try:
        while True:
            tel = adapter.latest_telemetry()
            raw = adapter.latest()  # Optional[RawInput]

            if raw is None:
                # Camera not ready yet -> keep drone stable, keep streaming setpoints
                action = Action(
                    t=tel.t,
                    throttle=0.0,  # no forward
                    roll=0.0,      # no lateral
                    pitch=0.0,     # no vertical (NED down)
                    yaw=0.0
                )
            else:
                if not saw_first_frame:
                    print("[info] first camera frame received")
                    saw_first_frame = True

                # TODO: policy(raw) -> Action
                action = Action(
                    t=raw.telemetry.t,
                    throttle=0.6,  # forward speed fraction
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.0
                )

            adapter.send(action)  # ALWAYS send every tick
            next_t = sleep_to_rate(next_t, dt)

    finally:
        adapter.close()


if __name__ == "__main__":
    main()