# main.py
import time

from core.types.data import Action
from adapters.mavsdk_px4_adapter.gst_camera_adapter import GstCameraAdapter, GstCameraConfig
from adapters.mavsdk_px4_adapter.mavsdk_px4_unified_adapter import UnifiedPx4GzAdapter


def main():
    # One shared MAVLink endpoint.
    # PX4 log says Onboard remote port 14540, so we listen there.
    # IMPORTANT: use udpin/udp to BIND ONCE.
    adapter = UnifiedPx4GzAdapter(
        mav_endpoint="udpin:0.0.0.0:14540",
    )
    adapter.connect()

    # (Optional) If you implemented offboard/arm helpers inside adapter.sender:
    # adapter.sender.begin_offboard_and_arm()

    hz = 30.0
    dt = 1.0 / hz
    try:
        while True:
            raw = adapter.read()

            # dummy policy
            action = Action(
                t=raw.telemetry.t,
                throttle=0.55,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
            )

            adapter.send(action)
            time.sleep(dt)
    finally:
        adapter.close()


if __name__ == "__main__":
    main()