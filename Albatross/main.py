# main.py
import time

from core.types.data import Action
from adapters.mavsdk_px4_adapter.mavlink_telemetry_adapter import MavlinkTelemetryAdapter, TelemetryEmulation
from adapters.mavsdk_px4_adapter.gst_camera_adapter import GstCameraAdapter, GstCameraConfig
from adapters.mavsdk_px4_adapter.mavlink_action_sender_adapter import MavlinkActionSender, ActionSenderConfig
from adapters.mavsdk_px4_adapter.mavsdk_px4_unified_adapter import UnifiedPx4GzAdapter


def main():
    # Telemetry: connect to SITL endpoint
    tel = MavlinkTelemetryAdapter(
        connection="udp:127.0.0.1:14540",
        emu=TelemetryEmulation(start_pos_only=True, start_pos_duration_s=1.0)
    )

    # Camera: assumes RTP/H264 on UDP 5600
    cam = GstCameraAdapter(GstCameraConfig(udp_port=5600))

    # Sender: also connects to MAVLink and sends commands
    sender = MavlinkActionSender(ActionSenderConfig(connection="udp:127.0.0.1:14540"))

    adapter = UnifiedPx4GzAdapter(telemetry=tel, camera=cam, sender=sender)
    adapter.connect()

    # Optional: best-effort offboard setup
    # sender.set_offboard_mode()
    # sender.arm()

    # Control loop
    hz = 30.0
    dt = 1.0 / hz
    try:
        while True:
            raw = adapter.read()

            # Dummy policy: hover-ish (you’ll replace with your stack)
            action = Action(
                t=raw.telemetry.t,
                throttle=0.55,
                roll=0.0,
                pitch=0.0,
                yaw=0.0
            )

            adapter.send(action)
            time.sleep(dt)
    finally:
        adapter.close()


if __name__ == "__main__":
    main()