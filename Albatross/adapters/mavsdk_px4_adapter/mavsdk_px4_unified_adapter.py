# adapters/unified_px4_gz_adapter.py
from __future__ import annotations

from core.types.data import RawInput, Action
from adapters.mavsdk_px4_adapter.mavlink_telemetry_adapter import MavlinkTelemetryAdapter
from adapters.mavsdk_px4_adapter.gst_camera_adapter import GstCameraAdapter
from adapters.mavsdk_px4_adapter.mavlink_action_sender_adapter import MavlinkActionAdapter
from adapters.mavsdk_px4_adapter.mavlink_connection import make_mavlink_connection


class UnifiedPx4GzAdapter:
    """
    Wraps 3 adapters:
      - telemetry (MAVLink)
      - camera (GStreamer)
      - action sender (MAVLink)
    Provides a single interface identical to the competition pattern:
      read() -> RawInput
      send(action) -> None
    """

    def __init__(
        self,
        telemetry: MavlinkTelemetryAdapter,
        camera: GstCameraAdapter,
        sender: MavlinkActionAdapter,
        mav_endpoint="udpin:0.0.0.0:14540",
    ):
        self.mav_endpoint = mav_endpoint
        self.telemetry = telemetry
        self.camera = camera
        self.sender = sender

    def connect(self) -> None:
        # Create a connection with Gazebo using Mavlink
        self.mav = make_mavlink_connection(self.mav_endpoint)
        
        # Initialize the mavlink telemetry retriever and action sender
        self.telemetry = MavlinkTelemetryAdapter(self.mav)
        self.sender = MavlinkActionAdapter(self.mav)

        # Camera last (so if it fails you already know MAVLink is ok)
        self.camera.connect()

    def close(self) -> None:
        self.camera.close()
        self.telemetry.close()
        self.sender.close()

    def read(self) -> RawInput:
        frame = self.camera.read()
        tel = self.telemetry.read()
        return RawInput(frame=frame, telemetry=tel)

    def send(self, action: Action) -> None:
        self.sender.send(action)