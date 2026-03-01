# adapters/unified_px4_gz_adapter.py
from __future__ import annotations

from core.types.data import RawInput, Action
from adapters.mavsdk_px4_adapter.mavlink_telemetry_adapter import MavlinkTelemetryAdapter
from adapters.mavsdk_px4_adapter.gst_camera_adapter import GstCameraAdapter, GstCameraConfig
from adapters.mavsdk_px4_adapter.mavlink_action_sender_adapter import MavlinkActionSenderAdapter
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
        mav_endpoint="udpin:0.0.0.0:14540",
    ):
        self.mav_endpoint = mav_endpoint
        # self.config = config
        self.telemetry = None
        self.camera = None
        self.sender = None

    def connect(self) -> None:
        # Create a connection with Gazebo using Mavlink
        self.mav = make_mavlink_connection(self.mav_endpoint)
        
        # Camera: RTP/H264 on UDP 5600 (adjust if your stream uses a different port)
        self.camera = GstCameraAdapter(GstCameraConfig(udp_port=5600))
        
        # Initialize the mavlink telemetry retriever and action sender
        self.telemetry = MavlinkTelemetryAdapter(self.mav)
        self.sender = MavlinkActionSenderAdapter(self.mav)

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