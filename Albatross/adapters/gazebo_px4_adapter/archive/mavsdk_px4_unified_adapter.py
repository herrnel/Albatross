from __future__ import annotations

from typing import Optional

from Albatross.core.types.archive.data import RawInput, Action
from adapters.mavsdk_px4_adapter.mavlink_telemetry_adapter import MavlinkTelemetryAdapter, TelemetryEmulation
from adapters.mavsdk_px4_adapter.gst_camera_adapter import GstCameraAdapter, GstCameraConfig
from adapters.mavsdk_px4_adapter.mavlink_action_sender_adapter import MavlinkActionSenderAdapter, ActionSenderConfig
from adapters.mavsdk_px4_adapter.mavlink_connection import make_mavlink_connection


class UnifiedPx4GzAdapter:
    """
    Continuous streaming adapter.
    - telemetry.latest() non-blocking
    - camera.latest() non-blocking (may be None at startup)
    - sender.send() should be called at fixed Hz forever
    """

    def __init__(
        self,
        mav_endpoint: str = "udpin:0.0.0.0:14540",
        camera_udp_port: int = 5601,
        telemetry_emu: Optional[TelemetryEmulation] = None,
        sender_cfg: Optional[ActionSenderConfig] = None,
    ):
        self.mav_endpoint = mav_endpoint
        self.camera_udp_port = camera_udp_port
        self.telemetry_emu = telemetry_emu
        self.sender_cfg = sender_cfg

        self.mav = None
        self.telemetry: Optional[MavlinkTelemetryAdapter] = None
        self.camera: Optional[GstCameraAdapter] = None
        self.sender: Optional[MavlinkActionSenderAdapter] = None

    def start(self) -> None:
        self.mav = make_mavlink_connection(self.mav_endpoint)

        self.telemetry = MavlinkTelemetryAdapter(self.mav, emu=self.telemetry_emu)
        self.sender = MavlinkActionSenderAdapter(self.mav, cfg=self.sender_cfg)
        self.camera = GstCameraAdapter(GstCameraConfig(udp_port=self.camera_udp_port))

        self.telemetry.start()
        self.camera.connect()

    def close(self) -> None:
        if self.camera:
            self.camera.close()
        if self.telemetry:
            self.telemetry.close()
        if self.sender:
            self.sender.close()

    def latest_telemetry(self):
        assert self.telemetry is not None
        return self.telemetry.latest()

    def latest_frame(self):
        assert self.camera is not None
        return self.camera.latest()

    def latest(self) -> Optional[RawInput]:
        """
        Returns RawInput only when a camera frame is available.
        This keeps Frame.image non-optional per your types.
        """
        assert self.telemetry is not None and self.camera is not None
        frame = self.camera.latest()
        if frame is None:
            return None
        tel = self.telemetry.latest()
        return RawInput(frame=frame, telemetry=tel)

    def send(self, action: Action) -> None:
        assert self.sender is not None
        self.sender.send(action)