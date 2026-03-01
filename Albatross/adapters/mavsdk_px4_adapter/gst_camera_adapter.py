# adapters/gst_camera_adapter.py
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from core.types.data import Frame


@dataclass
class GstCameraConfig:
    udp_port: int = 5600
    use_h264_rtp: bool = True


class GstCameraAdapter:
    def __init__(self, cfg: Optional[GstCameraConfig] = None, pipeline: Optional[str] = None):
        self.cfg = cfg or GstCameraConfig()
        self._t0: Optional[float] = None
        self.cap: Optional[cv2.VideoCapture] = None

        if pipeline is None:
            # RTP/H264 is the common Gazebo GStreamer output pattern
            pipeline = (
                f"udpsrc port={self.cfg.udp_port} "
                "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! "
                "rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false"
            )
        self.pipeline = pipeline

    def connect(self) -> None:
        self._t0 = time.time()
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(
                "Failed to open camera stream. Likely wrong port/encoding OR OpenCV lacks GStreamer.\n"
                f"Pipeline:\n{self.pipeline}"
            )

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()

    def read(self) -> Frame:
        assert self._t0 is not None and self.cap is not None, "Call connect() first."
        ok, img = self.cap.read()
        if not ok or img is None:
            raise RuntimeError("Failed to read camera frame.")
        t = time.time() - self._t0
        if img.dtype != np.uint8:
            img = img.astype(np.uint8, copy=False)
        return Frame(t=t, image=img)