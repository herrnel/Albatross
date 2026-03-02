from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from core.types.data import Frame

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst


@dataclass
class GstCameraConfig:
    udp_port: int = 5601
    payload: int = 96  # if unspecified in caps, 96 is common; we can omit too
    timeout_s: float = 5.0


class GstCameraAdapter:
    """
    UDP RTP/H264 -> decoded RGB frames via GStreamer appsink -> numpy.
    Works even when OpenCV is built without GStreamer (your case).
    """

    def __init__(self, cfg: GstCameraConfig):
        self.cfg = cfg
        self._t0: Optional[float] = None

        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsink = None

        self._latest_frame: Optional[np.ndarray] = None

    def connect(self) -> None:
        Gst.init(None)
        self._t0 = time.time()

        # Your gst-launch works with caps that omit payload; we’ll match that.
        caps = (
            "application/x-rtp, "
            "media=(string)video, "
            "clock-rate=(int)90000, "
            "encoding-name=(string)H264"
        )

        pipeline_str = (
            f"udpsrc port={self.cfg.udp_port} reuse=true caps=\"{caps}\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=RGB ! "
            "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        if self.appsink is None:
            raise RuntimeError("Failed to create appsink from GStreamer pipeline.")

        self.appsink.connect("new-sample", self._on_new_sample)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start GStreamer pipeline (PLAYING).")

    def close(self) -> None:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)

    def read(self) -> Frame:
        """
        Returns the most recent decoded frame. Blocks until at least one arrives.
        """
        assert self._t0 is not None

        deadline = time.time() + self.cfg.timeout_s
        while self._latest_frame is None and time.time() < deadline:
            time.sleep(0.005)

        if self._latest_frame is None:
            raise RuntimeError(
                f"No frames received within {self.cfg.timeout_s}s from UDP port {self.cfg.udp_port}. "
                "Stream exists (gst-launch works), so this usually means caps mismatch or decode not available."
            )

        t = time.time() - self._t0
        # Copy so downstream code can safely mutate without racing updates
        return Frame(t=t, image=self._latest_frame.copy())

    def _on_new_sample(self, sink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        width = s.get_value("width")
        height = s.get_value("height")

        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            data = map_info.data  # bytes-like
            frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            self._latest_frame = frame
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK