from __future__ import annotations

import time
from typing import Optional

import numpy as np

from adapters.vision.vision_base.vision_adapter import VisionAdapter
from core.types.shared_data.shared_state import SharedState
from core.types.shared_data.base_types.telemetry.camera_types import CameraFrame, CameraInfo

# PyGObject / GStreamer
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst


class GazeboGstVisionAdapter(VisionAdapter):
    """
    Vision adapter for the local PX4/Gazebo mono-camera setup using:
        RTP/H.264 over UDP -> GStreamer decode -> appsink -> NumPy frame

    Expected local setup:
    - Gazebo camera stream enters on 5600
    - external fan-out sends a copy to Python on 5601
    - this adapter listens on 5601

    Notes:
    - We use local receipt time as t_capture_ns because this setup does not
      provide authoritative capture timestamps by default.
    - Camera intrinsics are unknown unless provided separately.
    """

    def setup(
        self,
        shared_state: SharedState,
        udp_port: int = 5601,
        camera_name: str = "x500_mono_cam",
        width: Optional[int] = None,
        height: Optional[int] = None,
        pipeline_str: Optional[str] = None,
    ) -> None:
        self.shared_state = shared_state
        self.udp_port = udp_port
        self.camera_name = camera_name
        self.width = width
        self.height = height
        self.pipeline_str = pipeline_str

        self.emit_log_cb = None

        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsink = None

        self._frame_id = 0
        self._connected = False
        self._camera_info_published = False

        Gst.init(None)

    def store_logging_cb(self, emit_log_cb) -> None:
        self.emit_log_cb = emit_log_cb

    def connect(self) -> None:
        if self.pipeline_str is None:
            self.pipeline_str = self._build_pipeline(self.udp_port)

        print(f"INFO  [vision] connecting GStreamer UDP stream on port {self.udp_port}")
        pipeline = Gst.parse_launch(self.pipeline_str)

        appsink = pipeline.get_by_name("appsink0")
        if appsink is None:
            raise RuntimeError("Failed to find appsink named 'appsink0' in the pipeline.")

        # appsink behavior tuned for low latency:
        # - don't wait forever
        # - keep only the latest sample
        appsink.set_property("emit-signals", False)
        appsink.set_property("sync", False)
        appsink.set_property("drop", True)
        appsink.set_property("max-buffers", 1)

        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to set GStreamer pipeline to PLAYING state.")

        # Optional sanity check via bus for startup errors
        bus = pipeline.get_bus()
        msg = bus.timed_pop_filtered(
            1 * Gst.SECOND,
            Gst.MessageType.ERROR | Gst.MessageType.STATE_CHANGED
        )
        if msg is not None and msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            raise RuntimeError(f"GStreamer pipeline error: {err}; debug={debug}")

        self.pipeline = pipeline
        self.appsink = appsink
        self._connected = True

        # Try to pull one frame so we can infer width/height and verify the stream.
        initial = self._pull_sample(timeout_ns=2_000_000_000)
        if initial is not None:
            frame = self._sample_to_bgr(initial)
            if frame is not None:
                if self.height is None or self.width is None:
                    self.height, self.width = frame.shape[:2]
                self._publish_camera_info()
                self._publish_frame(frame)
                print(
                    f"INFO  [vision] stream opened successfully "
                    f"({self.width}x{self.height}) on port {self.udp_port}"
                )
                return

        # If we opened but did not decode an initial frame yet
        self._publish_camera_info()
        print("WARN  [vision] pipeline opened, but no initial frame was decoded yet")

    def disconnect(self) -> None:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.appsink = None
        self._connected = False

    def get_camera_info(self) -> Optional[CameraInfo]:
        info, _, _, _ = self.shared_state.sensors.camera_info.get()
        return info

    def pump_frames(self, max_frames: int = 1) -> int:
        if not self._connected or self.appsink is None:
            return 0

        count = 0
        while count < max_frames:
            sample = self._pull_sample(timeout_ns=1_000_000)  # 1 ms
            if sample is None:
                break

            frame = self._sample_to_bgr(sample)
            if frame is None:
                break

            if self.height is None or self.width is None:
                self.height, self.width = frame.shape[:2]
                self._publish_camera_info()

            self._publish_frame(frame)
            count += 1

        return count

    def _pull_sample(self, timeout_ns: int):
        if self.appsink is None:
            return None

        # Since emit-signals=False, we use pull methods directly.
        # appsink is designed for applications to retrieve samples this way.
        return self.appsink.emit("try-pull-sample", timeout_ns)

    def _sample_to_bgr(self, sample) -> Optional[np.ndarray]:
        caps = sample.get_caps()
        if caps is None:
            return None

        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        fmt = structure.get_value("format")

        if fmt != "BGR":
            raise RuntimeError(f"Expected BGR frames from appsink, got format={fmt}")

        buffer = sample.get_buffer()
        ok, map_info = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return None

        try:
            arr = np.frombuffer(map_info.data, dtype=np.uint8)
            expected = int(height) * int(width) * 3
            if arr.size < expected:
                return None
            frame = arr[:expected].reshape((int(height), int(width), 3)).copy()
            return frame
        finally:
            buffer.unmap(map_info)

    def _publish_camera_info(self) -> None:
        if self._camera_info_published:
            return
        if self.width is None or self.height is None:
            return

        now_ns = time.perf_counter_ns()
        info = CameraInfo(
            width=int(self.width),
            height=int(self.height),
            fx=None,
            fy=None,
            cx=None,
            cy=None,
            dist_coeffs=None,
            frame_id=self.camera_name,
        )

        info_seq = self.shared_state.sensors.camera_info.publish(info, t_ns=now_ns)
        self._camera_info_published = True

        if self.emit_log_cb is not None:
            self.emit_log_cb(
                "log_topic_publish",
                topic="sensors.camera_info",
                seq=info_seq,
                t_ref_ns=now_ns,
                t_publish_ns=now_ns,
                source="GazeboGstVisionAdapter",
                payload_obj=info,
                valid=True,
                input_seq_refs={},
            )

    def _publish_frame(self, frame_bgr: np.ndarray) -> None:
        now_ns = time.perf_counter_ns()

        msg = CameraFrame(
            t_capture_ns=now_ns,
            frame_id=self._frame_id,
            image_bgr=frame_bgr,
            camera_name=self.camera_name,
            valid=True,
        )

        frame_seq = self.shared_state.sensors.camera_frame.publish(msg, t_ns=now_ns)

        if self.emit_log_cb is not None:
            self.emit_log_cb(
                "log_topic_publish",
                topic="sensors.camera_frame",
                seq=frame_seq,
                t_ref_ns=msg.t_capture_ns,
                t_publish_ns=now_ns,
                source="GazeboGstVisionAdapter",
                payload_obj=msg,
                valid=msg.valid,
                input_seq_refs={},
            )

        self._frame_id += 1

    @staticmethod
    def _build_pipeline(port: int) -> str:
        """
        Local UDP RTP/H.264 -> decode -> BGR -> appsink.

        We explicitly convert to BGR so downstream NumPy/OpenCV code is easy.
        """
        return (
            f'udpsrc port={port} '
            f'caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! '
            f'rtpjitterbuffer latency=0 ! '
            f'rtph264depay ! h264parse ! avdec_h264 ! '
            f'videoconvert ! video/x-raw,format=BGR ! '
            f'appsink name=appsink0 drop=true max-buffers=1 sync=false'
        )