from __future__ import annotations

from pathlib import Path
import cv2

from core.types.telemetry.camera_types import CameraFrame, CameraInfo


class ReplayVisionAdapter:
    def __init__(self, shared_state, run_dir: str | Path):
        self.shared_state = shared_state
        self.run_dir = Path(run_dir)

    def publish_record(self, record):
        topic = record.topic
        p = record.payload

        if topic == "sensors.camera_info":
            obj = CameraInfo(**p)
            self.shared_state.sensors.camera_info.publish(obj, t_ns=record.t_publish_ns)

        elif topic == "sensors.camera_frame":
            image_path = self.run_dir / p["image_path"]
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            obj = CameraFrame(
                t_capture_ns=record.t_ref_ns,
                frame_id=p["frame_id"],
                image_bgr=image,
                camera_name=p.get("camera_name"),
                valid=p.get("valid", True),
            )
            self.shared_state.sensors.camera_frame.publish(obj, t_ns=record.t_publish_ns)