from __future__ import annotations

import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Optional

from eval.logging.schema import BaseRecord, TopicPublishRecord, ModuleTickRecord, EventRecord, to_json_dict
from eval.logging.serialize import jsonable, save_frame_image, save_npy_array


class RunLogger:
    def __init__(self, base_dir: str | Path, run_id: Optional[str] = None):
        self.base_dir = Path(base_dir)
        self.run_id = run_id or time.strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = self.base_dir / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.events_path = self.run_dir / "events.jsonl"
        self._fh = self.events_path.open("a", encoding="utf-8")
        self._lock = threading.Lock()

    def close(self) -> None:
        with self._lock:
            if not self._fh.closed:
                self._fh.flush()
                self._fh.close()

    def write_record(self, record: BaseRecord) -> None:
        payload = to_json_dict(record)
        with self._lock:
            self._fh.write(json.dumps(payload) + "\n")

    def log_event(self, name: str, payload: dict[str, Any] | None = None) -> None:
        now_wall = time.time()
        now_ns = time.perf_counter_ns()
        rec = EventRecord(
            run_id=self.run_id,
            t_wall=now_wall,
            t_runtime_ns=now_ns,
            name=name,
            payload=jsonable(payload or {}),
        )
        self.write_record(rec)

    def log_topic_publish(
        self,
        *,
        topic: str,
        seq: int,
        t_ref_ns: int,
        t_publish_ns: int,
        source: str,
        payload_obj: Any,
        source_local_tick: int | None = None,
        valid: bool = True,
        input_seq_refs: dict[str, int] | None = None,
    ) -> None:
        payload = self._serialize_topic_payload(topic=topic, seq=seq, payload_obj=payload_obj)

        rec = TopicPublishRecord(
            run_id=self.run_id,
            t_wall=time.time(),
            t_runtime_ns=time.perf_counter_ns(),
            topic=topic,
            seq=seq,
            t_ref_ns=t_ref_ns,
            t_publish_ns=t_publish_ns,
            source=source,
            source_local_tick=source_local_tick,
            valid=valid,
            input_seq_refs=input_seq_refs or {},
            payload=payload,
        )
        self.write_record(rec)

    def log_module_tick(
        self,
        *,
        module: str,
        local_tick: int,
        t_start_ns: int,
        t_end_ns: int,
        status: str,
        input_seq_refs: dict[str, int] | None = None,
        output_seq_refs: dict[str, int] | None = None,
        info: str | None = None,
    ) -> None:
        rec = ModuleTickRecord(
            run_id=self.run_id,
            t_wall=time.time(),
            t_runtime_ns=time.perf_counter_ns(),
            module=module,
            local_tick=local_tick,
            t_start_ns=t_start_ns,
            t_end_ns=t_end_ns,
            compute_ns=max(0, t_end_ns - t_start_ns),
            status=status,
            input_seq_refs=input_seq_refs or {},
            output_seq_refs=output_seq_refs or {},
            info=info,
        )
        self.write_record(rec)

    def _serialize_topic_payload(self, *, topic: str, seq: int, payload_obj: Any) -> dict[str, Any]:
        """
        Store large payloads like frames separately and keep JSONL lightweight.
        """
        # Camera frames: save image to file and log metadata only
        if topic == "sensors.camera_frame":
            image = payload_obj.image_bgr
            rel_path = save_frame_image(self.run_dir, seq=seq, image_bgr=image)
            return {
                "frame_id": int(payload_obj.frame_id),
                "camera_name": payload_obj.camera_name,
                "valid": bool(payload_obj.valid),
                "image_path": rel_path,
                "height": int(image.shape[0]),
                "width": int(image.shape[1]),
            }

        # Default: dataclass/ndarray/jsonable conversion
        return jsonable(payload_obj)