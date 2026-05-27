from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Literal, Optional


RecordType = Literal["topic_publish", "module_tick", "event"]


@dataclass(frozen=True)
class BaseRecord:
    run_id: str
    t_wall: float
    t_runtime_ns: int


@dataclass(frozen=True)
class TopicPublishRecord(BaseRecord):
    record_type: Literal["topic_publish"] = "topic_publish"
    topic: str = ""
    seq: int = -1
    t_ref_ns: int = 0
    t_publish_ns: int = 0
    source: str = ""
    source_local_tick: Optional[int] = None
    valid: bool = True
    input_seq_refs: dict[str, int] = field(default_factory=dict)
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ModuleTickRecord(BaseRecord):
    record_type: Literal["module_tick"] = "module_tick"
    module: str = ""
    local_tick: int = -1
    t_start_ns: int = 0
    t_end_ns: int = 0
    compute_ns: int = 0
    status: str = "ok"
    input_seq_refs: dict[str, int] = field(default_factory=dict)
    output_seq_refs: dict[str, int] = field(default_factory=dict)
    info: Optional[str] = None


@dataclass(frozen=True)
class EventRecord(BaseRecord):
    record_type: Literal["event"] = "event"
    name: str = ""
    payload: dict[str, Any] = field(default_factory=dict)


def to_json_dict(record: BaseRecord) -> dict[str, Any]:
    return asdict(record)


REPLAY_SOURCE_TOPICS = {
    "sensors.imu",
    "sensors.attitude",
    "sensors.local_pos",
    "sensors.odometry",
    "sensors.heartbeat_rx",
    "sensors.timesync",
    "sensors.camera_frame",
    "sensors.camera_info",
}