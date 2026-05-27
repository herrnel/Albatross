from __future__ import annotations

import json
from pathlib import Path
from typing import Iterator, Optional

from logging.schema import TopicPublishRecord, ModuleTickRecord, EventRecord, REPLAY_SOURCE_TOPICS


class ReplayLoader:
    def __init__(self, run_dir: str | Path):
        self.run_dir = Path(run_dir)
        self.events_path = self.run_dir / "events.jsonl"

    def iter_raw_records(self) -> Iterator[dict]:
        with self.events_path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                yield json.loads(line)

    def iter_records(self) -> Iterator[TopicPublishRecord | ModuleTickRecord | EventRecord]:
        for raw in self.iter_raw_records():
            rtype = raw["record_type"]
            if rtype == "topic_publish":
                yield TopicPublishRecord(**raw)
            elif rtype == "module_tick":
                yield ModuleTickRecord(**raw)
            elif rtype == "event":
                yield EventRecord(**raw)
            else:
                raise ValueError(f"Unknown record_type: {rtype}")

    def iter_topic_records(self, topic_prefix: Optional[str] = None) -> Iterator[TopicPublishRecord]:
        for rec in self.iter_records():
            if isinstance(rec, TopicPublishRecord):
                if topic_prefix is None or rec.topic.startswith(topic_prefix):
                    yield rec

    def iter_replay_inputs(self) -> Iterator[TopicPublishRecord]:
        for rec in self.iter_topic_records():
            if rec.topic in REPLAY_SOURCE_TOPICS:
                yield rec