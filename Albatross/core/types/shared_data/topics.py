from __future__ import annotations

import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Generic, Optional, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class TopicEntry(Generic[T]):
    seq: int
    t_ns: int
    value: T


class LastValueTopic(Generic[T]):
    """
    Latest-value cache with per-topic lock.
    Good for low-rate products where consumers only need the newest valid value.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._entry: Optional[TopicEntry[T]] = None
        self._seq = 0

    def publish(self, value: T, t_ns: Optional[int] = None) -> int:
        if t_ns is None:
            t_ns = time.perf_counter_ns()
        with self._lock:
            self._seq += 1
            self._entry = TopicEntry(seq=self._seq, t_ns=t_ns, value=value)
            return self._seq

    def get_entry(self) -> Optional[TopicEntry[T]]:
        with self._lock:
            return self._entry

    def get(self):
        entry = self.get_entry()
        if entry is None:
            return None, None, None
        return entry.value, entry.t_ns, entry.seq

    def age_ms(self, now_ns: Optional[int] = None) -> Optional[float]:
        entry = self.get_entry()
        if entry is None:
            return None
        if now_ns is None:
            now_ns = time.perf_counter_ns()
        return (now_ns - entry.t_ns) / 1_000_000.0

    def is_fresh(self, max_age_ms: float, now_ns: Optional[int] = None) -> bool:
        age = self.age_ms(now_ns=now_ns)
        return age is not None and age <= max_age_ms


class RingBufferTopic(Generic[T]):
    """
    Small bounded history with per-topic lock.
    Good for high-rate streams or time-window queries.
    """

    def __init__(self, maxlen: int):
        self._lock = threading.Lock()
        self._buf: Deque[TopicEntry[T]] = deque(maxlen=maxlen)
        self._seq = 0

    def publish(self, value: T, t_ns: Optional[int] = None) -> int:
        if t_ns is None:
            t_ns = time.perf_counter_ns()
        with self._lock:
            self._seq += 1
            self._buf.append(TopicEntry(seq=self._seq, t_ns=t_ns, value=value))
            return self._seq

    def latest(self) -> Optional[TopicEntry[T]]:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def latest_n(self, n: int) -> list[TopicEntry[T]]:
        with self._lock:
            return list(self._buf)[-n:]

    def between_ns(self, t0_ns: int, t1_ns: int) -> list[TopicEntry[T]]:
        with self._lock:
            return [entry for entry in self._buf if t0_ns <= entry.t_ns <= t1_ns]

    def age_ms(self, now_ns: Optional[int] = None) -> Optional[float]:
        entry = self.latest()
        if entry is None:
            return None
        if now_ns is None:
            now_ns = time.perf_counter_ns()
        return (now_ns - entry.t_ns) / 1_000_000.0

    def is_fresh(self, max_age_ms: float, now_ns: Optional[int] = None) -> bool:
        age = self.age_ms(now_ns=now_ns)
        return age is not None and age <= max_age_ms