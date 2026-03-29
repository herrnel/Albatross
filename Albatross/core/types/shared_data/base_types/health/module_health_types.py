from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Optional


HealthStatus = Literal["init", "ok", "stale", "fault"]


@dataclass(frozen=True)
class ModuleHealth:
    """
    Runtime health snapshot for one module.

    Purpose:
    - track whether the module is alive
    - expose its intended rate
    - expose the last time it reported health
    - provide a short status + debug message

    Notes:
    - age is usually computed by the reader from last_tick_time_ns
    - this object is for runtime supervision, not algorithm correctness
    """
    name: str
    hz_target: float
    last_tick_time_ns: Optional[int]
    status: HealthStatus
    info: Optional[str] = None

    def age_ms(self, now_ns: int) -> Optional[float]:
        if self.last_tick_time_ns is None:
            return None
        return (now_ns - self.last_tick_time_ns) / 1_000_000.0


@dataclass(frozen=True)
class LinkHealth:
    """
    Health snapshot for platform/transport connectivity.
    """
    t_ns: int
    connected: bool
    last_heartbeat_rx_age_ms: Optional[float]
    last_heartbeat_tx_age_ms: Optional[float]
    info: Optional[str] = None