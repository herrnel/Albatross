from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Action:
    """
    Internal control-layer output.

    This is what the policy/control module wants to do before safety shaping and
    final command mapping.
    """
    seq: int
    t_ns: int

    throttle: float
    roll: float
    pitch: float
    yaw: float

    confidence: float = 1.0
    source: str = "unknown"

    # Useful for traceability in logs/replay
    observation_seq_ref: Optional[int] = None