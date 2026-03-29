from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SafeAction:
    """
    Post-safety control-layer output.

    This is the policy action after stale-data handling, confidence gating, and
    safety clamping. It is still conceptually above the final MAVLink transport.
    """
    seq: int
    t_ns: int

    throttle: float
    roll: float
    pitch: float
    yaw: float

    confidence: float = 1.0
    override: bool = False
    reason: Optional[str] = None
    source: str = "safety"

    action_seq_ref: Optional[int] = None