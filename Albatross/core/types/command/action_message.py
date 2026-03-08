from dataclasses import dataclass
import numpy as np
from typing import Optional


@dataclass(frozen=True)
class Command:
    t: float = 0.0
    roll: float = 0.0          # rad or normalized (define once)
    pitch: float = 0.0         # rad or normalized
    yaw_rate: float = 0.0      # rad/s or normalized
    throttle: float = 0.0      # 0..1