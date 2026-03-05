from dataclasses import dataclass
import numpy as np
from typing import Optional


@dataclass(frozen=True)
class ActionMsg:
    t: float
    roll: float              # rad or normalized (define once)
    pitch: float             # rad or normalized
    yaw_rate: float          # rad/s or normalized
    throttle: float          # 0..1