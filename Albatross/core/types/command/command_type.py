from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    t: float = 0.0
    roll: float = 0.0          # rad or normalized (define once)
    pitch: float = 0.0         # rad or normalized
    yaw_rate: float = 0.0      # rad/s or normalized
    throttle: float = 0.0      # 0..1