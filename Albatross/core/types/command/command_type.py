from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    t: float = 0.0
    roll: float = 0.0          # rad or normalized (define once)
    pitch: float = 0.0         # rad or normalized
    yaw_rate: float = 0.0      # rad/s or normalized
    yaw_angle: float = 0.0      # rad/s or normalized (This will probably not be used but its here for now)
    thrust: float = 0.0      # 0..1