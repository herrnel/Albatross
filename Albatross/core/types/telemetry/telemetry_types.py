from dataclasses import dataclass
import numpy as np
from typing import Optional

@dataclass(frozen=True)
class ImuMsg:
    t: float                 # seconds (monotonic)
    accel: np.ndarray        # (3,) m/s^2
    gyro: np.ndarray         # (3,) rad/s
    
class Telemetry:
    t: float               # seconds (sim time if provided)
    vel: np.ndarray        # (3,)
    quat: np.ndarray       # (4,) w,x,y,z
    imu: Optional[ImuMsg]  # Contains IMU data. 

@dataclass(frozen=True)
class FrameMsg:
    t: float                 # capture time
    frame_id: int
    image: np.ndarray        # HxWxC uint8


