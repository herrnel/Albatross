from dataclasses import dataclass
import numpy as np
from typing import Optional

@dataclass(frozen=True)
class ImuSample:
    t: float                 # seconds (monotonic)
    accel: np.ndarray        # (3,) m/s^2
    gyro: np.ndarray         # (3,) rad/s
    
class AttitudeData:
    t: float               # seconds (sim time if provided)
    vel: np.ndarray        # (3,)
    quat: np.ndarray       # (4,) w,x,y,z (This is used instead of roll, pitch, and yaw degrees)
    # rollspeed: float     # Don't know if we will have this yet. 
    # pitchspeed: float
    # yawspeed: float
    imu: Optional[ImuSample]  # Contains IMU data. 

@dataclass(frozen=True)
class FrameMsg:
    t: float                 # capture time
    frame_id: int
    image: np.ndarray        # HxWxC uint8

@dataclass
class LocalPositionNED:
    t: float
    x: float
    y: float
    z: float
