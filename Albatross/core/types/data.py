from dataclasses import dataclass
from typing import Optional
import numpy as np

@dataclass
class Telemetry:
    t: float               # seconds (sim time if provided)
    pos: Optional[np.ndarray]   # (3,) x,y,z if provided (maybe only start)
    vel: np.ndarray        # (3,)
    quat: np.ndarray       # (4,) w,x,y,z
    ang_vel: Optional[np.ndarray]  # (3,) gyro if provided
    lin_acc: Optional[np.ndarray]  # (3,) accel if provided

@dataclass
class Frame:
    t: float
    image: np.ndarray      # HxWxC uint8
    camera_info: Optional[dict]   # intrinsics if provided

@dataclass
class RawInput:
    frame: Frame
    telemetry: Telemetry

@dataclass
class GateDetection:
    t: float
    gate_id: Optional[int]     # if track/gate numbering exists; else None
    mask: Optional[np.ndarray] # HxW bool/uint8
    corners_px: Optional[np.ndarray]  # (4,2) in image pixels
    center_px: Optional[np.ndarray]   # (2,)
    confidence: float          # 0..1

@dataclass
class GateRelativeState:
    t: float
    # "MonoRace-style": geometry / relative pose surrogates
    bearing: float             # radians (left/right)
    elevation: float           # radians (up/down)
    scale: float               # proxy for distance (e.g., gate size in px)
    yaw_error: float           # gate orientation error in image
    d_bearing: float
    d_elevation: float
    d_scale: float
    confidence: float          # smoothed conf

@dataclass
class Observation:
    t: float
    vec: np.ndarray            # fixed size float32
    mask: np.ndarray           # same length, 0/1 validity

@dataclass
class Action:
    t: float
    throttle: float
    roll: float
    pitch: float
    yaw: float
    confidence: float          # policy confidence or NA

@dataclass
class SafeAction(Action):
    reason: Optional[str] = None