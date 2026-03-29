from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass(frozen=True)
class CameraInfo:
    """
    Camera intrinsics / metadata.

    Keep this generic until the separate vision-stream specification arrives.
    """
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist_coeffs: Optional[np.ndarray] = None   # shape depends on camera model
    frame_id: Optional[str] = None


@dataclass(frozen=True)
class CameraFrame:
    """
    One forward-facing camera frame.
    """
    t_capture_ns: int
    frame_id: int
    image_bgr: np.ndarray   # HxWxC uint8
    camera_name: Optional[str] = None
    valid: bool = True