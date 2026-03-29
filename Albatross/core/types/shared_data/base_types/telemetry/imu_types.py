from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class IMUSample:
    """
    Normalized HIGHRES_IMU-style sample.

    t_ns:
        Local runtime timestamp from time.perf_counter_ns() when this sample
        was accepted into the system.

    accel_mps2:
        Linear acceleration in body frame, shape (3,).

    gyro_rps:
        Angular velocity in body frame, shape (3,).
    """
    t_ns: int
    accel_mps2: np.ndarray   # shape (3,)
    gyro_rps: np.ndarray     # shape (3,)