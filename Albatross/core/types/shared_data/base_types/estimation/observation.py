from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np


# ----------------------------
# Observation layout constants
# ----------------------------

OBS_BEARING = 0
OBS_ELEVATION = 1
OBS_SCALE = 2
OBS_YAW_ERROR = 3
OBS_D_BEARING = 4
OBS_D_ELEVATION = 5
OBS_D_SCALE = 6
OBS_CONFIDENCE = 7

OBS_DIM = 8

OBS_FEATURE_NAMES: Tuple[str, ...] = (
    "bearing",
    "elevation",
    "scale",
    "yaw_error",
    "d_bearing",
    "d_elevation",
    "d_scale",
    "confidence",
)


@dataclass(frozen=True)
class Observation:
    """
    Compact control/planning observation.

    Notes:
    - `vec` is the feature vector consumed by policy/control.
    - `mask` is 1.0 where the corresponding vec entry is valid, else 0.0.
    - `t_ref_ns` is the semantic time the observation refers to
      (usually tied to upstream estimator/gate state timing).
    - Topic publish seq/timestamp are handled by LastValueTopic / RingBufferTopic,
      so they are NOT duplicated here.
    """

    t_ref_ns: int

    vec: np.ndarray          # shape (OBS_DIM,), dtype float32 preferred
    mask: np.ndarray         # shape (OBS_DIM,), dtype float32 preferred

    valid: bool = True
    source: str = "observation_builder"

    # Traceability to upstream computations
    gate_relative_seq_ref: Optional[int] = None
    fused_state_seq_ref: Optional[int] = None
    target_gate_seq_ref: Optional[int] = None

    # Optional debug info
    info: Optional[str] = None

    def __post_init__(self) -> None:
        if self.vec.shape != (OBS_DIM,):
            raise ValueError(f"Observation.vec must have shape {(OBS_DIM,)}, got {self.vec.shape}")

        if self.mask.shape != (OBS_DIM,):
            raise ValueError(f"Observation.mask must have shape {(OBS_DIM,)}, got {self.mask.shape}")

        # Normalize dtypes for downstream policy code
        object.__setattr__(self, "vec", self.vec.astype(np.float32, copy=False))
        object.__setattr__(self, "mask", self.mask.astype(np.float32, copy=False))

    # ----------------------------
    # Convenience accessors
    # ----------------------------

    @property
    def bearing(self) -> float:
        return float(self.vec[OBS_BEARING])

    @property
    def elevation(self) -> float:
        return float(self.vec[OBS_ELEVATION])

    @property
    def scale(self) -> float:
        return float(self.vec[OBS_SCALE])

    @property
    def yaw_error(self) -> float:
        return float(self.vec[OBS_YAW_ERROR])

    @property
    def d_bearing(self) -> float:
        return float(self.vec[OBS_D_BEARING])

    @property
    def d_elevation(self) -> float:
        return float(self.vec[OBS_D_ELEVATION])

    @property
    def d_scale(self) -> float:
        return float(self.vec[OBS_D_SCALE])

    @property
    def confidence(self) -> float:
        return float(self.vec[OBS_CONFIDENCE])

    def is_feature_valid(self, idx: int) -> bool:
        return 0 <= idx < OBS_DIM and float(self.mask[idx]) > 0.0

    def feature_dict(self) -> dict[str, float]:
        return {name: float(self.vec[i]) for i, name in enumerate(OBS_FEATURE_NAMES)}

    def mask_dict(self) -> dict[str, float]:
        return {name: float(self.mask[i]) for i, name in enumerate(OBS_FEATURE_NAMES)}
    
    
    


class ObservationBuilder:
    def build(
        self,
        gate_relative,
        gate_relative_seq: int | None = None,
        fused_state_seq: int | None = None,
        target_gate_seq: int | None = None,
    ) -> Observation:
        vec = np.zeros((OBS_DIM,), dtype=np.float32)
        mask = np.zeros((OBS_DIM,), dtype=np.float32)

        if gate_relative is not None and getattr(gate_relative, "valid", False):
            if gate_relative.bearing is not None:
                vec[OBS_BEARING] = float(gate_relative.bearing)
                mask[OBS_BEARING] = 1.0

            if gate_relative.elevation is not None:
                vec[OBS_ELEVATION] = float(gate_relative.elevation)
                mask[OBS_ELEVATION] = 1.0

            if gate_relative.scale is not None:
                vec[OBS_SCALE] = float(gate_relative.scale)
                mask[OBS_SCALE] = 1.0

            if gate_relative.yaw_error is not None:
                vec[OBS_YAW_ERROR] = float(gate_relative.yaw_error)
                mask[OBS_YAW_ERROR] = 1.0

            if gate_relative.d_bearing is not None:
                vec[OBS_D_BEARING] = float(gate_relative.d_bearing)
                mask[OBS_D_BEARING] = 1.0

            if gate_relative.d_elevation is not None:
                vec[OBS_D_ELEVATION] = float(gate_relative.d_elevation)
                mask[OBS_D_ELEVATION] = 1.0

            if gate_relative.d_scale is not None:
                vec[OBS_D_SCALE] = float(gate_relative.d_scale)
                mask[OBS_D_SCALE] = 1.0

            vec[OBS_CONFIDENCE] = float(gate_relative.confidence)
            mask[OBS_CONFIDENCE] = 1.0

            valid = True
            t_ref_ns = int(gate_relative.t_ref_ns)
        else:
            valid = False
            t_ref_ns = time.perf_counter_ns()

        return Observation(
            t_ref_ns=t_ref_ns,
            vec=vec,
            mask=mask,
            valid=valid,
            source="observation_builder",
            gate_relative_seq_ref=gate_relative_seq,
            fused_state_seq_ref=fused_state_seq,
            target_gate_seq_ref=target_gate_seq,
        )