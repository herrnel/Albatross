from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from core.types.shared_data.base_types.estimation.observation import (
    OBS_BEARING,
    OBS_ELEVATION,
    OBS_SCALE,
    OBS_YAW_ERROR,
    OBS_CONFIDENCE,
)


@dataclass(frozen=True)
class HeuristicPolicyConfig:
    # Gains
    k_bearing_roll: float = 0.7
    k_yaw: float = 0.5
    k_elevation_throttle: float = 0.25
    k_pitch_forward: float = 0.45

    # Limits
    max_roll: float = 0.35
    max_pitch: float = 0.45
    max_yaw: float = 0.40

    # Forward-speed scheduling
    far_scale_threshold: float = 0.18
    near_scale_threshold: float = 0.55

    # Vertical command baseline
    base_throttle: float = 0.55

    # Confidence fallback
    min_conf_for_aggressive: float = 0.35


class HeuristicPolicy:
    """
    Simple gate-centering policy over a compact observation vector. Our heuristic policy is basically a hand-written “fly toward the gate” controller.
    The whole policy is:
        1. line up horizontally
        2. line up rotationally
        3. trim height
        4. move forward when alignment is decent


    Expected observation layout:
        idx 0: bearing
        idx 1: elevation
        idx 2: scale
        idx 3: yaw_error
        idx 4: d_bearing
        idx 5: d_elevation
        idx 6: d_scale
        idx 7: confidence

    obs.mask should be 1 when the corresponding feature is valid.
    """

    def __init__(self, config: HeuristicPolicyConfig | None = None):
        self.cfg = config or HeuristicPolicyConfig()
    
    def act(self, obs_vec: np.ndarray, obs_mask: np.ndarray) -> np.ndarray:
        """_summary_

        Args:
            obs_vec (np.ndarray): _description_
            obs_mask (np.ndarray): _description_

        Returns:
            np.ndarray: _description_
        """
        bearing = self._read(obs_vec, obs_mask, OBS_BEARING, default=0.0)
        elevation = self._read(obs_vec, obs_mask, OBS_ELEVATION, default=0.0)
        scale = self._read(obs_vec, obs_mask, OBS_SCALE, default=0.0)
        yaw_error = self._read(obs_vec, obs_mask, OBS_YAW_ERROR, default=0.0)
        conf = self._read(obs_vec, obs_mask, OBS_CONFIDENCE, default=0.0)

        # If confidence is poor, stay conservative.
        aggressive = conf >= self.cfg.min_conf_for_aggressive

        # Roll to reduce horizontal image error.
        roll = -self.cfg.k_bearing_roll * bearing # Here we adjust by some fixed about which in this case is 0.7 to get back on to the right path. 
        roll = float(np.clip(roll, -self.cfg.max_roll, self.cfg.max_roll))

        # Yaw to reduce gate orientation error.
        yaw = -self.cfg.k_yaw * yaw_error
        yaw = float(np.clip(yaw, -self.cfg.max_yaw, self.cfg.max_yaw))

        # Forward pitch depends on:
        # - confidence
        # - gate centeredness
        # - apparent distance proxy from scale
        centeredness = max(0.0, 1.0 - abs(bearing))
        forward_factor = self._forward_factor_from_scale(scale)

        if aggressive:
            pitch = self.cfg.k_pitch_forward * centeredness * forward_factor
        else:
            pitch = 0.15 * centeredness

        pitch = float(np.clip(pitch, -self.cfg.max_pitch, self.cfg.max_pitch))

        # Throttle trims vertical gate error. This is deliberately mild.
        throttle = self.cfg.base_throttle - self.cfg.k_elevation_throttle * elevation
        throttle = float(np.clip(throttle, 0.35, 0.75))

        return np.array([throttle, roll, pitch, yaw, conf], dtype=np.float32)

    def _forward_factor_from_scale(self, scale: float) -> float:
        """
        Smaller apparent gate scale -> farther away -> more permission to pitch forward.
        Larger scale -> close gate -> back off.
        """
        far_t = self.cfg.far_scale_threshold
        near_t = self.cfg.near_scale_threshold

        if scale <= far_t:
            return 1.0
        if scale >= near_t:
            return 0.2

        # Linear interpolation between far and near thresholds
        alpha = (scale - far_t) / (near_t - far_t)
        return float(1.0 - 0.8 * alpha)

    @staticmethod
    def _read(obs_vec: np.ndarray, obs_mask: np.ndarray, idx: int, default: float) -> float:
        if idx >= len(obs_vec):
            return default
        if obs_mask is not None and idx < len(obs_mask) and obs_mask[idx] <= 0:
            return default
        return float(obs_vec[idx])