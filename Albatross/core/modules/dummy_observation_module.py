from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import time
import numpy as np

from core.modules.module_base.module_type import ModuleBase
from core.types.shared_data.base_types.estimation.observation import (
    Observation,
    OBS_DIM,
    OBS_BEARING,
    OBS_ELEVATION,
    OBS_SCALE,
    OBS_YAW_ERROR,
    OBS_D_BEARING,
    OBS_D_ELEVATION,
    OBS_D_SCALE,
    OBS_CONFIDENCE,
)



@dataclass
class DummyObservationModuleConfig:
    hz: float = 30.0
    max_att_age_ms: float = 200.0
    constant_scale: float = 0.20
    confidence: float = 1.0


class DummyObservationModule(ModuleBase):
    """
    Smoke-test observation producer.

    Reads:
        shared_state.sensors.attitude

    Writes:
        shared_state.estimation.observation

    Behavior:
    - Uses current attitude to synthesize a fake gate-relative observation.
    - This is only for framework testing, not real navigation.
    """

    def __init__(self, shared_state, config: Optional[DummyObservationModuleConfig] = None):
        config = config or DummyObservationModuleConfig()
        super().__init__(shared_state=shared_state, hz=config.hz, name="dummy_observation")
        self.config = config
        self._last_roll = None
        self._last_pitch = None
        self._last_yaw = None
        self._last_t_ns = None

    def setup(self) -> None:
        self._publish_health("init", "dummy observation module ready", time.perf_counter_ns())

    def tick(self, now_ns: int, local_tick: int) -> None:
        att, att_t_ns, att_seq = self.shared_state.sensors.attitude.get()

        if att is None:
            self._publish_invalid_observation(now_ns, reason="no_attitude")
            return

        att_age_ms = self.shared_state.sensors.attitude.age_ms(now_ns=now_ns)
        if att_age_ms is None or att_age_ms > self.config.max_att_age_ms:
            self._publish_invalid_observation(now_ns, reason=f"stale_attitude age_ms={att_age_ms}")
            return

        vec = np.zeros((OBS_DIM,), dtype=np.float32)
        mask = np.zeros((OBS_DIM,), dtype=np.float32)

        # Dumb synthetic observation:
        # - treat current yaw as if it were gate bearing/yaw error
        # - treat current pitch as if it were vertical gate error
        # - keep scale constant so the heuristic will pitch forward
        bearing = float(att.yaw_rad)
        elevation = float(att.pitch_rad)
        scale = float(self.config.constant_scale)
        yaw_error = float(att.yaw_rad)

        d_bearing = 0.0
        d_elevation = 0.0
        d_scale = 0.0

        if self._last_t_ns is not None:
            dt = max((now_ns - self._last_t_ns) / 1e9, 1e-6)
            if self._last_yaw is not None:
                d_bearing = (att.yaw_rad - self._last_yaw) / dt
            if self._last_pitch is not None:
                d_elevation = (att.pitch_rad - self._last_pitch) / dt

        vec[OBS_BEARING] = bearing
        vec[OBS_ELEVATION] = elevation
        vec[OBS_SCALE] = scale
        vec[OBS_YAW_ERROR] = yaw_error
        vec[OBS_D_BEARING] = d_bearing
        vec[OBS_D_ELEVATION] = d_elevation
        vec[OBS_D_SCALE] = d_scale
        vec[OBS_CONFIDENCE] = self.config.confidence

        mask[:] = 1.0

        obs = Observation(
            t_ref_ns=now_ns,
            vec=vec,
            mask=mask,
            valid=True,
            source="dummy_observation_module",
            info=f"att_seq={att_seq}",
        )

        self.shared_state.estimation.observation.publish(obs, t_ns=now_ns)

        self._last_roll = att.roll_rad
        self._last_pitch = att.pitch_rad
        self._last_yaw = att.yaw_rad
        self._last_t_ns = now_ns

        self._publish_health(
            "ok",
            f"tick={local_tick}, att_age_ms={att_age_ms:.1f}, att_seq={att_seq}",
            now_ns,
        )

    def _publish_invalid_observation(self, now_ns: int, reason: str) -> None:
        vec = np.zeros((OBS_DIM,), dtype=np.float32)
        mask = np.zeros((OBS_DIM,), dtype=np.float32)

        obs = Observation(
            t_ref_ns=now_ns,
            vec=vec,
            mask=mask,
            valid=False,
            source="dummy_observation_module",
            info=reason,
        )
        self.shared_state.estimation.observation.publish(obs, t_ns=now_ns)
        self._publish_health("stale", reason, now_ns)
