from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from core.modules.module_base.module_type import ModuleBase
from core.types.command.action_type import Action

from libraries.policy.heuristic_policy import HeuristicPolicy, HeuristicPolicyConfig
# from libraries.policy.rl_policy import RLPolicy


@dataclass
class ControlModuleConfig:
    # Start conservative; this aligns better with the spec's external command guidance.
    hz: float = 120.0
    mode: str = "heuristic"             # "heuristic" | "rl"

    # Allow observation to be a bit older than the control loop period.
    max_obs_age_ms: float = 120.0

    # What to do when observation is stale/invalid/missing
    publish_neutral_on_stale: bool = True
    hold_throttle_on_stale: float = 0.55

    # Optional policy config
    heuristic: HeuristicPolicyConfig = HeuristicPolicyConfig()


class ControlModule(ModuleBase):
    """
    Runtime wrapper for control-policy inference.

    Reads:
        shared_state.estimation.observation

    Writes:
        shared_state.control.policy_action
        shared_state.health.module_health

    Does not:
        - send MAVLink
        - stream commands
        - apply safety clamping
    """

    def __init__(self, shared_state, config: Optional[ControlModuleConfig] = None):
        config = config or ControlModuleConfig()
        super().__init__(shared_state=shared_state, hz=config.hz, name="control")

        self.config = config

        if config.mode == "heuristic":
            self.controller = HeuristicPolicy(config.heuristic)
        elif config.mode == "rl":
            raise NotImplementedError("RL policy backend is not implemented yet.")
        else:
            raise ValueError(f"Unsupported control mode: {config.mode}")

        self._last_obs_seq: Optional[int] = None
        self._last_tick_ns: Optional[int] = None

    def setup(self) -> None:
        self._publish_health(
            status="init",
            info=f"mode={self.config.mode}, hz={self.hz}",
            now_ns=time.perf_counter_ns(),
        )

    def tick(self, now_ns: int, local_tick: int) -> None:
        tick_start_ns = now_ns
        self._last_tick_ns = tick_start_ns

        obs, obs_t_ns, obs_seq = self.shared_state.estimation.observation.get()

        if obs is None:
            self._handle_missing_observation(now_ns=tick_start_ns, reason="no_observation")
            return

        obs_age_ms = self.shared_state.estimation.observation.age_ms(now_ns=tick_start_ns)

        if not getattr(obs, "valid", True):
            self._handle_missing_observation(
                now_ns=tick_start_ns,
                reason=f"invalid_observation age_ms={obs_age_ms}",
            )
            return

        if obs_age_ms is None or obs_age_ms > self.config.max_obs_age_ms:
            self._handle_missing_observation(
                now_ns=tick_start_ns,
                reason=f"stale_observation age_ms={obs_age_ms}",
            )
            return

        # Compute outside of any topic lock.
        action_values = self.controller.act(obs.vec, obs.mask)

        action = Action(
            seq=int(local_tick),
            t_ns=tick_start_ns,
            throttle=float(action_values[0]),
            roll=float(action_values[1]),
            pitch=float(action_values[2]),
            yaw=float(action_values[3]),
            confidence=float(action_values[4]) if len(action_values) > 4 else 1.0,
            source=self.config.mode,
            observation_seq_ref=obs_seq,
        )

        self.shared_state.control.policy_action.publish(action, t_ns=tick_start_ns)

        compute_ms = (time.perf_counter_ns() - tick_start_ns) / 1_000_000.0
        self._last_obs_seq = obs_seq

        self._publish_health(
            status="ok",
            info=(
                f"tick={local_tick}, "
                f"obs_seq={obs_seq}, "
                f"obs_age_ms={obs_age_ms:.1f}, "
                f"compute_ms={compute_ms:.3f}"
            ),
            now_ns=tick_start_ns,
        )

    def _handle_missing_observation(self, now_ns: int, reason: str) -> None:
        if self.config.publish_neutral_on_stale:
            neutral = self._neutral_action(now_ns=now_ns, reason=reason)
            self.shared_state.control.policy_action.publish(neutral, t_ns=now_ns)

        self._publish_health(
            status="stale",
            info=reason,
            now_ns=now_ns,
        )

    def _neutral_action(self, now_ns: int, reason: str) -> Action:
        """
        Conservative policy-layer fallback.
        SafetyModule still gets final say.
        """
        return Action(
            seq=-1,
            t_ns=now_ns,
            throttle=self.config.hold_throttle_on_stale,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            confidence=0.0,
            source=f"{self.config.mode}:{reason}",
            observation_seq_ref=None,
        )

