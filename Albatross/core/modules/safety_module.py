from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from core.modules.module_base.module_type import ModuleBase
from core.types.command.action_type import Action
from core.types.command.safe_action_type import SafeAction
from core.types.command.command_type import Command


@dataclass
class SafetyModuleConfig:
    hz: float = 120.0

    # Freshness / validity
    max_action_age_ms: float = 150.0
    min_confidence: float = 0.10

    # Simple clamp limits
    min_throttle: float = 0.35
    max_throttle: float = 0.75
    max_roll: float = 0.35
    max_pitch: float = 0.45
    max_yaw: float = 0.40

    # Fallback / hold behavior
    hold_throttle: float = 0.55
    publish_fallback_on_missing: bool = True


class SafetyModule(ModuleBase):
    """
    Simple safety wrapper.

    Reads:
        shared_state.control.policy_action

    Writes:
        shared_state.control.safe_action
        shared_state.control.pilot_command
        shared_state.control.command

    Behavior:
    - if action is missing/stale/low-confidence -> publish safe fallback
    - otherwise clamp fields and forward them
    """

    def __init__(self, shared_state, config: Optional[SafetyModuleConfig] = None):
        config = config or SafetyModuleConfig()
        super().__init__(shared_state=shared_state, hz=config.hz, name="safety")
        self.config = config

    def setup(self) -> None:
        self._publish_health(
            status="init",
            info=f"hz={self.hz}",
            now_ns=time.perf_counter_ns(),
        )

    def tick(self, now_ns: int, local_tick: int) -> None:
        action, action_t_ns, action_seq = self.shared_state.control.policy_action.get()

        if action is None:
            self._handle_missing_action(now_ns, reason="no_policy_action")
            return

        action_age_ms = self.shared_state.control.policy_action.age_ms(now_ns=now_ns)
        if action_age_ms is None or action_age_ms > self.config.max_action_age_ms:
            self._handle_missing_action(
                now_ns,
                reason=f"stale_policy_action age_ms={action_age_ms}",
            )
            return

        if action.confidence < self.config.min_confidence:
            self._handle_missing_action(
                now_ns,
                reason=f"low_confidence conf={action.confidence:.3f}",
            )
            return

        safe_action = self._clamp_action(action=action, now_ns=now_ns)
        cmd = self._to_command(safe_action=safe_action, now_ns=now_ns)

        self.shared_state.control.safe_action.publish(safe_action, t_ns=now_ns)
        self.shared_state.control.pilot_command.publish(cmd, t_ns=now_ns)
        self.shared_state.control.command.publish(cmd, t_ns=now_ns)
        self.shared_state.control.command_tx_history.publish(cmd, t_ns=now_ns) # I have a suspicion this should be moved to the thread where the actual command will be sent out since its possible that this command will be overwritten before it gets sent out. 

        self._publish_health(
            status="ok",
            info=(
                f"tick={local_tick}, "
                f"action_seq={action_seq}, "
                f"action_age_ms={action_age_ms:.1f}, "
                f"conf={action.confidence:.2f}"
            ),
            now_ns=now_ns,
        )

    def _handle_missing_action(self, now_ns: int, reason: str) -> None:
        if self.config.publish_fallback_on_missing:
            safe_action = SafeAction(
                seq=-1,
                t_ns=now_ns,
                throttle=self.config.hold_throttle,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                confidence=0.0,
                override=True,
                reason=reason,
                source="safety:fallback",
                action_seq_ref=None,
            )
            cmd = self._to_command(safe_action=safe_action, now_ns=now_ns)

            self.shared_state.control.safe_action.publish(safe_action, t_ns=now_ns)
            self.shared_state.control.pilot_command.publish(cmd, t_ns=now_ns)
            self.shared_state.control.command.publish(cmd, t_ns=now_ns)
            self.shared_state.control.command_tx_history.publish(cmd, t_ns=now_ns) # I have a suspicion this should be moved to the thread where the actual command will be sent out since its possible that this command will be overwritten before it gets sent out. 

        self._publish_health(
            status="stale",
            info=reason,
            now_ns=now_ns,
        )

    def _clamp_action(self, action: Action, now_ns: int) -> SafeAction:
        throttle = float(np.clip(
            action.throttle,
            self.config.min_throttle,
            self.config.max_throttle,
        ))
        roll = float(np.clip(
            action.roll,
            -self.config.max_roll,
            self.config.max_roll,
        ))
        pitch = float(np.clip(
            action.pitch,
            -self.config.max_pitch,
            self.config.max_pitch,
        ))
        yaw = float(np.clip(
            action.yaw,
            -self.config.max_yaw,
            self.config.max_yaw,
        ))

        overridden = (
            throttle != action.throttle
            or roll != action.roll
            or pitch != action.pitch
            or yaw != action.yaw
        )

        return SafeAction(
            seq=action.seq,
            t_ns=now_ns,
            throttle=throttle,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            confidence=action.confidence,
            override=overridden,
            reason="clamped" if overridden else None,
            source="safety",
            action_seq_ref=action.seq,
        )

    def _to_command(self, safe_action: SafeAction, now_ns: int) -> Command:
        """
        Map SafeAction -> Command.

        IMPORTANT:
        Adjust this to match your actual Command dataclass.
        I am using the older style you showed earlier:
            Command(
                roll=...,
                pitch=...,
                yaw_rate=...,
                thrust=...,
                t=...
            )
        """
        return Command(
            roll=safe_action.roll,
            pitch=safe_action.pitch,
            yaw_rate=safe_action.yaw,
            thrust=safe_action.throttle,
            t=now_ns / 1e9,
        )