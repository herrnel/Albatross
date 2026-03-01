# adapters/mavlink_action_sender.py
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from core.types.data import Action


@dataclass
class ActionSenderConfig:
    # How to scale normalized roll/pitch/yaw into actual commands
    max_roll_rad: float = 0.6     # ~34 deg
    max_pitch_rad: float = 0.6
    max_yaw_rate_rads: float = 1.5


class MavlinkActionAdapter:
    def __init__(self, mav_connection, cfg: Optional[ActionSenderConfig] = None):
        self.cfg = cfg or ActionSenderConfig()
        self._mav: Optional[mavutil.mavfile] = mav_connection

    def close(self) -> None:
        pass

    # ---------- Optional helpers (best-effort) ----------
    def arm(self) -> None:
        assert self._mav is not None
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    def set_offboard_mode(self) -> None:
        """
        Best-effort: PX4 generally needs:
        - continuous setpoints streaming
        - then mode switch to OFFBOARD
        This helper just tries to set mode; you still must stream.
        """
        assert self._mav is not None
        # Try common PX4 custom mode approach:
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED + custom_mode for OFFBOARD
        # This can vary; treat as best-effort.
        self._mav.set_mode("OFFBOARD")

    # ---------- Sending ----------
    def send(self, action: Action) -> None:
        """
        Sends an attitude/thrust setpoint derived from normalized (throttle, roll, pitch, yaw).

        - roll/pitch: interpreted as desired angles (rad) scaled by max_*
        - yaw: interpreted as desired yaw rate (rad/s) scaled by max_yaw_rate_rads
        - throttle: interpreted as thrust 0..1
        """
        assert self._mav is not None

        # Scale normalized to physical-ish commands
        roll = float(action.roll) * self.cfg.max_roll_rad
        pitch = float(action.pitch) * self.cfg.max_pitch_rad
        yaw_rate = float(action.yaw) * self.cfg.max_yaw_rate_rads
        thrust = float(action.throttle)

        # Build a quaternion for roll/pitch only (ignore yaw angle, use yaw rate)
        # Simple small-angle composition is okay for baseline; for higher fidelity use proper quaternion math.
        # We'll do a proper conversion:
        q = _quat_from_roll_pitch(roll, pitch)

        # Type mask:
        # bit 0: ignore body roll rate
        # bit 1: ignore body pitch rate
        # bit 2: ignore body yaw rate  (we will NOT ignore yaw rate -> clear this bit)
        # bit 3: ignore attitude       (we will NOT ignore attitude -> clear this bit)
        # bit 4: ignore thrust         (we will NOT ignore thrust -> clear this bit)
        # We want: use attitude + yaw rate + thrust, ignore roll/pitch body rates.
        type_mask = 0b00000011  # ignore body roll/pitch rates

        self._mav.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            self._mav.target_system,
            self._mav.target_component,
            type_mask,
            q,            # q1,q2,q3,q4 (w,x,y,z)
            0.0,          # body_roll_rate ignored
            0.0,          # body_pitch_rate ignored
            yaw_rate,     # body_yaw_rate used
            thrust
        )


def _quat_from_roll_pitch(roll: float, pitch: float):
    """
    Quaternion from roll/pitch (yaw=0).
    Returns (w,x,y,z) as a list of floats.
    """
    import math
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    # yaw = 0 => cy=1, sy=0
    # q = q_yaw * q_pitch * q_roll (yaw=0 simplifies)
    w = cp * cr
    x = cp * sr
    y = sp * cr
    z = -sp * sr
    return [w, x, y, z]