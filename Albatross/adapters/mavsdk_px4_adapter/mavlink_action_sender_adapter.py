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


class MavlinkActionSenderAdapter:
    def __init__(self, mav_connection, cfg: Optional[ActionSenderConfig] = None):
        self.cfg = cfg or ActionSenderConfig()
        self._mav: Optional[mavutil.mavfile] = mav_connection
        self._t0 = time.time()   # <-- add this

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
        
    def begin_offboard_and_arm(self, warmup_s: float = 0.7, hz: float = 30.0):
        dt = 1.0 / hz
        t_end = time.time() + warmup_s
        while time.time() < t_end:
            neutral = Action(t=0.0, throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0)
            self.send(neutral)
            time.sleep(dt)
        self.set_offboard_mode()
        time.sleep(0.1)
        self.arm()

    # ---------- Sending ----------
    def send(self, action: Action) -> None:
        """
        Interpret Action roll/pitch as desired XY velocity direction (normalized),
        throttle as forward speed scaling (0..1), yaw as yaw-rate command.
        This gives you an easy “racer control” interface now.
        """
        # Map controls -> NED velocity setpoint
        # Convention: +vx forward (North), +vy right (East), +vz down.
        vx = float(action.throttle) * self.cfg.max_vx
        vy = float(action.roll) * self.cfg.max_vy
        vz = float(action.pitch) * self.cfg.max_vz   # careful: pitch maps to vertical for now (simple)
        yaw_rate = float(action.yaw) * self.cfg.max_yaw_rate

        # Clamp
        vx = max(-self.cfg.max_vx, min(self.cfg.max_vx, vx))
        vy = max(-self.cfg.max_vy, min(self.cfg.max_vy, vy))
        vz = max(-self.cfg.max_vz, min(self.cfg.max_vz, vz))

        time_boot_ms = int((time.time() - self._t0) * 1000) & 0xFFFFFFFF

        # Type mask: ignore position, accel, yaw angle; use vx,vy,vz and yaw_rate
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            # DO NOT ignore vx/vy/vz or yaw_rate
        )

        self._mav.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,   # x,y,z ignored
            vx, vy, vz,      # velocity setpoints USED
            0.0, 0.0, 0.0,   # accel ignored
            0.0,             # yaw ignored
            yaw_rate         # yaw rate USED
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