from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil
from Albatross.core.types.archive.data import Action


@dataclass
class ActionSenderConfig:
    max_vx: float = 6.0
    max_vy: float = 6.0
    max_vz: float = 3.0
    max_yaw_rate: float = 2.0


class MavlinkActionSenderAdapter:
    """
    Velocity+yaw_rate offboard sender. Call send() at fixed Hz forever.
    """

    def __init__(self, mav_connection: mavutil.mavfile, cfg: Optional[ActionSenderConfig] = None):
        self.cfg = cfg or ActionSenderConfig()
        self._mav: mavutil.mavfile = mav_connection
        self._t0 = time.time()

    def close(self) -> None:
        pass

    def arm(self) -> None:
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    def set_offboard_mode(self) -> None:
        self._mav.set_mode("OFFBOARD")

    def warmup_stream(self, warmup_s: float = 1.0, hz: float = 50.0) -> None:
        dt = 1.0 / hz
        end = time.time() + warmup_s
        neutral = Action(t=0.0, throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        while time.time() < end:
            self.send(neutral)
            time.sleep(dt)

    def begin_offboard_and_arm(self, warmup_s: float = 1.0, hz: float = 50.0) -> None:
        self.warmup_stream(warmup_s, hz)
        self.set_offboard_mode()
        time.sleep(0.1)
        self.arm()

    def send(self, action: Action) -> None:
        vx = float(action.throttle) * self.cfg.max_vx
        vy = float(action.roll) * self.cfg.max_vy
        vz = float(action.pitch) * self.cfg.max_vz
        yaw_rate = float(action.yaw) * self.cfg.max_yaw_rate

        vx = max(-self.cfg.max_vx, min(self.cfg.max_vx, vx))
        vy = max(-self.cfg.max_vy, min(self.cfg.max_vy, vy))
        vz = max(-self.cfg.max_vz, min(self.cfg.max_vz, vz))
        yaw_rate = max(-self.cfg.max_yaw_rate, min(self.cfg.max_yaw_rate, yaw_rate))

        time_boot_ms = int((time.time() - self._t0) * 1000) & 0xFFFFFFFF

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self._mav.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            0.0,
            yaw_rate
        )