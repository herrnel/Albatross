# adapters/mavlink_px4_adapter.py
from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
from pymavlink import mavutil

from core.types.data import Telemetry
from adapters.adapter_base.base_adapter import BaseAdapter


# We want to intentionally throw away extra info we get from SITL 
# So that we can emulate the AGP protocol as best as we can during 
# training and testing in other simulators. AGPEmulationConfig 
# defines this. 
@dataclass
class AGPEmulationConfig:
    # If True: only provide starting position, then pos=None afterwards.
    start_pos_only: bool = True
    # Seconds after connect during which pos is still returned
    start_pos_duration_s: float = 1.0

    # If True: hide IMU-derived fields (ang_vel/lin_acc)
    hide_imu: bool = True

    # If True: clamp to only vel+quat even if other messages exist
    minimal_fields: bool = True


class MavlinkPX4Adapter(BaseAdapter):
    """
    Reads PX4 SITL telemetry over MAVLink (UDP) and returns a competition-like Telemetry object.
    Typical PX4 SITL MAVLink endpoint is usually udp:127.0.0.1:14540 (varies by setup).
    """

    def __init__(
        self,
        connection: str = "udp:127.0.0.1:14540",
        emu: Optional[AGPEmulationConfig] = None,
        heartbeat_timeout_s: float = 10.0,
    ):
        self.connection = connection
        self.emu = emu or AGPEmulationConfig()

        self.heartbeat_timeout_s = heartbeat_timeout_s

        self._mav: Optional[mavutil.mavfile] = None
        self._lock = threading.Lock()

        # Latest cached values
        self._t0: Optional[float] = None
        self._last_local_pos_ned: Optional[dict] = None
        self._last_att_quat: Optional[dict] = None
        self._last_imu: Optional[dict] = None

        self._rx_thread: Optional[threading.Thread] = None
        self._stop = False

    def connect(self) -> None:
        self._mav = mavutil.mavlink_connection(self.connection)
        # Wait for a heartbeat so we know system/component IDs
        self._mav.wait_heartbeat(timeout=self.heartbeat_timeout_s)

        self._t0 = time.time()
        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Optionally: request higher stream rates (PX4 supports this in SITL)
        # Not strictly required; depends on your SITL config.
        try:
            # Rate in Hz
            self._set_stream_rate(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 50)
            self._set_stream_rate(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50)  # attitude
            self._set_stream_rate(mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 50)  # vfr_hud, etc
        except Exception:
            # Safe to ignore; stream rates vary by stack/setup.
            pass

    def close(self) -> None:
        self._stop = True
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)

    def read_telemetry(self) -> Telemetry:
        """
        Returns best-effort Telemetry from latest cached MAVLink messages.
        If some required fields aren't available yet, it will block briefly.
        """
        assert self._t0 is not None, "Call connect() first"

        # Wait until we have at least local pos and quaternion
        deadline = time.time() + 2.0
        while time.time() < deadline:
            with self._lock:
                has_pos = self._last_local_pos_ned is not None
                has_quat = self._last_att_quat is not None
            if has_pos and has_quat:
                break
            time.sleep(0.005)

        now = time.time()
        t = now - self._t0

        with self._lock:
            lp = self._last_local_pos_ned.copy() if self._last_local_pos_ned else None
            aq = self._last_att_quat.copy() if self._last_att_quat else None
            imu = self._last_imu.copy() if self._last_imu else None

        # Default fallback values if not ready
        if lp is None:
            pos = None
            vel = np.zeros(3, dtype=np.float32)
        else:
            # LOCAL_POSITION_NED is NED frame
            pos_ned = np.array([lp["x"], lp["y"], lp["z"]], dtype=np.float32)
            vel_ned = np.array([lp["vx"], lp["vy"], lp["vz"]], dtype=np.float32)

            pos = pos_ned
            vel = vel_ned

        if aq is None:
            quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        else:
            # ATTITUDE_QUATERNION uses q1..q4 where q1 is w
            quat_wxyz = np.array([aq["q1"], aq["q2"], aq["q3"], aq["q4"]], dtype=np.float32)

        # --- Apply AGP-style emulation constraints ---
        if self.emu.start_pos_only:
            if t > self.emu.start_pos_duration_s:
                pos = None

        ang_vel = None
        lin_acc = None
        if not self.emu.hide_imu and imu is not None:
            # HIGHRES_IMU: xacc,yacc,zacc (m/s^2), xgyro,ygyro,zgyro (rad/s)
            lin_acc = np.array([imu["xacc"], imu["yacc"], imu["zacc"]], dtype=np.float32)
            ang_vel = np.array([imu["xgyro"], imu["ygyro"], imu["zgyro"]], dtype=np.float32)

        if self.emu.minimal_fields:
            return Telemetry(t=t, pos=pos, vel=vel, quat=quat_wxyz)

        return Telemetry(t=t, pos=pos, vel=vel, quat=quat_wxyz, ang_vel=ang_vel, lin_acc=lin_acc)

    # ---------------- internals ----------------

    def _rx_loop(self) -> None:
        assert self._mav is not None
        while not self._stop:
            msg = self._mav.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue

            mtype = msg.get_type()
            if mtype == "LOCAL_POSITION_NED":
                with self._lock:
                    self._last_local_pos_ned = {
                        "x": float(msg.x),
                        "y": float(msg.y),
                        "z": float(msg.z),
                        "vx": float(msg.vx),
                        "vy": float(msg.vy),
                        "vz": float(msg.vz),
                    }

            elif mtype == "ATTITUDE_QUATERNION":
                with self._lock:
                    self._last_att_quat = {
                        "q1": float(msg.q1),
                        "q2": float(msg.q2),
                        "q3": float(msg.q3),
                        "q4": float(msg.q4),
                    }

            elif mtype == "HIGHRES_IMU":
                with self._lock:
                    self._last_imu = {
                        "xacc": float(msg.xacc),
                        "yacc": float(msg.yacc),
                        "zacc": float(msg.zacc),
                        "xgyro": float(msg.xgyro),
                        "ygyro": float(msg.ygyro),
                        "zgyro": float(msg.zgyro),
                    }

    def _set_stream_rate(self, stream_id: int, rate_hz: int) -> None:
        assert self._mav is not None
        # MAV_DATA_STREAM_* is legacy but still works in SITL often.
        self._mav.mav.request_data_stream_send(
            self._mav.target_system,
            self._mav.target_component,
            stream_id,
            rate_hz,
            1,
        )