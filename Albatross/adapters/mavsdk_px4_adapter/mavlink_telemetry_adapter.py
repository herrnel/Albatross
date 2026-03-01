# adapters/mavlink_telemetry_adapter.py
from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
from pymavlink import mavutil

from core.types.data import Telemetry


@dataclass
class TelemetryEmulation:
    start_pos_only: bool = True
    start_pos_duration_s: float = 1.0


class MavlinkTelemetryAdapter:
    def __init__(
        self,
        mav_connection,
        emu: Optional[TelemetryEmulation] = None,
    ):
        self._mav = mav_connection
        self.emu = emu or TelemetryEmulation()

        self._mav: Optional[mavutil.mavfile] = None
        self._t0: Optional[float] = None

        self._lock = threading.Lock()
        self._stop = False
        self._rx_thread: Optional[threading.Thread] = None

        self._last_local_pos_ned: Optional[dict] = None
        self._last_att_quat: Optional[dict] = None

    def connect(self) -> None:
        self._t0 = time.time()

        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop = True
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)

    def read(self) -> Telemetry:
        assert self._t0 is not None, "Call connect() first."

        # Wait briefly for first data
        deadline = time.time() + 2.0
        while time.time() < deadline:
            with self._lock:
                ok = (self._last_local_pos_ned is not None) and (self._last_att_quat is not None)
            if ok:
                break
            time.sleep(0.005)

        now = time.time()
        t = now - self._t0

        with self._lock:
            lp = self._last_local_pos_ned.copy() if self._last_local_pos_ned else None
            aq = self._last_att_quat.copy() if self._last_att_quat else None

        if lp is None:
            pos = None
            vel = np.zeros(3, dtype=np.float32)
        else:
            pos = np.array([lp["x"], lp["y"], lp["z"]], dtype=np.float32)
            vel = np.array([lp["vx"], lp["vy"], lp["vz"]], dtype=np.float32)

        if aq is None:
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        else:
            quat = np.array([aq["q1"], aq["q2"], aq["q3"], aq["q4"]], dtype=np.float32)

        if self.emu.start_pos_only and t > self.emu.start_pos_duration_s:
            pos = None

        return Telemetry(t=t, pos=pos, vel=vel, quat=quat)

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
                        "q1": float(msg.q1),  # w
                        "q2": float(msg.q2),
                        "q3": float(msg.q3),
                        "q4": float(msg.q4),
                    }