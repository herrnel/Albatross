from __future__ import annotations

from pymavlink import mavutil


def make_mavlink_connection(endpoint: str, heartbeat_timeout_s: float = 10.0) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(endpoint)
    hb = mav.wait_heartbeat(timeout=heartbeat_timeout_s)
    if hb is None:
        raise RuntimeError(f"No heartbeat on {endpoint}. Is PX4 running and bound to that port?")
    print("Heartbeat from", mav.target_system, mav.target_component)
    return mav