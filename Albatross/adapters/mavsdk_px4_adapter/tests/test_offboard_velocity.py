#!/usr/bin/env python3
from __future__ import annotations

import time
from pymavlink import mavutil


def is_armed(hb) -> bool:
    return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


def send_vel_local_ned(mav, t0, vx, vy, vz, yaw_rate):
    """
    SET_POSITION_TARGET_LOCAL_NED with velocity setpoints.
    Frame LOCAL_NED: +x North/forward, +y East/right, +z Down
    So: vz < 0 => climb up.
    """
    time_boot_ms = int((time.time() - t0) * 1000) & 0xFFFFFFFF

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    mav.mav.set_position_target_local_ned_send(
        time_boot_ms,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0, 0.0, 0.0,                 # pos ignored
        float(vx), float(vy), float(vz),# vel used
        0.0, 0.0, 0.0,                 # accel ignored
        0.0,                           # yaw ignored
        float(yaw_rate)                # yaw rate used
    )


def arm(mav):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )


def disarm(mav):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )


def request_offboard(mav):
    mav.set_mode("OFFBOARD")


def main():
    endpoint = "udpin:0.0.0.0:14540"
    print("[info] connecting:", endpoint)
    mav = mavutil.mavlink_connection(endpoint)

    hb = mav.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No heartbeat")
    print("[info] heartbeat OK sysid/compid:", mav.target_system, mav.target_component)

    hz = 50.0
    dt = 1.0 / hz
    t0 = time.time()

    def stream_for(seconds: float, vx: float, vy: float, vz: float, yaw_rate: float):
        end = time.time() + seconds
        last_print = 0.0
        while time.time() < end:
            send_vel_local_ned(mav, t0, vx, vy, vz, yaw_rate)
            hb_now = mav.recv_match(type="HEARTBEAT", blocking=False)
            if hb_now and time.time() - last_print > 1.0:
                armed = is_armed(hb_now)
                print(f"[hb] armed={armed} base_mode={hb_now.base_mode} custom_mode={hb_now.custom_mode}")
                last_print = time.time()
            time.sleep(dt)

    try:
        # 1) Warmup stream
        print("[phase] warmup: neutral setpoints")
        stream_for(2.0, 0.0, 0.0, 0.0, 0.0)

        # 2) Arm while streaming
        print("[cmd] ARM")
        arm(mav)
        stream_for(0.5, 0.0, 0.0, 0.0, 0.0)

        # 3) Offboard while streaming
        print("[cmd] OFFBOARD")
        request_offboard(mav)

        # 4) Takeoff/climb: vz negative (NED)
        print("[phase] climb: vz=-0.6 m/s for 2s (up)")
        stream_for(2.0, 0.0, 0.0, -0.6, 0.0)

        # 5) Forward flight
        print("[phase] forward: vx=2.0 m/s for 6s")
        stream_for(6.0, 2.0, 0.0, 0.0, 0.0)

        # 6) Stop (hover hold in velocity terms)
        print("[phase] hold: zero vel for 1s")
        stream_for(1.0, 0.0, 0.0, 0.0, 0.0)

        # 7) Disarm request
        print("[cmd] DISARM request")
        disarm(mav)

        print("[done] If you saw lift-off, offboard + velocity control is working.")
    finally:
        # brief neutral streaming on exit
        try:
            stream_for(0.3, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()