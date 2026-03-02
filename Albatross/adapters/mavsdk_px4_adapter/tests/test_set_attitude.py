#!/usr/bin/env python3
from __future__ import annotations

import time
import math
from pymavlink import mavutil


# ----------------------------
# Utilities
# ----------------------------
def quat_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    """
    Euler (rad) -> quaternion (w, x, y, z), aerospace convention (ZYX: yaw->pitch->roll).
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # Normalize to be safe
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n > 0:
        w, x, y, z = w/n, x/n, y/n, z/n
    return [w, x, y, z]


def is_armed_from_heartbeat(hb) -> bool:
    return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


def send_attitude_target(
    mav: mavutil.mavfile,
    t0: float,
    roll: float,
    pitch: float,
    yaw_angle: float,
    yaw_rate: float,
    thrust: float,
):
    """
    Send SET_ATTITUDE_TARGET.

    - roll/pitch/yaw_angle are absolute attitude angles (rad) used to form quaternion.
    - yaw_rate is body yaw rate (rad/s) IF not ignored by type_mask.
    - thrust in [0,1].
    """
    thrust = max(0.0, min(1.0, float(thrust)))
    yaw_rate = float(yaw_rate)

    q = quat_from_euler(roll, pitch, yaw_angle)

    # Use attitude + yaw_rate + thrust. Ignore body roll/pitch rates.
    # bit0 ignore body_roll_rate, bit1 ignore body_pitch_rate
    # keep bit2 clear (use body_yaw_rate), bit3 clear (use attitude), bit4 clear (use thrust)
    type_mask = 0b00000011

    time_boot_ms = int((time.time() - t0) * 1000) & 0xFFFFFFFF

    mav.mav.set_attitude_target_send(
        time_boot_ms,
        mav.target_system,
        mav.target_component,
        type_mask,
        q,
        0.0,        # body_roll_rate ignored
        0.0,        # body_pitch_rate ignored
        yaw_rate,   # used
        thrust      # used
    )


def request_offboard(mav: mavutil.mavfile):
    try:
        mav.set_mode("OFFBOARD")
        print("[cmd] requested OFFBOARD")
    except Exception as e:
        print("[cmd] OFFBOARD request failed:", e)


def request_arm(mav: mavutil.mavfile):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[cmd] requested ARM")


def request_disarm(mav: mavutil.mavfile):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("[cmd] requested DISARM")
    

def wait_local_position(mav, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg:
            return msg
        time.sleep(0.01)
    return None


# ----------------------------
# Main test
# ----------------------------
def main():
    endpoint = "udpin:0.0.0.0:14540"

    print("[info] connecting:", endpoint)
    mav = mavutil.mavlink_connection(endpoint)

    hb = mav.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No heartbeat received. Wrong port or PX4 not running?")
    print("[info] heartbeat OK from sysid/compid:", mav.target_system, mav.target_component)

    # Use a higher rate to avoid any “offboard loss” sensitivity
    hz = 50.0
    dt = 1.0 / hz

    # Thrust tuning: hover varies by model. We use:
    hover_thrust = 0.74
    takeoff_thrust = 0.82  # short bump to break ground contact safely

    # Phase durations
    warmup_s = 2.0
    after_arm_stream_s = 0.5
    takeoff_bump_s = 1.5
    active_s = 3.0
    cooldown_s = 1.0

    # Attitude commands
    pitch_forward = math.radians(10.0)  # +10° pitch
    roll_cmd = 0.0

    # Yaw: keep angle 0 in quaternion, use yaw_rate if desired
    yaw_angle_cmd = 0.0
    yaw_rate_cmd = 0.0

    t0 = time.time()
    
    def thrust_ramp_find_takeoff(duration_s: float, start: float, end: float):
        steps = int(duration_s * hz)
        z0 = None

        for i in range(steps):
            a = i / max(1, steps - 1)
            thrust = start + a * (end - start)

            send_attitude_target(mav, t0, roll=0.0, pitch=0.0, yaw_angle=yaw_angle_cmd, yaw_rate=0.0, thrust=thrust)

            msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg:
                # NED: z is DOWN. So going UP means z becomes MORE NEGATIVE.
                if z0 is None:
                    z0 = float(msg.z)
                else:
                    dz = float(msg.z) - z0
                    # If we've moved up by > 0.15 m (z decreased by ~0.15), call it takeoff.
                    if dz < -0.15:
                        print(f"[takeoff-detected] thrust≈{thrust:.3f}  z0={z0:.2f}  z={float(msg.z):.2f}")
                        return thrust

            time.sleep(dt)

        print("[takeoff-not-detected] try increasing end thrust or ramp duration")
        return None


    def stream_for(seconds: float, roll: float, pitch: float, thrust: float, yaw_rate: float):
        end = time.time() + seconds
        last_hb_print = 0.0
        while time.time() < end:
            send_attitude_target(
                mav, t0,
                roll=roll,
                pitch=pitch,
                yaw_angle=yaw_angle_cmd,
                yaw_rate=yaw_rate,
                thrust=thrust
            )

            # print state about 1 Hz
            if time.time() - last_hb_print > 1.0:
                hb_now = mav.recv_match(type="HEARTBEAT", blocking=False)
                if hb_now:
                    print(f"[hb] armed={is_armed_from_heartbeat(hb_now)} base_mode={hb_now.base_mode} custom_mode={hb_now.custom_mode}")
                last_hb_print = time.time()

            time.sleep(dt)

    try:
        # 1) Warmup: stream neutral setpoints before arming/mode changes
        print("[phase] warmup: neutral streaming")
        stream_for(warmup_s, roll=0.0, pitch=0.0, thrust=hover_thrust, yaw_rate=0.0)

        # 2) Arm FIRST (keep streaming through it)
        print("[cmd] ARM (while streaming)")
        request_arm(mav)
        stream_for(after_arm_stream_s, roll=0.0, pitch=0.0, thrust=hover_thrust, yaw_rate=0.0)

        # 3) Request OFFBOARD (still streaming)
        print("[cmd] OFFBOARD (while streaming)")
        request_offboard(mav)

        # # 4) Takeoff bump (level attitude, slightly higher thrust)
        # print("[phase] takeoff bump: level attitude, higher thrust")
        # stream_for(takeoff_bump_s, roll=0.0, pitch=0.0, thrust=takeoff_thrust, yaw_rate=0.0)
        
        print("[phase] takeoff ramp: level attitude")
        thrust_ramp_find_takeoff(2.0, hover_thrust, 0.88)

        # 5) Active: pitch forward (back to hover-ish thrust)
        print("[phase] active: pitch forward")
        stream_for(active_s, roll=roll_cmd, pitch=pitch_forward, thrust=hover_thrust, yaw_rate=yaw_rate_cmd)

        # 6) Cooldown: neutral
        print("[phase] cooldown: neutral")
        stream_for(cooldown_s, roll=0.0, pitch=0.0, thrust=hover_thrust, yaw_rate=0.0)

        # 7) Disarm request (may be denied if still airborne; that's ok)
        request_disarm(mav)
        print("[done]")

    finally:
        # Avoid abrupt stop (best effort)
        try:
            stream_for(0.3, roll=0.0, pitch=0.0, thrust=hover_thrust, yaw_rate=0.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()