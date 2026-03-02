from pymavlink import mavutil

def make_mavlink_connection(endpoint: str, heartbeat_timeout_s: float = 10.0):
    mav = mavutil.mavlink_connection(endpoint)
    mav.wait_heartbeat(timeout=heartbeat_timeout_s)
    print("Heartbeat from", mav.target_system, mav.target_component)
    return mav