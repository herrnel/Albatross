import time
import math

def monotonic_sleep_until(next_t: float) -> float:
    now = time.perf_counter()
    if now < next_t:
        time.sleep(next_t - now)
    return next_t


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