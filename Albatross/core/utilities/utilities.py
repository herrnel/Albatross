import time

def monotonic_sleep_until(next_t: float) -> float:
    now = time.perf_counter()
    if now < next_t:
        time.sleep(next_t - now)
    return next_t