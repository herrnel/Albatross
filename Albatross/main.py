# main.py
import time

from core.types.data import Action
from adapters.mavsdk_px4_adapter.gst_camera_adapter import GstCameraAdapter, GstCameraConfig
from adapters.mavsdk_px4_adapter.mavsdk_px4_unified_adapter import UnifiedPx4GzAdapter


def main():
    # One shared MAVLink endpoint.
    # PX4 log says Onboard remote port 14540, so we listen there.
    # IMPORTANT: use udpin/udp to BIND ONCE.
    adapter = UnifiedPx4GzAdapter(
        mav_endpoint="udpin:0.0.0.0:14540",
    )
    adapter.connect()
    adapter.sender.begin_offboard_and_arm()

    # (Optional) If you implemented offboard/arm helpers inside adapter.sender:
    # adapter.sender.begin_offboard_and_arm()

    hz = 30.0
    dt = 1.0 / hz
    try:
        # while True:
        #     last = 0
        #     if time.time() - last > 1.0:
        #         print_hb(adapter.mav)   # expose mav on unified adapter
        #         last = time.time()
                
        #     raw = adapter.read()

        #     # dummy policy
        #     action = Action(
        #         t=raw.telemetry.t,
        #         throttle=0.55,
        #         roll=0.0,
        #         pitch=0.0,
        #         yaw=0.0,
        #     )

        #     adapter.send(action)
        #     time.sleep(dt)
        
        t0 = time.time()
        while time.time() - t0 < 2.0:
            raw = adapter.read()
            action = Action(t=raw.telemetry.t, throttle=0.6, roll=0.0, pitch=0.0, yaw=0.0)
            adapter.send(action)
            time.sleep(1/30)
    finally:
        adapter.close()
        
        
def print_hb(mav):
    msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg is None:
        print("No heartbeat")
        return
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"HB armed={armed} base_mode={msg.base_mode} custom_mode={msg.custom_mode}")       


if __name__ == "__main__":
    main()