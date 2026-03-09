from core.types.module.module_type import FixedRateThread
from core.types.telemetry.shared_state import SharedState
import threading
from threading import Thread, Event
import time
from core.utilities.utilities import monotonic_sleep_until

class ControlModule(FixedRateThread):
    def __init__(self, shared_state: SharedState, hz: float = 500.0):
        super().__init__(hz=hz, name="Control")
        self.shared_state = shared_state
        self.name = "control"
        self._last_vision = None

    #  This is how controller should work if we are using stepping
    # def step(self):
    #     st = self.shared_state.state_latest.get()
    #     if st is None:
    #         return

    #     v = self.shared_state.vision_latest.get()
    #     if v is not None:
    #         self._last_vision = v

    #     # if no vision, do search
    #     if self._last_vision is None:
    #         act = Command(t=time.perf_counter(), roll=0.0, pitch=0.0, yaw_rate=0.6, throttle=0.55)
    #         self.bus.action_latest.set(act)
    #         return

    #     # simple guidance using bearing/elevation
    #     yaw_err = self._last_vision.bearing
    #     pitch_err = -self._last_vision.elevation

    #     roll = float(np.clip(2.0 * yaw_err, -0.6, 0.6))
    #     pitch = float(np.clip(0.2 + 2.0 * pitch_err, -0.6, 0.6))
    #     yaw_rate = float(np.clip(2.0 * yaw_err, -2.0, 2.0))
    #     throttle = float(np.clip(0.55, 0.0, 1.0))

    #     self.shared_state.action_latest.set(Command(
    #         t=time.perf_counter(), roll=roll, pitch=pitch, yaw_rate=yaw_rate, throttle=throttle
    # ))
    
    def create_thread(self, shared_state: SharedState, stop_evt: Event) -> Thread: 
        """
        How a thread is created for a module should be unique to that module. 
        """
        return threading.Thread(target=self.control_loop, args=(shared_state, stop_evt), daemon=True)
        
    def control_loop(shared: SharedState, stop_evt: threading.Event, control_hz: float = 500.0):
        dt = 1.0 / control_hz
        next_t = time.perf_counter()

        yaw_angle_cmd = 0.0
        yaw_rate_cmd = 0.0
        hover_thrust = 0.74
        pitch_forward = math.radians(-10.0)

        while not stop_evt.is_set():
            if mode == "scripted":
                cmd = Command(
                    roll=0.0,
                    pitch=pitch_forward,
                    yaw_angle=yaw_angle_cmd,
                    yaw_rate=yaw_rate_cmd,
                    thrust=hover_thrust,
                    t=time.perf_counter(),
                )
            else:
                # Placeholder for future control logic
                cmd = Command(
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle=yaw_angle_cmd,
                    yaw_rate=0.0,
                    thrust=hover_thrust,
                    t=time.perf_counter(),
                )

            shared.set_command(cmd)

            next_t += dt
            monotonic_sleep_until(next_t)