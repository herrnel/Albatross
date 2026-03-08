from core.types.module.module_type import FixedRateThread
from core.types.

class ControlModule(FixedRateThread):
    def __init__(self, bus: Bus, hz: float = 500.0):
        super().__init__(hz=hz, name="Control")
        self.bus = bus
        self._last_vision = None

    def step(self):
        st = self.bus.state_latest.get()
        if st is None:
            return

        v = self.bus.vision_latest.get()
        if v is not None:
            self._last_vision = v

        # if no vision, do search
        if self._last_vision is None:
            act = ActionMsg(t=time.perf_counter(), roll=0.0, pitch=0.0, yaw_rate=0.6, throttle=0.55)
            self.bus.action_latest.set(act)
            return

        # simple guidance using bearing/elevation
        yaw_err = self._last_vision.bearing
        pitch_err = -self._last_vision.elevation

        roll = float(np.clip(2.0 * yaw_err, -0.6, 0.6))
        pitch = float(np.clip(0.2 + 2.0 * pitch_err, -0.6, 0.6))
        yaw_rate = float(np.clip(2.0 * yaw_err, -2.0, 2.0))
        throttle = float(np.clip(0.55, 0.0, 1.0))

        self.bus.action_latest.set(ActionMsg(
            t=time.perf_counter(), roll=roll, pitch=pitch, yaw_rate=yaw_rate, throttle=throttle
        ))