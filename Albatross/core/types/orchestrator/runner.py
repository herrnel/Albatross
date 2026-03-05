import time

from adapters.adapter_base.platform_adapter import PlatformAdapter
from core.types.message_bus.bus_types import Bus

# Here make an attempt to have the runner loop run at 500hz but its not necessary. 
class Runner:
    def __init__(self, adapter: PlatformAdapter, bus: Bus, modules: list):
        self.adapter = adapter
        self.bus = bus
        self.modules = modules
        self._running = False

    def run(self, rate_hz: float = 500.0):
        period = 1.0 / rate_hz
        next_t = time.perf_counter()

        self.adapter.start()
        for m in self.modules:
            m.start()

        self._running = True
        try:
            while self._running:
                # 1) ingest and update bus
                self.adapter.pump_sensors(self.bus)

                # 2) apply latest action (if any)
                act = self.bus.action_latest.get()
                if act is not None:
                    self.adapter.apply_action(act)

                # fixed-rate scheduling
                now = time.perf_counter()
                if now < next_t:
                    time.sleep(next_t - now)
                next_t += period
        finally:
            for m in self.modules:
                m.stop()
            self.adapter.stop()