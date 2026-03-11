import time
import threading
from abc import abstractmethod, ABC
from threading import Thread, Event
from core.types import SharedState


class Module(ABC):
    name: str
    
    @abstractmethod
    def __init__(self, shared_state: SharedState): ...

    @abstractmethod
    def start(self) -> None: ...
    
    @abstractmethod
    def create_thread(self, stop_evt: Event) -> Thread: ...
    
    @abstractmethod
    def stop(self) -> None: ...


class FixedRateThread:
    def __init__(self, hz: float, name: str):
        self.hz = hz
        self.period = 1.0 / hz
        self.name = name
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def should_stop(self) -> bool:
        return self._stop.is_set()

    def step(self):
        raise NotImplementedError

    def _run(self):
        next_t = time.perf_counter()
        while not self.should_stop():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
                continue
            next_t += self.period
            self.step()