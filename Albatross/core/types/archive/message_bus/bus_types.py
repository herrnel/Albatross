from collections import deque
from threading import Lock
from typing import Generic, TypeVar, Optional, List

T = TypeVar("T")

# This file has been conceptually replaced by the sharedstate module. 

class LatestValue(Generic[T]):
    def __init__(self):
        self._lock = Lock()
        self._v: Optional[T] = None

    def set(self, v: T) -> None:
        with self._lock:
            self._v = v

    def get(self) -> Optional[T]:
        with self._lock:
            return self._v

class RingBuffer(Generic[T]):
    def __init__(self, maxlen: int):
        self._lock = Lock()
        self._buf = deque(maxlen=maxlen)

    def push(self, v: T) -> None:
        with self._lock:
            self._buf.append(v)

    def get_after(self, t0: float) -> List[T]:
        with self._lock:
            return [x for x in self._buf if getattr(x, "t") > t0]
        
        
class Bus:
    def __init__(self):
        self.imu = RingBuffer[ImuMsg](maxlen=8000)
        self.frame_latest = LatestValue[FrameMsg]()
        self.vision_latest = LatestValue()   # e.g., GateMeasMsg
        self.state_latest = LatestValue()    # e.g., StateMsg
        self.action_latest = LatestValue[ActionMsg]()