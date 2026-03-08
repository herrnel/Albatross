from dataclasses import dataclass, field
import numpy as np
from typing import Optional
import threading
from collections import deque

@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    latest_command: Command = field(default_factory=Command)
    latest_local_pos: Optional[LocalPositionNED] = None
    latest_attitude: Optional[AttitudeData] = None
    latest_heartbeat = None
    imu_buffer: deque = field(default_factory=lambda: deque(maxlen=4000))

    def set_command(self, cmd: Command) -> None:
        with self.lock:
            self.latest_command = cmd

    def get_command(self) -> Command:
        with self.lock:
            return Command(
                roll=self.latest_command.roll,
                pitch=self.latest_command.pitch,
                yaw_angle=self.latest_command.yaw_angle,
                yaw_rate=self.latest_command.yaw_rate,
                thrust=self.latest_command.thrust,
                t=self.latest_command.t,
            )

    def push_imu(self, sample: ImuSample) -> None:
        with self.lock:
            self.imu_buffer.append(sample)

    def set_local_pos(self, pos: LocalPositionNED) -> None:
        with self.lock:
            self.latest_local_pos = pos

    def get_local_pos(self) -> Optional[LocalPositionNED]:
        with self.lock:
            return self.latest_local_pos

    def set_attitude(self, att: AttitudeData) -> None:
        with self.lock:
            self.latest_attitude = att

    def set_heartbeat(self, hb) -> None:
        with self.lock:
            self.latest_heartbeat = hb

    def get_heartbeat(self):
        with self.lock:
            return self.latest_heartbeat

