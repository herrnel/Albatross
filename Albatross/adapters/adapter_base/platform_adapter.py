from abc import ABC, abstractmethod

from core.types.message_bus.bus_types import Bus
from core.types.command.action_message import ActionMsg

class PlatformAdapter(ABC):
    """
    SimulatorAdapter and RealDroneAdapter both implement this.
    They are responsible ONLY for IO with the platform.
    """
    
    @abstractmethod
    def connect(self) -> None: ...
    
    @abstractmethod
    def stream_for(self, t0: float,  seconds: float, roll: float, pitch: float, thrust: float, yaw_rate: float) -> None: ...
    
    @abstractmethod
    def send_attitude_target(
        self,
        t0: float,
        roll: float,
        pitch: float,
        yaw_angle: float,
        yaw_rate: float,
        thrust: float,
    ) -> None:  ...
    
    @abstractmethod
    def request_arm(self) -> None: ...
    
    
    @abstractmethod
    def request_offboard() -> None: ...
    
    @abstractmethod
    def request_disarm() -> None: ...


    @abstractmethod
    def now(self) -> float:
        """Monotonic time in seconds."""
        ...

    @abstractmethod
    def start(self) -> None:
        """Initialize connections/resources."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Clean up resources."""
        ...

    @abstractmethod
    # This works for both push and step api implementations
    def pump_sensors(self, bus: Bus) -> None:
        """
        Non-blocking (or short-blocking) call that reads any available sensor data
        and pushes ImuMsg/FrameMsg/etc. into the bus.
        """
        ...

    @abstractmethod
    def apply_action(self, action: ActionMsg) -> None:
        """Send action to the platform (attitude + throttle)."""
        ...