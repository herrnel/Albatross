from abc import ABC, abstractmethod

from core.types.message_bus.bus_types import Bus
from core.types.command.action_message import ActionMsg

class PlatformAdapter(ABC):
    """
    SimulatorAdapter and RealDroneAdapter both implement this.
    They are responsible ONLY for IO with the platform.
    """

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