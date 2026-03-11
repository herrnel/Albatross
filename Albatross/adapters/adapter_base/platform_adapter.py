from abc import ABC, abstractmethod

from core.types import Command, SharedState

class PlatformAdapter(ABC):
    """
    SimulatorAdapter and RealDroneAdapter both implement this.
    They are responsible ONLY for IO with the platform.
    """
    @abstractmethod
    def setup(
        self,
        shared_state: SharedState,
        endpoint: str = "udpin:0.0.0.0:14540",
        heartbeat_timeout: float = 10.0,
        default_yaw_angle: float = 0.0, 
        ) -> None: 
        ...
        
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
    def is_armed_from_heartbeat(self, hb) -> bool: ...
    
    @abstractmethod
    def request_arm(self) -> None: ...
    
    @abstractmethod
    def request_offboard(self) -> None: ...
    
    @abstractmethod
    def request_disarm(self) -> None: ...


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
    def pump_sensors(self, max_msgs: int = 200) -> int:
        """
        Non-blocking (or short-blocking) call that reads any available sensor data
        and pushes ImuMsg/FrameMsg/etc. into the bus.
        """
        ...

    @abstractmethod
    def apply_action(self, action: Command) -> None:
        """Send action to the platform (attitude + throttle)."""
        ...