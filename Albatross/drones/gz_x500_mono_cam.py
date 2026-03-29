import time

from drones.drone_base.drone_base import Drone
from core import Pipeline
from adapters.platform import PlatformAdapter
from core.types import Command


class gz_x500_mono_cam(Drone):
    """
    Drone profile for the Gazebo x500 mono-camera setup.

    Responsibilities:
    - store vehicle-specific tuning values
    - expose common high-level vehicle actions
    - publish simple direct commands into shared state for startup/testing

    Notes:
    - In the new architecture, direct command helpers publish into
      shared_state.control.pilot_command and shared_state.control.command.
    - Longer term, SafetyModule should usually own final command publication.
    """

    def setup(self, adapter: PlatformAdapter, pipeline: Pipeline) -> None:
        self.adapter: PlatformAdapter = adapter
        self.pipeline: Pipeline = pipeline

        # Thrust tuning
        self.hover_thrust = 0.74
        self.takeoff_thrust = 0.87

    # -------------------------
    # Core runtime functionality
    # -------------------------

    def start_processing(self) -> None:
        self.pipeline.start_processing()

    def take_control(self) -> None:
        self.pipeline.take_control()

    def arm(self) -> None:
        self.adapter.request_arm()

    def offboard(self) -> None:
        self.adapter.request_offboard()

    def disarm(self) -> None:
        self.adapter.request_disarm()

    # -------------------------
    # Direct command helpers
    # -------------------------

    def hover(self) -> None:
        """
        Publish a neutral hover command into shared state.

        Useful for:
        - startup warmup streaming
        - fallback/default command
        - simple testing before full autonomy is active
        """
        now_ns = time.perf_counter_ns()

        cmd = Command(
            roll=0.0,
            pitch=0.0,
            yaw_angle=0.0,
            yaw_rate=0.0,
            thrust=self.hover_thrust,
            t=now_ns / 1e9,
        )

        self._publish_command(cmd, now_ns)

    def bump_and_run(self) -> None:
        """
        Publish a simple takeoff / forward-flight command into shared state.

        This is still just a direct command helper, not a real autonomy policy.
        """
        now_ns = time.perf_counter_ns()

        cmd = Command(
            roll=0.0,
            pitch=0.0,
            yaw_angle=0.0,
            yaw_rate=0.0,
            thrust=self.takeoff_thrust,
            t=now_ns / 1e9,
        )

        self._publish_command(cmd, now_ns)

    # -------------------------
    # Internal helper
    # -------------------------

    def _publish_command(self, cmd: Command, now_ns: int) -> None:
        """
        Publish a command into the new shared-state topics.

        Why both?
        - pilot_command: semantic "desired command" layer
        - command: actual command loop reads this and streams it

        For now, publishing both keeps startup helpers simple.
        Later, SafetyModule can become the main publisher of `command`.
        """
        self.pipeline.shared_state.control.pilot_command.publish(cmd, t_ns=now_ns)
        self.pipeline.shared_state.control.command.publish(cmd, t_ns=now_ns)