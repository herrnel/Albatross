from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

from core.types.shared_data.topics import LastValueTopic, RingBufferTopic

# Telemetry / sensor payload types
from core.types.shared_data.base_types.telemetry.imu_types import IMUSample
from core.types.shared_data.base_types.telemetry.camera_types import CameraFrame, CameraInfo
from core.types.shared_data.base_types.telemetry.telemetry_types import (
    AttitudeData,
    LocalPositionNED,
    HeartbeatData,
    OdometryData,
    TimeSyncData,
    SystemStatusData,
    NavReferenceData,
)

# # Perception payload types
# from core.types.perception.segmentation_types import SegmentationResult
# from core.types.perception.gate_detection_types import GateDetection

# # Estimation payload types
# from core.types.estimation.gate_relative_state import GateRelativeState
# from core.types.estimation.fused_state import FusedState
from core.types.shared_data.base_types.estimation.observation import Observation

# # Planning payload types
# from core.types.planning.planner_types import PlannerState, RaceProgress, TargetGate

# Control / command payload types
from core.types.command.action_type import Action
from core.types.command.safe_action_type import SafeAction
from core.types.command.command_type import Command

# # Health / runtime payload types
from core.types.shared_data.base_types.health.module_health_types import ModuleHealth, LinkHealth

# # Competition / orchestration payload types
# from core.types.competition.run_state_types import RunPhase, RunStatus


@dataclass
class SensorTopics:
    """
    Topics populated primarily by adapters / I/O threads.
    """

    # High-rate streams
    imu: RingBufferTopic[IMUSample] = field(
        default_factory=lambda: RingBufferTopic[IMUSample](maxlen=4000)
    )
    camera_frame: RingBufferTopic[CameraFrame] = field(
        default_factory=lambda: RingBufferTopic[CameraFrame](maxlen=8)
    )

    # Latest-value sensor state
    attitude: LastValueTopic[AttitudeData] = field(
        default_factory=LastValueTopic[AttitudeData]
    )
    odometry: LastValueTopic[OdometryData] = field(
        default_factory=LastValueTopic[OdometryData]
    )
    local_pos: LastValueTopic[LocalPositionNED] = field(
        default_factory=LastValueTopic[LocalPositionNED]
    )
    system_status: LastValueTopic[SystemStatusData] = field( # In depth status of the connection gathered from telemetry and heartbeat - Good for printing and watchdogs. 
        default_factory=LastValueTopic[SystemStatusData]
    )
    nav_ref: LastValueTopic[NavReferenceData] = field(
        default_factory=LastValueTopic[NavReferenceData]
    )

    # Link-management / timing
    heartbeat_rx: LastValueTopic[HeartbeatData] = field( # Recieved Heartbeat - Used for debugging 
        default_factory=LastValueTopic[HeartbeatData]
    )
    heartbeat_tx: LastValueTopic[HeartbeatData] = field( # Transmitted Hearbeat - Used for debugging
        default_factory=LastValueTopic[HeartbeatData]
    )
    timesync: LastValueTopic[TimeSyncData] = field(
        default_factory=LastValueTopic[TimeSyncData]
    )

    # Camera metadata
    camera_info: LastValueTopic[CameraInfo] = field(
        default_factory=LastValueTopic[CameraInfo]
    )


# @dataclass
# class PerceptionTopics:
#     """
#     Topics produced by perception runtime modules.
#     """
#     segmentation: LastValueTopic[SegmentationResult] = field(
#         default_factory=LastValueTopic[SegmentationResult]
#     )
#     gate_detection: LastValueTopic[GateDetection] = field(
#         default_factory=LastValueTopic[GateDetection]
#     )


@dataclass
class EstimationTopics:
    """
    Topics produced by tracking / filtering / state-estimation modules.
    """
    # gate_relative: LastValueTopic[GateRelativeState] = field(
    #     default_factory=LastValueTopic[GateRelativeState]
    # )
    # fused_state: LastValueTopic[FusedState] = field(
    #     default_factory=LastValueTopic[FusedState]
    # )
    # fused_state_history: RingBufferTopic[FusedState] = field(
    #     default_factory=lambda: RingBufferTopic[FusedState](maxlen=512)
    # )
    observation: LastValueTopic[Observation] = field(
        default_factory=LastValueTopic[Observation]
    )


# @dataclass
# class PlanningTopics:
#     """
#     Topics produced by the planning layer.

#     This is here because the spec explicitly includes:
#     Vision + Telemetry -> Perception -> Planning -> Control -> Pilot Commands -> Stabilized Controller
#     """
#     planner_state: LastValueTopic[PlannerState] = field(
#         default_factory=LastValueTopic[PlannerState]
#     )
#     race_progress: LastValueTopic[RaceProgress] = field(
#         default_factory=LastValueTopic[RaceProgress]
#     )
#     target_gate: LastValueTopic[TargetGate] = field(
#         default_factory=LastValueTopic[TargetGate]
#     )


@dataclass
class ControlTopics:
    """
    Topics produced by policy / safety / command-publication layers.
    """
    policy_action: LastValueTopic[Action] = field(
        default_factory=LastValueTopic[Action]
    )
    safe_action: LastValueTopic[SafeAction] = field(
        default_factory=LastValueTopic[SafeAction]
    )

    # The command your stack wants to send next
    pilot_command: LastValueTopic[Command] = field(
        default_factory=LastValueTopic[Command]
    )

    # The command actually handed off to the transport layer
    command: LastValueTopic[Command] = field(
        default_factory=LastValueTopic[Command]
    )

    # Small history of transmitted (tx) commands for timing/debug/replay
    command_tx_history: RingBufferTopic[Command] = field(
        default_factory=lambda: RingBufferTopic[Command](maxlen=512)
    )


@dataclass
class HealthTopics:
    """
    Runtime health and diagnostics.
    """
    module_health: LastValueTopic[Dict[str, ModuleHealth]] = field(
        default_factory=LastValueTopic[Dict[str, ModuleHealth]]
    )
    # link_health: LastValueTopic[LinkHealth] = field(
    #     default_factory=LastValueTopic[LinkHealth]
    # )


# @dataclass
# class CompetitionTopics:
#     """
#     Competition/orchestrator state.
#     """
#     run_phase: LastValueTopic[RunPhase] = field(
#         default_factory=LastValueTopic[RunPhase]
#     )
#     run_status: LastValueTopic[RunStatus] = field(
#         default_factory=LastValueTopic[RunStatus]
#     )
#     finish_detected: LastValueTopic[bool] = field(
#         default_factory=LastValueTopic[bool]
#     )
#     deadline_ns: LastValueTopic[int] = field(
#         default_factory=LastValueTopic[int]
#     )


class SharedState:
    """
    Top-level registry of typed topic stores.

    Notes:
    - No single global lock
    - Each topic owns its own lock
    - High-rate streams use RingBufferTopic[T]
    - Latest-value state uses LastValueTopic[T]
    """

    def __init__(self) -> None:
        self.sensors = SensorTopics()
        # self.perception = PerceptionTopics()
        self.estimation = EstimationTopics()
        # self.planning = PlanningTopics()
        self.control = ControlTopics()
        self.health = HealthTopics()
        # self.competition = CompetitionTopics()