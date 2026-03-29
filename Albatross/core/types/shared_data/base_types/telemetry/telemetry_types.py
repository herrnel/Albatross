from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from core.types.shared_data.base_types.telemetry.imu_types import IMUSample


@dataclass(frozen=True)
class AttitudeData:
    """
    Normalized ATTITUDE message.

    The spec explicitly lists ATTITUDE as supported telemetry. Use this for the
    direct attitude stream even if richer pose data is also available elsewhere.
    """
    t_ns: int

    roll_rad: float
    pitch_rad: float
    yaw_rad: float

    rollspeed_rps: float
    pitchspeed_rps: float
    yawspeed_rps: float

    # Optional convenience fields if derived elsewhere
    quat_wxyz: Optional[np.ndarray] = None     # shape (4,)
    imu: Optional[IMUSample] = None


@dataclass(frozen=True)
class LocalPositionNED:
    """
    Local position in simulator Cartesian frame.

    The spec says the simulator uses a local Cartesian coordinate system and
    does not expose absolute global position. This type is still useful if your
    bridge provides local position-like telemetry. :contentReference[oaicite:6]{index=6}
    """
    t_ns: int
    x_m: float
    y_m: float
    z_m: float

    vx_mps: Optional[float] = None
    vy_mps: Optional[float] = None
    vz_mps: Optional[float] = None


@dataclass(frozen=True)
class HeartbeatData:
    """
    Normalized HEARTBEAT state.

    The spec requires HEARTBEAT support and a minimum heartbeat rate of 2 Hz,
    and says the client must maintain heartbeat messages. 
    """
    t_ns: int

    system_id: Optional[int] = None
    component_id: Optional[int] = None

    armed: Optional[bool] = None
    mode: Optional[str] = None
    system_status: Optional[str] = None

    mav_type: Optional[int] = None
    autopilot: Optional[int] = None
    base_mode: Optional[int] = None
    custom_mode: Optional[int] = None


@dataclass(frozen=True)
class OdometryData:
    """
    Normalized ODOMETRY-style pose/twist state.

    The spec lists ODOMETRY and says telemetry includes orientation and linear
    velocities, so this is the cleanest internal place for that richer state. 
    """
    t_ns: int

    # Position in local frame (if provided)
    pos_m: Optional[np.ndarray] = None         # shape (3,)

    # Orientation quaternion [w, x, y, z], shape (4,)
    quat_wxyz: Optional[np.ndarray] = None     # shape (4,)

    # Linear velocity
    vel_mps: Optional[np.ndarray] = None       # shape (3,)

    # Angular velocity
    ang_vel_rps: Optional[np.ndarray] = None   # shape (3,)

    # Optional frame metadata
    pose_frame: Optional[str] = None
    velocity_frame: Optional[str] = None


@dataclass(frozen=True)
class TimeSyncData:
    """
    Normalized TIMESYNC data.

    The spec explicitly lists TIMESYNC under supported MAVLink messages. Keep
    this simple until you inspect the actual packets. :contentReference[oaicite:9]{index=9}
    """
    t_ns: int

    remote_time_ns: Optional[int] = None
    local_echo_ns: Optional[int] = None
    offset_ns: Optional[int] = None
    round_trip_ns: Optional[int] = None


@dataclass(frozen=True)
class SystemStatusData:
    """
    Normalized system status flags.

    The spec explicitly lists 'system status flags' under telemetry. :contentReference[oaicite:10]{index=10}
    """
    t_ns: int

    armed: Optional[bool] = None
    offboard_enabled: Optional[bool] = None
    guided_enabled: Optional[bool] = None
    failsafe: Optional[bool] = None

    # Preserve raw status/mode fields if you need them later
    mode: Optional[str] = None
    system_status: Optional[str] = None

    raw: dict = field(default_factory=dict)


@dataclass(frozen=True)
class NavReferenceData:
    """
    Placeholder for 'simulator navigation reference data'.

    The spec names this concept but does not define the fields yet, so keep this
    intentionally lightweight until live integration confirms what is present. :contentReference[oaicite:11]{index=11}
    """
    t_ns: int

    frame_name: Optional[str] = None
    origin: Optional[np.ndarray] = None        # shape (3,)
    axes_description: Optional[str] = None
    raw: dict = field(default_factory=dict)