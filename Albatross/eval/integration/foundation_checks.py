from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Optional
import math
import time


@dataclass(frozen=True)
class TelemetrySnapshot:
    t_ns: int
    x_m: float
    y_m: float
    z_m: float
    vx_mps: Optional[float]
    vy_mps: Optional[float]
    vz_mps: Optional[float]
    roll_rad: Optional[float]
    pitch_rad: Optional[float]
    yaw_rad: Optional[float]


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    details: str


@dataclass
class PhaseWindow:
    name: str
    samples: list[TelemetrySnapshot]

    @property
    def start(self) -> TelemetrySnapshot:
        return self.samples[0]

    @property
    def end(self) -> TelemetrySnapshot:
        return self.samples[-1]

    def max_abs_roll(self) -> float:
        vals = [abs(s.roll_rad) for s in self.samples if s.roll_rad is not None]
        return max(vals) if vals else 0.0

    def max_abs_pitch(self) -> float:
        vals = [abs(s.pitch_rad) for s in self.samples if s.pitch_rad is not None]
        return max(vals) if vals else 0.0

    def z_delta(self) -> float:
        return self.end.z_m - self.start.z_m

    def yaw_delta(self) -> float:
        if self.start.yaw_rad is None or self.end.yaw_rad is None:
            return 0.0
        return shortest_angle_delta(self.start.yaw_rad, self.end.yaw_rad)

    def pitch_delta(self) -> float:
        if self.start.pitch_rad is None or self.end.pitch_rad is None:
            return 0.0
        return self.end.pitch_rad - self.start.pitch_rad

    def roll_delta(self) -> float:
        if self.start.roll_rad is None or self.end.roll_rad is None:
            return 0.0
        return self.end.roll_rad - self.start.roll_rad


def shortest_angle_delta(a0: float, a1: float) -> float:
    d = a1 - a0
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def sample_telemetry(shared_state) -> Optional[TelemetrySnapshot]:
    pos, _, _ = shared_state.sensors.local_pos.get()
    att, _, _ = shared_state.sensors.attitude.get()

    if pos is None:
        return None

    return TelemetrySnapshot(
        t_ns=time.perf_counter_ns(),
        x_m=pos.x_m,
        y_m=pos.y_m,
        z_m=pos.z_m,
        vx_mps=pos.vx_mps,
        vy_mps=pos.vy_mps,
        vz_mps=pos.vz_mps,
        roll_rad=att.roll_rad if att is not None else None,
        pitch_rad=att.pitch_rad if att is not None else None,
        yaw_rad=att.yaw_rad if att is not None else None,
    )


def check_hover_stability(
    window: PhaseWindow,
    max_abs_z_delta_m: float = 0.20,
    max_abs_roll_rad: float = 0.35,
    max_abs_pitch_rad: float = 0.35,
) -> CheckResult:
    dz = abs(window.z_delta())
    roll = window.max_abs_roll()
    pitch = window.max_abs_pitch()

    passed = (
        dz <= max_abs_z_delta_m
        and roll <= max_abs_roll_rad
        and pitch <= max_abs_pitch_rad
    )

    return CheckResult(
        name=window.name,
        passed=passed,
        details=(
            f"hover dz={dz:.3f}m "
            f"max_abs_roll={roll:.3f}rad "
            f"max_abs_pitch={pitch:.3f}rad"
        ),
    )


def check_climb_response(
    window: PhaseWindow,
    min_abs_z_delta_m: float = 0.20,
    expect_negative_z_for_climb: bool = True,
) -> CheckResult:
    dz = window.z_delta()
    enough_motion = abs(dz) >= min_abs_z_delta_m
    correct_sign = (dz < 0.0) if expect_negative_z_for_climb else (dz > 0.0)

    passed = enough_motion and correct_sign

    return CheckResult(
        name=window.name,
        passed=passed,
        details=(
            f"climb dz={dz:.3f}m "
            f"enough_motion={enough_motion} "
            f"correct_sign={correct_sign}"
        ),
    )


def check_yaw_response(
    window: PhaseWindow,
    commanded_yaw_rate: float,
    min_abs_delta_rad: float = 0.20,
) -> CheckResult:
    dyaw = window.yaw_delta()
    enough_motion = abs(dyaw) >= min_abs_delta_rad
    correct_sign = (dyaw > 0.0) if commanded_yaw_rate > 0.0 else (dyaw < 0.0)

    passed = enough_motion and correct_sign

    return CheckResult(
        name=window.name,
        passed=passed,
        details=(
            f"yaw_delta={dyaw:.3f}rad "
            f"enough_motion={enough_motion} "
            f"correct_sign={correct_sign}"
        ),
    )


def check_pitch_attitude_response(
    window: PhaseWindow,
    commanded_pitch: float,
    min_abs_delta_rad: float = 0.05,
) -> CheckResult:
    dpitch = window.pitch_delta()
    enough_motion = abs(dpitch) >= min_abs_delta_rad
    correct_sign = (dpitch > 0.0) if commanded_pitch > 0.0 else (dpitch < 0.0)

    passed = enough_motion and correct_sign

    return CheckResult(
        name=window.name,
        passed=passed,
        details=(
            f"pitch_delta={dpitch:.3f}rad "
            f"enough_motion={enough_motion} "
            f"correct_sign={correct_sign}"
        ),
    )


def check_roll_attitude_response(
    window: PhaseWindow,
    commanded_roll: float,
    min_abs_delta_rad: float = 0.05,
) -> CheckResult:
    droll = window.roll_delta()
    enough_motion = abs(droll) >= min_abs_delta_rad
    correct_sign = (droll > 0.0) if commanded_roll > 0.0 else (droll < 0.0)

    passed = enough_motion and correct_sign

    return CheckResult(
        name=window.name,
        passed=passed,
        details=(
            f"roll_delta={droll:.3f}rad "
            f"enough_motion={enough_motion} "
            f"correct_sign={correct_sign}"
        ),
    )