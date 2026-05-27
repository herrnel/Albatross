from __future__ import annotations

import time
import numpy as np

from core.types.telemetry.imu_types import IMUSample
from core.types.telemetry.telemetry_types import (
    AttitudeData,
    LocalPositionNED,
    HeartbeatData,
    OdometryData,
    TimeSyncData,
)


class ReplayPlatformAdapter:
    def __init__(self, shared_state):
        self.shared_state = shared_state

    def publish_record(self, record):
        topic = record.topic
        p = record.payload

        if topic == "sensors.imu":
            obj = IMUSample(
                t_ns=record.t_ref_ns,
                accel_mps2=np.array(p["accel_mps2"], dtype=np.float32),
                gyro_rps=np.array(p["gyro_rps"], dtype=np.float32),
            )
            self.shared_state.sensors.imu.publish(obj, t_ns=record.t_publish_ns)

        elif topic == "sensors.attitude":
            obj = AttitudeData(
                t_ns=record.t_ref_ns,
                roll_rad=p["roll_rad"],
                pitch_rad=p["pitch_rad"],
                yaw_rad=p["yaw_rad"],
                rollspeed_rps=p["rollspeed_rps"],
                pitchspeed_rps=p["pitchspeed_rps"],
                yawspeed_rps=p["yawspeed_rps"],
                quat_wxyz=np.array(p["quat_wxyz"], dtype=np.float32) if p.get("quat_wxyz") is not None else None,
                imu=None,
            )
            self.shared_state.sensors.attitude.publish(obj, t_ns=record.t_publish_ns)

        elif topic == "sensors.local_pos":
            obj = LocalPositionNED(**p)
            self.shared_state.sensors.local_pos.publish(obj, t_ns=record.t_publish_ns)

        elif topic == "sensors.heartbeat_rx":
            obj = HeartbeatData(**p)
            self.shared_state.sensors.heartbeat_rx.publish(obj, t_ns=record.t_publish_ns)

        elif topic == "sensors.odometry":
            obj = OdometryData(
                t_ns=p["t_ns"],
                pos_m=np.array(p["pos_m"], dtype=np.float32) if p.get("pos_m") is not None else None,
                quat_wxyz=np.array(p["quat_wxyz"], dtype=np.float32) if p.get("quat_wxyz") is not None else None,
                vel_mps=np.array(p["vel_mps"], dtype=np.float32) if p.get("vel_mps") is not None else None,
                ang_vel_rps=np.array(p["ang_vel_rps"], dtype=np.float32) if p.get("ang_vel_rps") is not None else None,
                pose_frame=p.get("pose_frame"),
                velocity_frame=p.get("velocity_frame"),
            )
            self.shared_state.sensors.odometry.publish(obj, t_ns=record.t_publish_ns)

        elif topic == "sensors.timesync":
            obj = TimeSyncData(**p)
            self.shared_state.sensors.timesync.publish(obj, t_ns=record.t_publish_ns)