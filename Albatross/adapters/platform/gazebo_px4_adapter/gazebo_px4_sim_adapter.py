import time
import numpy as np
from typing import Optional
from pymavlink import mavutil
from core.types.command.command_type import Command
from core.types.shared_data.base_types.telemetry.imu_types import IMUSample
from core.types.shared_data.base_types.telemetry.telemetry_types import AttitudeData, LocalPositionNED, HeartbeatData, TimeSyncData, OdometryData, SystemStatusData
from core.types.shared_data.shared_state import SharedState
from adapters.platform.adapter_base.platform_adapter import PlatformAdapter
from core.utilities import quat_from_euler


class GazeboPx4MavlinkAdapter(PlatformAdapter):
    """
    MAVLink adapter for PX4 SITL/real that:
    - streams SET_ATTITUDE_TARGET via set_attitude_target()
    - drains telemetry via pump_sensors()

    This is NOT "step-based". Telemetry arrives continuously; pump_sensors flushes it.
    """

    def setup(
        self,
        shared_state: SharedState,
        endpoint: str = "udpin:0.0.0.0:14540",
        heartbeat_timeout: float = 10.0,
        default_yaw_angle: float = 0.0, 
    ):
        self.shared_state = shared_state
        self.endpoint = endpoint
        self.heartbeat_timeout = heartbeat_timeout
        self.default_yaw_angle = default_yaw_angle # This may never change since we have the yaw_rate to use. 

        self.mav_connection: Optional[mavutil.mavfile] = None
        self._t0_wall: Optional[float] = None
        self._started = False

        # last command (optional: resend if control loop pauses)
        self._last_action: Optional[Command] = None
        
    def connect(self) -> None:
        print("[info] connecting:", self.endpoint)
        self.mav_connection = mavutil.mavlink_connection(self.endpoint)

        hb = self.mav_connection.wait_heartbeat(timeout=self.heartbeat_timeout)
        if hb is None:
            raise RuntimeError("No heartbeat received. Wrong port or PX4 not running?")

        self._t0_wall = time.time()
        now_ns = time.perf_counter_ns()

        hb_data = self._heartbeat_from_msg(hb, now_ns)
        self.shared_state.sensors.heartbeat_rx.publish(hb_data, t_ns=now_ns)

        print(
            "[info] heartbeat OK from sysid/compid:",
            self.mav_connection.target_system,
            self.mav_connection.target_component,
        )

            
    # -----------------------------
    # Outbound MAVLink
    # -----------------------------

    def send_heartbeat(self) -> None:
        """
        Client-side heartbeat TX helper.
        Call this from a dedicated heartbeat loop at >= 2 Hz.
        """
        if self.mav_connection is None:
            return

        now_ns = time.perf_counter_ns()

        self.mav_connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

        hb = HeartbeatData(
            t_ns=now_ns,
            system_id=None,
            component_id=None,
            armed=None,
            mode="client_tx",
            system_status="sent",
            mav_type=int(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER),
            autopilot=int(mavutil.mavlink.MAV_AUTOPILOT_INVALID),
            base_mode=0,
            custom_mode=0,
        )
        self.shared_state.sensors.heartbeat_tx.publish(hb, t_ns=now_ns)

    def send_command(self, cmd: Command) -> None:
        """
        Main outbound command API used by your command loop.
        """
        self._last_command = cmd
        self.send_attitude_target(cmd)

    def send_attitude_target(self, cmd: Command) -> None:
        """
        Send SET_ATTITUDE_TARGET using your internal Command type.
        """
        if self.mav_connection is None:
            raise RuntimeError("MAVLink connection is not initialized.")

        thrust = max(0.0, min(1.0, float(cmd.thrust)))
        yaw_rate = float(cmd.yaw_rate)

        q = quat_from_euler(
            float(cmd.roll),
            float(cmd.pitch),
            float(getattr(cmd, "yaw_angle", self.default_yaw_angle)),
        )

        # ignore body roll/pitch rates, use attitude + yaw_rate + thrust
        type_mask = 0b00000011

        if self._t0_wall is None:
            self._t0_wall = time.time()
        time_boot_ms = int((time.time() - self._t0_wall) * 1000) & 0xFFFFFFFF

        self.mav_connection.mav.set_attitude_target_send(
            time_boot_ms,
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            type_mask,
            q,
            0.0,
            0.0,
            yaw_rate,
            thrust,
        )

        now_ns = time.perf_counter_ns()
        self.shared_state.control.command_tx_history.publish(cmd, t_ns=now_ns)

    def request_arm(self) -> None:
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        print("[cmd] requested ARM")

    def request_offboard(self) -> None:
        try:
            self.mav_connection.set_mode("OFFBOARD")
            print("[cmd] requested OFFBOARD SUCCESSFUL")
        except Exception as e:
            print("[cmd] OFFBOARD request failed:", e)

    def request_disarm(self) -> None:
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )
        print("[cmd] requested DISARM")
        
    # -----------------------------
    # Inbound MAVLink telemetry
    # -----------------------------

    def pump_sensors(self, max_msgs: int = 200) -> int:
        """
        Drain available MAVLink messages without blocking and publish them into
        the new typed SharedState topics.
        """
        if self.mav_connection is None:
            return 0

        count = 0
        while count < max_msgs:
            msg = self.mav_connection.recv_match(blocking=False)
            if msg is None:
                break

            mtype = msg.get_type()
            now_ns = time.perf_counter_ns()

            if mtype == "HIGHRES_IMU":
                imu = IMUSample(
                    t_ns=now_ns,
                    accel_mps2=np.array([msg.xacc, msg.yacc, msg.zacc], dtype=np.float32),
                    gyro_rps=np.array([msg.xgyro, msg.ygyro, msg.zgyro], dtype=np.float32),
                )
                self.shared_state.sensors.imu.publish(imu, t_ns=now_ns)

            elif mtype == "LOCAL_POSITION_NED":
                local_pos = LocalPositionNED(
                    t_ns=now_ns,
                    x_m=float(msg.x),
                    y_m=float(msg.y),
                    z_m=float(msg.z),
                    vx_mps=float(getattr(msg, "vx", 0.0)),
                    vy_mps=float(getattr(msg, "vy", 0.0)),
                    vz_mps=float(getattr(msg, "vz", 0.0)),
                )
                self.shared_state.sensors.local_pos.publish(local_pos, t_ns=now_ns)

            elif mtype == "ATTITUDE":
                att = AttitudeData(
                    t_ns=now_ns,
                    roll_rad=float(msg.roll),
                    pitch_rad=float(msg.pitch),
                    yaw_rad=float(msg.yaw),
                    rollspeed_rps=float(msg.rollspeed),
                    pitchspeed_rps=float(msg.pitchspeed),
                    yawspeed_rps=float(msg.yawspeed),
                    quat_wxyz=None,
                    imu=None,
                )
                self.shared_state.sensors.attitude.publish(att, t_ns=now_ns)

            elif mtype == "HEARTBEAT":
                hb = self._heartbeat_from_msg(msg, now_ns)
                self.shared_state.sensors.heartbeat_rx.publish(hb, t_ns=now_ns)

                system_status = SystemStatusData(
                    t_ns=now_ns,
                    armed=self.is_armed_from_heartbeat(msg),
                    offboard_enabled=self._is_offboard_from_heartbeat(msg),
                    guided_enabled=None,
                    failsafe=None,
                    mode=self._mode_string_from_heartbeat(msg),
                    system_status=str(getattr(msg, "system_status", None)),
                    raw={
                        "base_mode": int(getattr(msg, "base_mode", 0)),
                        "custom_mode": int(getattr(msg, "custom_mode", 0)),
                        "system_status": int(getattr(msg, "system_status", 0)),
                    },
                )
                self.shared_state.sensors.system_status.publish(system_status, t_ns=now_ns)

            elif mtype == "ODOMETRY":
                odom = OdometryData(
                    t_ns=now_ns,
                    pos_m=np.array([msg.x, msg.y, msg.z], dtype=np.float32)
                    if all(hasattr(msg, f) for f in ("x", "y", "z")) else None,
                    quat_wxyz=np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]], dtype=np.float32)
                    if hasattr(msg, "q") and msg.q is not None else None,
                    vel_mps=np.array([msg.vx, msg.vy, msg.vz], dtype=np.float32)
                    if all(hasattr(msg, f) for f in ("vx", "vy", "vz")) else None,
                    ang_vel_rps=np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed], dtype=np.float32)
                    if all(hasattr(msg, f) for f in ("rollspeed", "pitchspeed", "yawspeed")) else None,
                    pose_frame=str(getattr(msg, "frame_id", None)),
                    velocity_frame=str(getattr(msg, "child_frame_id", None)),
                )
                self.shared_state.sensors.odometry.publish(odom, t_ns=now_ns)

            elif mtype == "TIMESYNC":
                ts = TimeSyncData(
                    t_ns=now_ns,
                    remote_time_ns=int(getattr(msg, "tc1", 0)) if hasattr(msg, "tc1") else None,
                    local_echo_ns=int(getattr(msg, "ts1", 0)) if hasattr(msg, "ts1") else None,
                    offset_ns=None,
                    round_trip_ns=None,
                )
                self.shared_state.sensors.timesync.publish(ts, t_ns=now_ns)

            count += 1

        return count
    
    # -----------------------------
    # Helpers
    # -----------------------------

    def is_armed_from_heartbeat(self, hb) -> bool:
        return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def _heartbeat_from_msg(self, msg, now_ns: int) -> HeartbeatData:
        return HeartbeatData(
            t_ns=now_ns,
            system_id=getattr(msg, "_header", None).srcSystem if getattr(msg, "_header", None) else None,
            component_id=getattr(msg, "_header", None).srcComponent if getattr(msg, "_header", None) else None,
            armed=self.is_armed_from_heartbeat(msg),
            mode=self._mode_string_from_heartbeat(msg),
            system_status=str(getattr(msg, "system_status", None)),
            mav_type=int(getattr(msg, "type", 0)) if hasattr(msg, "type") else None,
            autopilot=int(getattr(msg, "autopilot", 0)) if hasattr(msg, "autopilot") else None,
            base_mode=int(getattr(msg, "base_mode", 0)) if hasattr(msg, "base_mode") else None,
            custom_mode=int(getattr(msg, "custom_mode", 0)) if hasattr(msg, "custom_mode") else None,
        )

    def _mode_string_from_heartbeat(self, msg) -> str:
        base_mode = int(getattr(msg, "base_mode", 0))
        custom_mode = int(getattr(msg, "custom_mode", 0))
        return f"base={base_mode},custom={custom_mode}"

    def _is_offboard_from_heartbeat(self, msg) -> bool:
        """
        Placeholder. PX4 custom_mode decoding is stack-specific.
        Keep this conservative until you confirm the exact custom_mode mapping.
        """
        # For now, just expose false unless you later decode PX4 custom_mode properly.
        return False
    