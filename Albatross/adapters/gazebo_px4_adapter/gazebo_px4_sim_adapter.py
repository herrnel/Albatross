import time
from typing import Optional
from pymavlink import mavutil
from core.types.command.action_message import Command
from adapter_base.platform_adapter import PlatformAdapter
from core.utilities.utilities import quat_from_euler

class GazeboPx4MavlinkAdapter(PlatformAdapter):
    """
    MAVLink adapter for PX4 SITL/real that:
    - streams SET_ATTITUDE_TARGET via apply_action()
    - drains telemetry via pump_sensors()

    This is NOT "step-based". Telemetry arrives continuously; pump_sensors flushes it.
    """

    def __init__(
        self,
        endpoint: str = "udpin:0.0.0.0:14540",
        heartbeat_timeout: float = 10.0,
        default_yaw_angle: float = 0.0, 
    ):
        self.endpoint = endpoint
        self.heartbeat_timeout = heartbeat_timeout
        self.default_yaw_angle = default_yaw_angle # This may never change since we have the yaw_rate to use. 

        self.mav_connection: Optional[mavutil.mavfile] = None
        self._t0_wall: Optional[float] = None
        self._started = False

        # last command (optional: resend if control loop pauses)
        self._last_action: Optional[Command] = None

    def connect(self) -> None:
        endpoint = "udpin:0.0.0.0:14540"

        print("[info] connecting:", endpoint)
        mav_connection = mavutil.mavlink_connection(endpoint)
        
        hb = mav_connection.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError("No heartbeat received. Wrong port or PX4 not running?")
        print("[info] heartbeat OK from sysid/compid:", mav_connection.target_system, mav_connection.target_component)
    
    # This essentially what the control loop in the pipeline class does FYI. 
    def stream_for(self, t0: float,  seconds: float, roll: float, pitch: float, thrust: float, yaw_rate: float):
        # Use a higher rate to avoid any “offboard loss” sensitivity
        hz = 50.0
        dt = 1.0 / hz
        
        end = time.time() + seconds
        last_hb_print = 0.0
        while time.time() < end:
            self.send_attitude_target(
                self.mav, t0,
                roll=roll,
                pitch=pitch,
                yaw_angle=self.default_yaw_angle,
                yaw_rate=yaw_rate,
                thrust=thrust
            )

            # print state about 1 Hz
            if time.time() - last_hb_print > 1.0:
                hb_now = self.mav_connection.recv_match(type="HEARTBEAT", blocking=False)
                if hb_now:
                    print(f"[hb] armed={self.is_armed_from_heartbeat(hb_now)} base_mode={hb_now.base_mode} custom_mode={hb_now.custom_mode}")
                last_hb_print = time.time()

            time.sleep(dt)
            
  
    def send_attitude_target(
        self,
        t0: float,
        roll: float,
        pitch: float,
        yaw_angle: float,
        yaw_rate: float,
        thrust: float,
    ):
        """
        Send SET_ATTITUDE_TARGET.

        - roll/pitch/yaw_angle are absolute attitude angles (rad) used to form quaternion.
        - yaw_rate is body yaw rate (rad/s) IF not ignored by type_mask.
        - thrust in [0,1].
        """
        thrust = max(0.0, min(1.0, float(thrust)))
        yaw_rate = float(yaw_rate)

        q = quat_from_euler(roll, pitch, yaw_angle)

        # Use attitude + yaw_rate + thrust. Ignore body roll/pitch rates.
        # bit0 ignore body_roll_rate, bit1 ignore body_pitch_rate
        # keep bit2 clear (use body_yaw_rate), bit3 clear (use attitude), bit4 clear (use thrust)
        type_mask = 0b00000011

        time_boot_ms = int((time.time() - t0) * 1000) & 0xFFFFFFFF

        self.mav_connection.mav.set_attitude_target_send(
            time_boot_ms,
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            type_mask,
            q,
            0.0,        # body_roll_rate ignored
            0.0,        # body_pitch_rate ignored
            yaw_rate,   # used
            thrust      # used
        )

    def is_armed_from_heartbeat(self, hb) -> bool:
        return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    
    
    def request_arm(self, mav: mavutil.mavfile) -> None:
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("[cmd] requested ARM")
        
        
    def request_offboard(mav: mavutil.mavfile) -> None:
        try:
            mav.set_mode("OFFBOARD")
            print("[cmd] requested OFFBOARD")
        except Exception as e:
            print("[cmd] OFFBOARD request failed:", e)
            
            
    def request_disarm(mav: mavutil.mavfile) -> None: 
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("[cmd] requested DISARM")
    

    def start(self) -> None:
        print("[info] connecting:", self.endpoint)
        self.mav = mavutil.mavlink_connection(self.endpoint)

        hb = self.mav.wait_heartbeat(timeout=self.heartbeat_timeout)
        if hb is None:
            raise RuntimeError("No heartbeat received. Wrong port or PX4 not running?")
        print("[info] heartbeat OK from sysid/compid:", self.mav.target_system, self.mav.target_component)

        self._t0_wall = time.time()
        self._started = True

        # (Optional) Set stream rates. PX4/Gazebo often already streams IMU/attitude.
        # You can request specific MAVLink message intervals if needed:
        # self._set_msg_interval(mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, 500)  # 500 Hz
        # self._set_msg_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 200)     # 200 Hz
        # self._set_msg_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)  # 50 Hz

    def stop(self) -> None:
        self._started = False
        # pymavlink doesn't always have a clean close; best effort:
        try:
            if self.mav is not None:
                self.mav.close()
        except Exception:
            pass
        self.mav = None

    def now(self) -> float:
        # Prefer monotonic clock for scheduling.
        return time.perf_counter()

    def apply_action(self, action: ActionMsg) -> None:
        if not self._started or self.mav is None or self._t0_wall is None:
            return

        # Save for potential resend behavior
        self._last_action = action

        send_attitude_target(
            self.mav,
            self._t0_wall,
            roll=action.roll,
            pitch=action.pitch,
            yaw_angle=action.yaw_angle,
            yaw_rate=action.yaw_rate,
            thrust=action.thrust,
        )

    def pump_sensors(self, bus: SimpleBus) -> None:
        """
        Drain all currently available MAVLink messages (non-blocking)
        and push normalized messages to your bus.

        Call this frequently (e.g., Runner IO loop at 200–500 Hz).
        """
        if not self._started or self.mav is None:
            return

        # Drain a bounded number per call to avoid infinite loops if flooded.
        # Increase if you expect high telemetry volume.
        max_msgs = 200
        drained = 0

        while drained < max_msgs:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

            mtype = msg.get_type()
            t = self._msg_time_seconds(msg)

            if mtype == "HIGHRES_IMU":
                # Units:
                # xacc/yacc/zacc: m/s^2
                # xgyro/ygyro/zgyro: rad/s
                imu = ImuMsg(
                    t=t,
                    accel=np.array([msg.xacc, msg.yacc, msg.zacc], dtype=np.float32),
                    gyro=np.array([msg.xgyro, msg.ygyro, msg.zgyro], dtype=np.float32),
                )
                bus.push_imu(imu)

            elif mtype == "ATTITUDE":
                att = AttitudeMsg(
                    t=t,
                    roll=float(msg.roll),
                    pitch=float(msg.pitch),
                    yaw=float(msg.yaw),
                    rollspeed=float(msg.rollspeed),
                    pitchspeed=float(msg.pitchspeed),
                    yawspeed=float(msg.yawspeed),
                )
                bus.set_attitude(att)

            elif mtype == "LOCAL_POSITION_NED":
                lp = LocalPosMsg(
                    t=t,
                    x=float(msg.x),
                    y=float(msg.y),
                    z=float(msg.z),
                )
                bus.set_local_pos(lp)

            # Add more telemetry parsing here as you need:
            # - "ODOMETRY"
            # - "ATTITUDE_QUATERNION"
            # - "RAW_IMU"
            # - motor outputs / rpm (if available)
            # - etc.

            drained += 1

    # -------- helpers --------

    def _msg_time_seconds(self, msg) -> float:
        """
        Convert MAVLink message time fields into seconds.
        Falls back to perf_counter if no timestamp exists.
        """
        # Many MAVLink messages include time_usec or time_boot_ms.
        if hasattr(msg, "time_usec") and msg.time_usec:
            return float(msg.time_usec) * 1e-6
        if hasattr(msg, "time_boot_ms") and msg.time_boot_ms is not None:
            return float(msg.time_boot_ms) * 1e-3

        # Fallback (not ideal for EKF latency compensation)
        return time.perf_counter()

    def _set_msg_interval(self, msg_id: int, rate_hz: float) -> None:
        """
        Ask PX4 to stream a specific message at rate_hz using MAV_CMD_SET_MESSAGE_INTERVAL.
        Works on many MAVLink stacks, but not all configurations.
        """
        if self.mav is None:
            return
        interval_us = int(1e6 / max(1e-3, rate_hz))
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )