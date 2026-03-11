import time
import numpy as np
from typing import Optional
from pymavlink import mavutil
from core.types import Command, SharedState, ImuSample, AttitudeData, LocalPositionNED
from adapters import PlatformAdapter
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
        
        hb = self.mav_connection.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError("No heartbeat received. Wrong port or PX4 not running?")
        print("[info] heartbeat OK from sysid/compid:", self.mav_connection.target_system, self.mav_connection.target_component)
    
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
    
    
    def request_arm(self) -> None:
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
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
            0, 0, 0, 0, 0, 0, 0
        )
        print("[cmd] requested DISARM")
    
    def pump_sensors(self, max_msgs: int = 200) -> int:
        """
        Drain all currently available MAVLink messages without blocking.
        """
        count = 0
        while count < max_msgs:
            msg = self.mav_connection.recv_match(blocking=False)
            if msg is None:
                break

            mtype = msg.get_type()

            if mtype == "HIGHRES_IMU":
                t = getattr(msg, "time_usec", 0) * 1e-6 if getattr(msg, "time_usec", 0) else time.perf_counter()
                self.shared_state.push_imu(
                    ImuSample(
                        t=t,
                        accel=np.array([msg.xacc, msg.yacc, msg.zacc], dtype=np.float32),
                        gyro=np.array([msg.xgyro, msg.ygyro, msg.zgyro], dtype=np.float32),
                    )
                )

            elif mtype == "LOCAL_POSITION_NED":
                t = getattr(msg, "time_boot_ms", 0) * 1e-3 if getattr(msg, "time_boot_ms", 0) else time.perf_counter()
                self.shared_state.set_local_pos(
                    LocalPositionNED(
                        t=t,
                        x=float(msg.x),
                        y=float(msg.y),
                        z=float(msg.z),
                    )
                )

            elif mtype == "ATTITUDE":
                t = getattr(msg, "time_boot_ms", 0) * 1e-3 if getattr(msg, "time_boot_ms", 0) else time.perf_counter()
                self.shared_state.set_attitude(
                    AttitudeData(
                        t=t,
                        roll=float(msg.roll),
                        pitch=float(msg.pitch),
                        yaw=float(msg.yaw),
                        rollspeed=float(msg.rollspeed),
                        pitchspeed=float(msg.pitchspeed),
                        yawspeed=float(msg.yawspeed),
                    )
                )

            elif mtype == "HEARTBEAT":
                self.shared_state.set_heartbeat(msg)

            count += 1

        return count
