from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path
import math

from colorama import Fore, Style

from core import Pipeline
from adapters import PlatformAdapter, VisionAdapter
from drones import Drone
from core.types import Command

from eval.integration.foundation_checks import (
    CheckResult,
    PhaseWindow,
    check_climb_response,
    check_hover_stability,
    check_pitch_attitude_response,
    check_roll_attitude_response,
    check_yaw_response,
    sample_telemetry,
)


class FoundationRunner:
    """
    Phase-1 flight test runner.

    Purpose:
    - validate basic vehicle/adapter/command-path behavior
    - avoid perception/control stack complexity
    - produce repeatable pass/fail checks

    This intentionally does NOT:
    - connect vision
    - start processing modules
    - activate the control module
    """

    def setup(
        self,
        drone: Drone,
        adapter: PlatformAdapter,
        vision_adapter: VisionAdapter,
        pipline: Pipeline,
    ) -> None:
        self.drone = drone
        self.adapter = adapter
        self.vision_adapter = vision_adapter
        self.pipeline = pipline

        self.results: list[CheckResult] = []

    def run(self) -> None:
        print(f"{Fore.MAGENTA}Foundation flight test starting{Style.RESET_ALL}")

        # 1) Connect platform only
        self.drone.adapter.connect()

        # 2) Seed initial hover command
        self.drone.hover(3)

        # 3) Prepare non-module threads
        print(
            f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} "
            f"{Fore.YELLOW}preparing logger and non-module threads{Style.RESET_ALL}"
        )
        self.drone.pipeline.logger_init()
        self.drone.pipeline.pump_sensors_init()
        self.drone.pipeline.send_heartbeat_init()
        self.drone.pipeline.send_command_init()
        self.drone.pipeline.hb_print_init()

        try:
            self.drone.pipeline.logger_start()
            self.drone.pipeline.pump_sensors_start()
            self.drone.pipeline.send_heartbeat_start()
            self.drone.pipeline.send_command_start()
            self.drone.pipeline.hb_print_start()

            print(
                f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} "
                f"{Fore.YELLOW}warmup neutral stream{Style.RESET_ALL}"
            )
            self.drone.hover(2)
            
            print(
                f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} "
                f"{Fore.YELLOW}arming{Style.RESET_ALL}"
            )
            self.drone.arm()
            time.sleep(0.5)

            print(
                f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} "
                f"{Fore.YELLOW}request offboard{Style.RESET_ALL}"
            )
            self.drone.offboard()
            time.sleep(1.0)

            # -------- scripted phases --------
            
            hold_yaw = self.current_yaw_angle()
            print(f"hold_yaw angle {hold_yaw}")

            hover_window = self._run_phase(
                name="hover_stability",
                command=self._cmd(
                    thrust=self.drone.hover_thrust,
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=2.0,
            )
            self.results.append(check_hover_stability(hover_window))

            climb_window = self._run_phase(
                name="climb_response",
                command=self._cmd(
                    thrust=0.810,
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=1.5,
            )
            self.results.append(check_climb_response(climb_window))
        
        
            self._run_phase(
                name="recover_hover_1",
                command=self._cmd(
                    thrust=self.drone.hover_thrust,
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=2.0,
            )

            yaw_cmd = 1 # postive yaw (right turn)
            yaw_window = self._run_phase(
                name="yaw_response",
                command=self._cmd(
                    thrust=self.drone.hover_thrust,
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle= hold_yaw,
                    yaw_rate=yaw_cmd,
                ),
                duration_s=5,
            )
            self.results.append(check_yaw_response(yaw_window, commanded_yaw_rate=yaw_cmd))


            self._run_phase(
                name="recover_hover_2",
                command=self._cmd(
                    thrust=self.drone.hover_thrust,
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=2.0,
            )

            
            pitch_cmd = -0.12 # negative pitch (nose down)
            pitch_window = self._run_phase(
                name="pitch_attitude_response",
                command=self._cmd(
                    thrust=self.drone.hover_thrust + .05, # Tilt-compensated thrust
                    roll=0.0, 
                    pitch=pitch_cmd,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=1.2,
            )
            self.results.append(
                check_pitch_attitude_response(pitch_window, commanded_pitch=pitch_cmd)
            )

            self._run_phase(
                name="recover_hover_3",
                command=self._cmd(
                    thrust=self.drone.hover_thrust,
                    roll=0.0,
                    pitch=0.0,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=2.0,
            )

            roll_cmd = 0.12 # postive roll (right side down)
            roll_window = self._run_phase(
                name="roll_attitude_response",
                command=self._cmd(
                    thrust=self.drone.hover_thrust + .05, # Tilt-compensated thrust
                    roll=roll_cmd,
                    pitch=0.0,
                    yaw_angle=hold_yaw,
                    yaw_rate=0.0,
                ),
                duration_s=1.2,
            )
            self.results.append(
                check_roll_attitude_response(roll_window, commanded_roll=roll_cmd)
            )

            self._print_summary()
            self._write_summary()

        finally:
            # Best-effort return to neutral and stop threads
            self.drone.hover(0.5)

            self.drone.pipeline.stop_evt.set()
            time.sleep(0.3)

            try:
                for _ in range(10):
                    self.drone.hover(0.02)
            except Exception:
                pass

            try:
                self.drone.disarm()
            except Exception:
                pass

    def _cmd(
        self,
        *,
        thrust: float,
        roll: float,
        pitch: float,
        yaw_rate: float,
        yaw_angle: float = 0.0,
    ) -> Command:
        now_ns = time.perf_counter_ns()
        return Command(
            roll=roll,
            pitch=pitch,
            yaw_angle=yaw_angle,
            yaw_rate=yaw_rate,
            thrust=thrust,
            t=now_ns / 1e9,
        )

    def _publish_command(self, cmd: Command) -> None:
        now_ns = time.perf_counter_ns()
        self.pipeline.shared_state.control.pilot_command.publish(cmd, t_ns=now_ns)
        self.pipeline.shared_state.control.command.publish(cmd, t_ns=now_ns)

    def _run_phase(
        self,
        *,
        name: str,
        command: Command,
        duration_s: float,
        republish_hz: float = 10.0,
        sample_hz: float = 25.0,
    ) -> PhaseWindow:
        print(
            f"INFO  {Fore.LIGHTYELLOW_EX}[phase]{Style.RESET_ALL} "
            f"{Fore.YELLOW}{name}{Style.RESET_ALL}"
        )

        samples = []

        start = time.perf_counter()
        next_publish = start
        next_sample = start

        while True:
            now = time.perf_counter()
            if now - start >= duration_s:
                break

            if now >= next_publish:
                self._publish_command(command)
                next_publish += 1.0 / republish_hz

            if now >= next_sample:
                snap = sample_telemetry(self.pipeline.shared_state)
                if snap is not None:
                    samples.append(snap)
                next_sample += 1.0 / sample_hz

            time.sleep(0.002)

        if not samples:
            raise RuntimeError(f"No telemetry samples captured during phase '{name}'")

        return PhaseWindow(name=name, samples=samples)

    def _print_summary(self) -> None:
        print(f"{Fore.MAGENTA}Foundation test summary{Style.RESET_ALL}")
        for result in self.results:
            status = f"{Fore.GREEN}PASS{Style.RESET_ALL}" if result.passed else f"{Fore.RED}FAIL{Style.RESET_ALL}"
            print(f"{status} {result.name}: {result.details}")

    def _write_summary(self) -> None:
        out_dir = Path("eval/foundation_results")
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"foundation_{time.strftime('%Y%m%d_%H%M%S')}.json"

        payload = {
            "results": [asdict(r) for r in self.results],
        }

        with out_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        print(f"INFO  [foundation] wrote summary to {out_path}")
        
    # def hold_hover_for(self, duration_s: float, hz: float = 10.0):
    #     dt = 1.0 / hz
    #     t0 = time.perf_counter()
    #     while time.perf_counter() - t0 < duration_s:
    #         self.drone.hover()
    #         time.sleep(dt)
            
    def current_yaw_angle(self) -> float:
        att, _, _ = self.pipeline.shared_state.sensors.attitude.get()
        if att is None:
            return 0.0
        return att.yaw_rad
    
  

    def tilt_compensated_thrust(hover_thrust: float, roll_rad: float, pitch_rad: float, max_thrust: float = 1.0) -> float:
        denom = math.cos(roll_rad) * math.cos(pitch_rad)

        if denom < 0.5:
            denom = 0.5  # avoid exploding near extreme tilt

        thrust = hover_thrust / denom
        return min(max(thrust, 0.0), max_thrust)