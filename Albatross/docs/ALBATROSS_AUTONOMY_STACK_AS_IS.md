# Albatross Autonomy Stack (As-Is Documentation)

This document describes the current code structure and runtime behavior of the Albatross Autonomy Stack exactly as implemented today.

## 1) Where execution starts

Entrypoint: `main.py`

`main.py` is responsible for wiring the runtime pieces together:

1. Parse CLI args for `drone`, `adapter`, and `runner`.
2. Instantiate the selected implementations.
3. Create one shared `SharedState` object.
4. Define the active module list (currently only `ControlModule`).
5. Call `adapter.setup(shared_state)`.
6. Build `Pipeline(modules, adapter, shared_state)`.
7. Build the drone with `drone.setup(adapter, pipeline)`.
8. Build the orchestrator with `runner.setup(drone, adapter, pipeline)`.
9. Execute `runner.run()`.

Current selectable values in `main.py`:

- Drone: `gz_x500_mono_cam`
- Adapter: `GazeboPx4`
- Runner: `Runner`

So the effective run command shape is:

```bash
python main.py gz_x500_mono_cam GazeboPx4 Runner
```

## 2) High-level architecture

The stack is split into three main roles:

- `Drone`: vehicle-level behavior and tuning values (hover/takeoff thrust, arm/offboard/disarm helpers).
- `PlatformAdapter`: all platform I/O (MAVLink connection, telemetry ingest, command transmission).
- `Runner`: flight orchestration sequence (connect, warmup, arm/offboard, module activation, cooldown, disarm).

The `Pipeline` inside `core/` is the runtime thread manager and module coordinator. It owns:

- module threads,
- the pump sensor thread,
- the command streaming thread,
- the heartbeat/position print thread,
- and a shared shutdown event (`stop_evt`).

## 3) Core runtime threads (standard)

As currently implemented, the standard pipeline threads are:

1. Pump sensor thread (`pump_loop`)
- Continuously drains adapter telemetry via `adapter.pump_sensors(max_msgs=500)`.
- Sleeps briefly only when no messages were drained.

2. Command loop thread (`command_loop`)
- Streams the latest command from shared state to the adapter at 50 Hz.
- Calls `adapter.send_attitude_target(...)` each cycle.
- Prints stream heartbeat info roughly once per second.

3. Heartbeat print thread (`hb_print_loop`)
- Prints heartbeat armed/mode status and local position once per second.

Control is treated specially:

- `Pipeline.start_processing()` starts all modules except `control`.
- `Pipeline.take_control()` starts the control loop thread explicitly at 500 Hz.

This matches your design intent: control is activated as a separate phase after initial flight setup.

## 4) Shared data model

Shared state class: `core/types/telemetry/shared_state.py`

`SharedState` is the cross-thread/module data exchange object. It uses a `threading.Lock` and currently contains:

- `latest_command: Command`
- `latest_local_pos: LocalPositionNED | None`
- `latest_attitude: AttitudeData | None`
- `latest_heartbeat`
- `imu_buffer: deque(maxlen=4000)`

All modules and runtime threads read/write through this object, so command generation, telemetry pumping, and status printing share a single synchronized state container.

## 5) Module insertion/removal in `main.py`

`main.py` makes module wiring explicit and easy to edit through the `modules = [...]` list.

Current configuration:

```python
modules = [
    ControlModule(shared_state, hz=500, mode="scripted")
]
```

Commented examples already show intended plug-in style:

- `VisionModule(...)`
- `EkfModule(...)`

This means adding/removing modules is primarily done in one place (`main.py`) and then managed by `Pipeline` automatically.

## 6) Flight orchestration sequence (`Runner`)

`orchestrators/runner.py` currently runs this sequence:

1. Connect adapter (`drone.adapter.connect()`).
2. Seed neutral command (`drone.hover()`).
3. Init pipeline core threads: pump/send/print.
4. Start core threads.
5. Warmup neutral streaming (~2s).
6. Arm.
7. Request offboard.
8. Start non-control modules (`start_processing`).
9. Apply takeoff bump (`bump_and_run`, then wait).
10. Start active control (`take_control`, 500 Hz control loop).
11. Cooldown to neutral hover.
12. Disarm.
13. `finally`: set stop event and send best-effort neutral commands briefly.

## 7) Drone and adapter implementation currently in use

### Drone

`drones/gz_x500_mono_cam.py`

- Implements `Drone` abstract interface.
- Stores thrust tuning:
  - `hover_thrust = 0.74`
  - `takeoff_thrust = 0.87`
- Delegates arm/offboard/disarm to adapter methods.
- Writes hover/takeoff commands into `pipeline.shared_state`.

### Adapter

`adapters/gazebo_px4_adapter/gazebo_px4_sim_adapter.py`

- Implements `PlatformAdapter` for MAVLink/PX4.
- Connects via `pymavlink` endpoint (default `udpin:0.0.0.0:14540`).
- Sends `SET_ATTITUDE_TARGET` in `send_attitude_target`.
- Ingests MAVLink telemetry in `pump_sensors` and updates `SharedState`:
  - `HIGHRES_IMU` -> IMU buffer
  - `LOCAL_POSITION_NED` -> local position
  - `ATTITUDE` -> attitude
  - `HEARTBEAT` -> heartbeat

## 8) File structure (as currently present)

Top-level under project root (`Albatross/`):

- `main.py`: runtime wiring/entrypoint.
- `core/`: pipeline, control module, shared types, utilities.
- `adapters/`: platform abstraction + concrete simulator/transport adapters.
- `drones/`: drone abstraction + concrete drone profile(s).
- `orchestrators/`: runner(s) that orchestrate flight phases.
- `eval/`: placeholder replay/metrics utilities.
- `tests/`: fault-related placeholders and test scaffolding.
- `docs/`: documentation (this file).

Important subpaths:

- `core/pipeline.py`: thread orchestration + module startup logic.
- `core/policy/control_module.py`: current 500 Hz control loop implementation.
- `core/types/telemetry/shared_state.py`: shared, lock-protected runtime state.
- `core/types/command/command_type.py`: command dataclass.
- `core/types/telemetry/telemetry_types.py`: IMU/attitude/frame/position types.
- `adapters/adapter_base/platform_adapter.py`: adapter interface contract.
- `drones/drone_base/drone_base.py`: drone interface contract.
- `orchestrators/runner.py`: current orchestration flow.

## 9) Extensibility points (as-is)

Current design already supports clean swapping in `main.py` for:

- drone profile,
- platform adapter,
- runner,
- module list.

To add a new component, the intended pattern is:

1. Implement the relevant abstract interface (`Drone` or `PlatformAdapter` or runner class).
2. Export it in package `__init__.py` if needed.
3. Add to `main.py` argument choices and `match` selection.
4. Wire it into the module/runner setup path.

This preserves the same startup lifecycle and shared-state threading model.
