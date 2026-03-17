# 🦅 Albatross - Autonomous Drone Competition System

## Currently Working On

Based on `Albatross/docs/TODO.md`:

- [ ] Split shared state properly for multi-thread access.
- [ ] Add timing alignment and deterministic replay support across modules and logging.
- [ ] Create a Gazebo camera adapter and support many module types beyond only Vision/EKF.
- [ ] Implement replay and offline tooling (run logger, log schema, replay runner, module eval, visualization, policy-vs-heuristic comparisons).
- [ ] Move away from one global lock to per-state objects, ring buffers for high-rate streams, last-value caches, timestamps, and freshness checks.

## What This Repo Is

Albatross is a modular Python autonomy stack for autonomous drone competition workflows.
It is built to run against PX4/Gazebo now, while keeping adapters, drones, runners, and modules swappable.

## Competition Constraints

- Must fly using Neros Technologies drones.
- Must use DCL's AI Vector Module.
- Code must be written in Python.
- Custom drone hardware is not allowed.

## Current Stack (As Implemented)

- Entrypoint: `Albatross/main.py`
- Runtime choices currently wired in `main.py`:
  - Drone: `gz_x500_mono_cam`
  - Adapter: `GazeboPx4`
  - Runner: `Runner`
- Core runtime thread model (`Albatross/core/pipeline.py`):
  - pump sensor thread
  - command loop thread
  - heartbeat print thread
- Shared cross-thread data object: `SharedState` (`Albatross/core/types/telemetry/shared_state.py`)
- Module insertion/removal is done in `main.py` via the `modules = [...]` list.

## Quick Start

1. Start PX4 SITL + Gazebo (from `external/PX4-Autopilot`):

```bash
make px4_sitl gz_x500
```

2. Run Albatross autonomy (from repo root):

```bash
cd Albatross
source .venv-albatross/bin/activate
python main.py gz_x500_mono_cam GazeboPx4 Runner
```

## Repo Layout

- `Albatross/main.py`: entrypoint and runtime wiring.
- `Albatross/core/`: pipeline, control module, types, and utilities.
- `Albatross/adapters/`: platform adapter interface and implementations.
- `Albatross/drones/`: drone interface and drone profiles.
- `Albatross/orchestrators/`: flight orchestration runners.
- `Albatross/docs/`: architecture docs and TODO notes.
- `Albatross/tests/`: fault injection and test scaffolding.
- `external/PX4-Autopilot/`: PX4 submodule.

## Near-Term Goals

- Improve state synchronization/timing discipline.
- Add robust replay/offline evaluation.
- Expand adapters and module coverage.
- Harden the stack for repeatable autonomous runs.

## References

- PX4 Documentation: https://docs.px4.io/main/en/
