# 🦅 Albatross – Autonomous Drone Competition System

Albatross is a modular, high-performance autonomous drone system built for competitive real-world flight. The mission is simple:

> **Build the most stable, fastest-reacting, fully autonomous drone under strict competition constraints.**

This repository documents architecture decisions, environment setup, research direction, and development milestones.

---

# 🎯 Mission Directives

1. **Fully Autonomous**

   * Autonomous takeoff
   * Autonomous landing
   * Autonomous recovery

2. **Adapt to Real-World Conditions**

   * Rain
   * Wind
   * Clouds
   * Dynamic lighting

3. **Performance First**

   * Maximum stability
   * Fastest reaction time
   * High-speed capability (target: 40 m/s)

---

# 📜 Competition Constraints

* Must fly using **Neros Technologies drones**
* Must utilize **DCL’s AI Vector Module**
* Code must be written in **Python**
* Cannot bring your own drone hardware
* FLOSS projects are allowed

---

# 🧠 System Architecture Philosophy

We are building a **modular autonomous system**, not an end-to-end monolith.

Modern autonomous drone systems follow a multi-module architecture:

* Perception
* Localization
* Planning
* Control

Each component:

* Operates semi-independently
* Communicates through well-defined interfaces
* Enables scalability, safety, and debuggability

Inspired by autonomous driving system architecture and ROS-based modular robotics design.

---

# ⚙️ Control Stack Overview

The system is split across **three runtime processes**:

### 🖥 Terminal 1 – PX4 + Gazebo

```
make px4_sitl gz_x500
```

* PX4 Autopilot (C++)
* Physics + sensors via Gazebo
* MAVLink over UDP

### 🖥 Terminal 2 – Python Autonomy Layer

```
source .venv/bin/activate
python main.py
```

* Connects via MAVSDK (UDP)
* Sends offboard commands
* Runs ML models
* Implements autonomy logic

### 🖥 (Optional) Terminal 3 – Simulator tools / debugging

---

# 🏗 Architecture Layers

## Low-Level Control (C++ – Provided)

Expected to be handled by:

* PX4
* DCL AI Vector Module

Responsibilities:

* High-frequency motor control
* PID loops (~1 kHz)
* Real-time stability
* Hardware-level interfacing

Python does **NOT** handle motor loops.

---

## High-Level Autonomy (Python – Our Responsibility)

Responsibilities:

* Perception (CV models)
* State estimation
* Planning
* Decision logic
* Safety handling

Uses:

* PyTorch
* TensorFlow
* scikit-learn
* MAVSDK
* NumPy / Numba (if needed)

---

# 🐍 Python Tradeoffs

### Strengths

* Rapid prototyping
* Massive ML ecosystem
* ROS integration
* Easy to read and maintain

### Weaknesses

* Slower than C++
* Not deterministic at microsecond scale
* Requires optimization strategies

### What We Lose by Not Using C++

* Real-time motor loops
* Direct hardware interfacing
* Millisecond-level performance guarantees

Fortunately, PX4 and the Vector Module handle this.

---

# 🧩 High-Level Development Plan

There is no time to experiment blindly.

Plan:

1. Use an existing flight platform (PX4)
2. Get something working in simulation
3. Modify using modern CV + RL models
4. Build drone-agnostic adapter layer
5. Test in multiple simulators
6. Harden for real-world conditions

---

# 🧪 Simulation Stack

## Primary: PX4 SITL + Gazebo

* Autopilot: PX4 (C++)
* Physics: Gazebo
* Communication: MAVLink (UDP)
* Python connects via MAVSDK

### MAVLink Instances Observed

* High-rate normal instance
* Onboard instance
* Gimbal instance
* Logger instance

Preflight warnings like:

```
Preflight Fail: No connection to the GCS
```

Are expected if QGroundControl is not connected.

---

# 🧠 Future Simulation Targets

* NVIDIA Isaac
* AirSim

Goal: Multi-simulator compatibility via adapter abstraction.

---

# 🗂 Proposed Modular Structure

We are designing for portability from day one.

Core abstractions:

* `Policy`
* `Estimator`
* `DroneAdapter`
* `SafetyManager`

Supports:

* Full mode
* Partial mode
* Degraded mode

Fault injection testing planned.

---

# 📈 Milestones

## Knowledge Gathering

1. Machine Learning foundations
2. Perception & Sensing
3. State Estimation & Planning
4. Math fundamentals
5. Simulation & coding skills

---

## Development

* ✅ Run PX4 SITL + Gazebo
* 🚧 Create Albatross system architecture
* ⏳ Add realism to Gazebo world
* ⏳ Implement MAVSDK adapter
* ⏳ Fly with simple velocity policy
* ⏳ Integrate Isaac & AirSim
* ⏳ Add perception model
* ⏳ Gate detection / waypoint system

---

# 🔬 Research Direction

Inspired by:

* Modular autonomous systems in ADS
* ROS-based node architectures
* Competitive drone racing systems

Areas of focus:

* CV robustness in degraded environments
* Domain randomization
* Motion capture for supervision
* Robust testing pipelines
* Fault-tolerant autonomy

---

# 🛠 macOS Development Notes (Important)

Key lessons learned:

* Separate Homebrew Python (toolchain) from PX4 Python environment
* Do NOT use `--user` for PX4 installs
* Maintain two virtual environments:

  * Toolchain dependencies
  * PX4 Python build environment
* Symlink fixes required for macOS dynamic libraries
* Avoid `make distclean` if symlinks are required
* Use `lsof -i udp:<port>` to debug port conflicts
* Recommended UDP port: 14540 for MAVSDK

---

# 🚀 Target Performance

Benchmark inspiration:

* TU Delft MavLab reached 28 m/s
* Target: **40 m/s**

Speed alone does not win — stability + reaction time does.

---

# 📚 Resources

## Robotics

* Robotics Knowledge Base
  [https://roboticsknowledgebase.com/wiki/robotics-project-guide/choose-a-sim/](https://roboticsknowledgebase.com/wiki/robotics-project-guide/choose-a-sim/)

## PX4

* macOS Development Environment
  [https://docs.px4.io/main/en/dev_setup/dev_env_mac](https://docs.px4.io/main/en/dev_setup/dev_env_mac)

* PX4 Autopilot User Guide
  [https://docs.px4.io/main/en/](https://docs.px4.io/main/en/)

* PX4 Fundamentals Workshop
  [https://www.youtube.com/watch?v=-1MASuJDGn4](https://www.youtube.com/watch?v=-1MASuJDGn4)

* PX4 Discussion Forum
  [https://discuss.px4.io/](https://discuss.px4.io/)

* PX4-Autopilot GitHub
  [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

---

# 🧭 Current Status

* PX4 + Gazebo building successfully
* MAVLink broadcasting correctly
* MAVSDK connection test working
* Architecture refactoring in progress
* System design phase active

---

# 🏁 Final Goal

Build a:

* Fully autonomous
* Real-world robust
* Modular
* Fault-tolerant
* High-speed

competition-winning drone system.

---

If you're reading this:
Welcome to Albatross.
