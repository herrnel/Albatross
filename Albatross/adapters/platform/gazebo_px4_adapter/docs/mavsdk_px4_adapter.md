# Gazebo + Mav Simulator Setup
## My Setup

#### Vehicle Model

- Found this version of the normal x500 drone with a mono-camera which should be good to work with. [X500 Quadrotor with Monocular Camera](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)

#### Communication Protocol

- Mavlink is the protocol used between our python code and PX4 which is the controller and model in the simulation. 
- Messages are defined by xml files and are also known as *dialects* 
- Px4 is speaking `Mavlink version 2` we see this when we run `mavlink status` 
- The dialect is determined by the `pymavlink` which I believe is `pymavlink/dialects/v20/ardupilotmega.py` that means our `pymavlink` install is currently using the `ardupilotmega` dialect module as the generated Python definitions for messages/fields. That does **not** mean our vehicle is ArduPilot. It just means pymavlink is using that generated module as the message definitions. That generated module contains the common.xml and another xml as an extension for Ardupilot. We only really care about the common.xml messages. 
- [PX4 telemetry basics]([https://docs.px4.io/main/en/middleware/mavlink.html](https://docs.px4.io/main/en/middleware/mavlink.html)) & [Offboard control examples (Python)]([https://docs.px4.io/main/en/advanced/offboard.html](https://docs.px4.io/main/en/advanced/offboard.html)) These cover exactly what we’re doing with `SET_POSITION_TARGET_LOCAL_NED` and `SET_ATTITUDE_TARGET`.
- To mimic real life streaming inputs, we should probably use **Stream Groups**. (This did not end up happening)
- We will read telemetry and output commands using  *mavlink_telemetry_adapter.py*



#### Streaming Video from a Mono-Cam

- We can stream video from the drone to our adapter using the [GstCameraSystem](https://docs.px4.io/main/en/sim_gazebo_gz/plugins) plugin which Streams camera feeds via UDP (RTP/H.264) or RTMP with optional NVIDIA CUDA hardware acceleration.
- To read and format this video data we will need an adapter specialized for this task. *gst_camera_adapter.py*

***What won’t match for the competition (important expectations)***

Even with this approach:

- This gives you video frames, but not necessarily camera intrinsics or exact timestamps unless you add them.

- The stream may be encoded (H.264) → you’ll decode, which adds latency.
 
## Simulation Quick Start

#### **(Open QGroundControl)**
- It should auto-connect to SITL (often UDP port 14550 / 14540 depending on setup)
- Opening up QGC and having it connected to PX4 usually satifies:

```
WARN  [health_and_arming_checks] Preflight Fail: No connection to the GCS
WARN  [commander] Arming denied: Resolve system health failures first
```

#### **(Terminal 1)** Start Running PX4

If you have ran make previously run (Optional):

```bash
make distclean
```

Sometimes closing Px4 and Gazbeo will leave some background processes running so the following series of commands will help find and kill them. 

```bash
ps aux | grep -E "gz sim|gzserver|ign gazebo" | grep -v grep
kill -9 <pid> <pid>
```

Change directories to the PX4-Autopilot directory. Run the standard simulation using:

*Note:* take a look at **"Simulations Environments"** Section to learn how to run in different environments.

```bash
make px4_sitl gz_x500_mono_cam
```

#### **(Terminal 2)** Fan out image stream

Fan-out RTP packets to two local ports using:

- Fanner listens on 5600 and splits into:
- Python listens on 5601
- gst-launch viewer listens on 5602

```bash
gst-launch-1.0 -v \
  udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
  ! tee name=t \
  t. ! queue ! udpsink host=127.0.0.1 port=5601 sync=false async=false \
  t. ! queue ! udpsink host=127.0.0.1 port=5602 sync=false async=false
```

#### **(Terminal 3)** Launch onboard camera POV

View the mono-camera feed using:

```bash
gst-launch-1.0 -v \
  udpsrc port=5602 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

#### **(Terminal 4)** Run Albatross
Run the Albatross autonomy stack:

use the .venv-albatross by running the following:

```bash
source /Users/danyherrera/Projects/AI-Grand-Prix/.venv314/bin/activate
```

Now run the stack using:

```bash
python3 main.py gz_x500_mono_cam GazeboPx4 Runner
```

#### **(Example View)**

![Example View](Screenshot%202026-03-09%20at%208.46.40 PM.png) 

## Simulation Environments

I have found [pedrogasg/drone-gate](github.com/pedrogasg/drone-gate) public project that has some worlds and objects available for us to use.

The biggest challenge I am facing right now is determining if the simulation is set up properly. I have not easy way to test this.

---

### Custom Worlds: the Illini Warehouse setup (2026-05)

This section documents how the project actually launches PX4 SITL inside a
non-default world (currently `x3_illini_warehouse`), what every moving
part does, and why the submodule layout looks the way it does. It's
written so future-you (or a new teammate) can read it once and not have
to rediscover any of it.

#### TL;DR — the recipe

```bash
cd ~/Projects/AI-Grand-Prix
source scripts/env.sh                                   # once per shell
cd external/PX4-Autopilot
PX4_GZ_WORLD=x3_illini_warehouse make px4_sitl gz_x500_mono_cam
```

Add `HEADLESS=1` in front to skip the Gazebo GUI. Drop `PX4_GZ_WORLD=...`
to use the default empty world.

#### What that command actually does (chain of events)

```
PX4_GZ_WORLD=x3_illini_warehouse  make px4_sitl gz_x500_mono_cam
        │                              │
        │                              └─► CMake-generated target.
        │                                  Sets PX4_SIM_MODEL=gz_x500_mono_cam
        │                                  (= which drone to SPAWN)
        │
        └─► Env override: "use this WORLD file"
            Resolved to <PX4>/Tools/simulation/gz/worlds/x3_illini_warehouse.sdf
```

After `make` builds `px4`, the script `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`
runs and does, in order:

1. Pick the right `gz sim` version (we force major `8` because Homebrew
   installs gz-10 alongside gz-8 on macOS — see the macOS fixes commit
   in the PX4 fork).
2. **Launch Gazebo** with the world SDF:
   `gz sim --force-version 8 -r -s <PX4>/Tools/simulation/gz/worlds/x3_illini_warehouse.sdf`.
3. Wait until Gazebo reports the world is ready.
4. **Spawn the drone** by sending Gazebo a `create entity` RPC pointing
   at `<PX4>/Tools/simulation/gz/models/x500_mono_cam/model.sdf`.
   PX4 names this entity `x500_mono_cam_0` (the `_0` is the instance
   number).
5. Start `gz_bridge` — the translator between PX4 and Gazebo:
   - sends motor commands **out** to the spawned drone, and
   - reads IMU / GPS / camera **in** from it.
6. Move the Gazebo GUI camera to follow `x500_mono_cam_0`.

If you see these lines in the log you know each step worked:

```
INFO  [init] Gazebo simulator 8.10.0 (forced major 8)
INFO  [init] gazebo already running world: x3_illini_warehouse
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: x3_illini_warehouse, model: x500_mono_cam_0
INFO  [init] Setting camera to follow x500_mono_cam_0
INFO  [px4] Startup script returned successfully
```

#### The three things that must all exist

| # | Question | Where to look |
|---|---|---|
| A | Does the **world** file exist? | `external/PX4-Autopilot/Tools/simulation/gz/worlds/x3_illini_warehouse.sdf` |
| B | Does the **model** for the drone exist? | `external/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/model.sdf` |
| C | Can Gazebo resolve `model://` includes inside the world (X3, gates, warehouse, drums, …)? | `GZ_SIM_RESOURCE_PATH` must include the `mav_simulator` model dirs — `scripts/env.sh` sets that. |

If any of A/B/C is missing, the chain breaks. The error messages are
usually clear ("failed to load world", "could not find model", etc.).

#### Two drones in the scene — only one is "yours"

The warehouse world ships with a passive **X3** decoration drone
(`<include><uri>model://X3</uri></include>` inside the world SDF). PX4
spawns the **x500_mono_cam** separately on top of that. So the scene
contains:

- **X3** — scenery. Lives in the world file. PX4 doesn't know it
  exists. Has motor plugins but nothing drives them.
- **x500_mono_cam_0** — the actual flown drone. Spawned by PX4 at
  startup. Talks to `gz_bridge`. This is the one the camera follows
  and the one your Albatross stack controls.

If the extra X3 ever gets in the way, edit
`Tools/simulation/gz/worlds/x3_illini_warehouse.sdf` in the
`PX4-gazebo-models` fork and delete the `<include>` block for X3.

#### Common preflight warnings (and what they mean)

| Warning | Meaning | Action |
|---|---|---|
| `barometer 0 missing` | x500_mono_cam SDF has no baro sensor and `SENS_EN_BAROSIM` isn't on | `param set SENS_EN_BAROSIM 1` in `pxh>`, then `param save`, then restart |
| `Found 0 compass (required: 1)` | Same: no mag sensor in SDF, `SENS_EN_MAGSIM` off | `param set SENS_EN_MAGSIM 1`, save, restart |
| `ekf2 missing data` | State estimator hasn't seen enough samples yet | Wait a few seconds; goes away |
| `No connection to the GCS` | No QGroundControl running | Start QGC (only matters if flying via QGC) |
| `vehicle_command_ack lost` | Mavlink generation gap during startup | Harmless |

#### How the submodule layout supports all this

The project pins three external repos as **forks** under `external/`, so
a fresh clone is reproducible and a `git reset --hard` can never lose
local customizations again. All three follow the identical pattern.

| Submodule path | Upstream (`origin`) | Our fork (`myfork`) | Branch | Why we forked |
|---|---|---|---|---|
| `external/PX4-Autopilot` | `PX4/PX4-Autopilot` | `herrnel/PX4-Autopilot` | `ai-grand-prix` | Carries macOS/Homebrew build fixes (Qt5 lookup, `--force-version 8`, DYLD paths, `-Wno-double-promotion`, etc.) |
| `external/PX4-Autopilot/Tools/simulation/gz` (nested) | `PX4/PX4-gazebo-models` | `herrnel/PX4-gazebo-models` | `ai-grand-prix` | Holds the `x3_illini_warehouse.sdf` world that PX4's CMake globs |
| `external/mav_simulator` | `UIUC-Robotics/mav_simulator` | `herrnel/mav_simulator` | `ai-grand-prix` | Pins the X3 model + warehouse / gate / drum / forklift / generator decoration models referenced by the world |

Clone the whole project with:

```bash
git clone --recurse-submodules https://github.com/herrnel/Albatross.git
```

Each fork has an `AGENTS.md` at its root that documents the rules for
editing it (never edit `main`, never push to `origin`, always push to
`myfork`, branch is `ai-grand-prix`). AI agents working in those
directories read that file before making changes.

#### Helper scripts (in `scripts/`)

- **`scripts/env.sh`** — `source` this once per shell before SITL. Sets
  `GZ_SIM_RESOURCE_PATH` so Gazebo can find the X3 + warehouse + gate
  models that live in `mav_simulator`. Also auto-activates
  `external/PX4-Autopilot/.venv-px4` if it exists.
- **`scripts/bump-submodule.sh <path> "<msg>"`** — after you edit files
  inside a submodule, this commits + pushes the submodule branch and
  stages the pointer bump in the parent repo. Saves a 5-step dance.
- **`scripts/sync-upstream.sh <path> [upstream-branch]`** — periodically
  rebase a fork's `ai-grand-prix` branch onto its upstream `main` and
  force-push (with-lease) to `myfork`. Use when upstream PX4 adds a
  feature we want.

Daily workflow when changing a submodule (e.g. editing the warehouse
world in the gazebo-models fork):

```bash
# 1. Edit files inside the submodule on its ai-grand-prix branch.
cd external/PX4-Autopilot/Tools/simulation/gz
git branch --show-current   # must be 'ai-grand-prix'
# ...edit worlds/x3_illini_warehouse.sdf...

# 2. From the project root, run the helper:
cd ~/Projects/AI-Grand-Prix
scripts/bump-submodule.sh external/PX4-Autopilot/Tools/simulation/gz \
  "Tweak warehouse lighting"
# That commits + pushes the inner submodule, then commits the pointer
# bump in the next-level-up repo (PX4-Autopilot fork). For nested
# submodules you may need to also bump the next layer:
scripts/bump-submodule.sh external/PX4-Autopilot \
  "Bump gazebo-models pointer: Tweak warehouse lighting"

# 3. Push the parent repo when you're happy:
git log -1 && git push
```

#### Why `PX4_GZ_WORLD` is an env var, not a make target argument

PX4's CMake globs `Tools/simulation/gz/worlds/*.sdf` at configure time
and generates a make target `gz_<model>_<world>` for every combination.
So there's *also* a target `gz_x500_mono_cam_x3_illini_warehouse` that
bakes the world choice into the target name. Using `PX4_GZ_WORLD=<name>`
as an env override lets you reuse the default `gz_x500_mono_cam` target
but swap the world — handy when iterating on the world without a full
reconfigure.

#### Adding a brand-new world later

1. Add the SDF to `external/PX4-Autopilot/Tools/simulation/gz/worlds/`
   (i.e. in the PX4-gazebo-models fork). The filename (minus `.sdf`)
   becomes the world name.
2. If it references new `model://` URIs, drop the models into
   `external/mav_simulator/mav_gazebo/models/` (or the description repo)
   so `GZ_SIM_RESOURCE_PATH` can resolve them. No env.sh changes needed
   if you reuse existing roots.
3. Re-run cmake (`make px4_sitl` once with no specific target) so the
   new world is picked up by the glob and a `gz_<model>_<world>` target
   is generated.
4. Commit + push via `scripts/bump-submodule.sh` (twice — gazebo-models
   first, then PX4-Autopilot).

