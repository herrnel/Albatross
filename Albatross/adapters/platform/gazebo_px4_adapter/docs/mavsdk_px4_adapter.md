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
