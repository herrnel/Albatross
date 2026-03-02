# Gazebo + Mav Simulator Setup

### Vehicle Model

- Found this version of the normal x500 drone which should be good to work with. [X500 Quadrotor with Monocular Camera](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)

### Communication Protocol

- This seems to be the right link to the protocol we will be using [ARDUPILOT](https://ardupilot.org/copter/docs/ArduCopter_MAVLink_Messages.html#arducopter-mavlink-messages)

- To mimic real life streaming inputs, we should probably use **Stream Groups** as mentioned in the ARDUPILOT documentation. 
- We will read telemetry and output commands using  *mavlink_telemetry_adapter.py*

### Streaming Video from a Mono-Cam

- We can stream video from the drone to our adapter using the [GstCameraSystem](https://docs.px4.io/main/en/sim_gazebo_gz/plugins) plugin which Streams camera feeds via UDP (RTP/H.264) or RTMP with optional NVIDIA CUDA hardware acceleration.
- To read and format this video data we will need an adapter specialized for this task. *gst_camera_adapter.py*

***What won’t match (important expectations)***

Even with this approach, remember:

- This gives you video frames, but not necessarily camera intrinsics or exact timestamps unless you add them.

- The stream may be encoded (H.264) → you’ll decode, which adds latency.
 

# Simulation 

**(Open QGroundControl)**
- It should auto-connect to SITL (often UDP port 14550 / 14540 depending on setup)
- Opening up QGC and having it connected to PX4 usually satifies:

```
WARN  [health_and_arming_checks] Preflight Fail: No connection to the GCS
WARN  [commander] Arming denied: Resolve system health failures first
```



**(Terminal 1)** \
Run the simulation using:

```
make px4_sitl gz_x500_mono_cam
```

**(Terminal 2)** \
Fan-out RTP packets to two local ports using:

- Fanner listens on 5600 and splits into:
- Python listens on 5601
- gst-launch viewer listens on 5602

```
gst-launch-1.0 -v \
  udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
  ! tee name=t \
  t. ! queue ! udpsink host=127.0.0.1 port=5601 sync=false async=false \
  t. ! queue ! udpsink host=127.0.0.1 port=5602 sync=false async=false
```

**(Terminal 3)** \
View the mono-camera feed using:

```
gst-launch-1.0 -v \
  udpsrc port=5602 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```


# Simulation Observations

- “takeoff threshold” for this model + this sim state is around 0.836 (at least at that moment).
- Once the craft starts moving, the required thrust changes slightly (ground effect, controller, battery sim, etc.), so "hover thrust" is not a single universal constant - but 0.836 is the right ballpark for take off. 