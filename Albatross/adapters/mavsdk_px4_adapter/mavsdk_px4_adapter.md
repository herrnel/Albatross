# Gazebo + Mav Simulator Setup

### Vehicle Model

- Found this version of the normal x500 drone which should be good to work with. [X500 Quadrotor with Monocular Camera](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)

### Communication Protocol

- This seems to be the right link to the protocol we will be using [ARDUPILOT](https://ardupilot.org/copter/docs/ArduCopter_MAVLink_Messages.html#arducopter-mavlink-messages)

- To mimic real life streaming inputs, we should probably use **Stream Groups** as mentioned in the ARDUPILOT documentation. 

### Streaming Video from a Mono-Cam

- We can stream video from the drone to our adapter using the [GstCameraSystem](https://docs.px4.io/main/en/sim_gazebo_gz/plugins) plugin which Streams camera feeds via UDP (RTP/H.264) or RTMP with optional NVIDIA CUDA hardware acceleration.

***What won’t match (important expectations)***

Even with this approach, remember:

- This gives you video frames, but not necessarily camera intrinsics or exact timestamps unless you add them.

- The stream may be encoded (H.264) → you’ll decode, which adds latency.
 

# Simulation Commands

**(Terminal 1)** \
Run the simulation using:

```
make px4_sitl gz_x500_mono_cam
```

**(Terminal 2)** \
View the mono-camera feed using:

```
gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```