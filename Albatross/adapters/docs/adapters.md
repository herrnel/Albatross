## `adapters/`
**Purpose:** Handles external I/O with simulators, flight stacks, sensors, or replay sources.  
**What lives here:** Platform-specific and vision-specific adapters.  
**How it relates to the rest of the code:** Adapters are the only layer that should directly talk to PX4, Gazebo, AirSim, Issac, ROS topics, camera streams, replay logs, etc. They publish sensor data into shared topics and send commands outward.
