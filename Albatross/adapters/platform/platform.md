## `adapters/platform/`
**Purpose:** Platform I/O for telemetry and command transmission.  
**What lives here:** PX4/MAVLink/Gazebo adapters that connect, pump telemetry, and send commands.  
**How it relates to the rest of the code:** This is the drone/simulator control side. It feeds IMU, attitude, heartbeat, local position, and other platform telemetry into the runtime, and receives final commands from control/safety.
