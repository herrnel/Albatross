
## `adapters/vision/`
**Purpose:** Camera/image ingestion as a secondary adapter layer.  
**What lives here:** Gazebo camera adapters, ROS camera adapters, replay camera adapters, or real-camera adapters.  
**How it relates to the rest of the code:** This is separate from platform telemetry so image ingestion can run in its own thread and timing domain. It publishes frames and camera metadata into shared topics for perception modules to consume.

