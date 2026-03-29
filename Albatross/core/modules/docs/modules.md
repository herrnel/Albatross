## `core/runtime/` *(or `core/modules/`)*
**Purpose:** Runtime wrappers that the pipeline schedules as threads or timed tasks.  
**What lives here:** `gate_detection_runtime.py`, `state_estimation_runtime.py`, `control_runtime.py`, `safety_runtime.py`, etc.  
**How it relates to the rest of the code:** These files are the bridge between infrastructure and algorithms. They pull inputs from shared topics, call functions/classes from `perception/`, `estimation/`, or `policy/`, then publish outputs back to shared state.