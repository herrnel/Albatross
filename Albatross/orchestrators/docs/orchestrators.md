## `orchestrators/`
**Purpose:** High-level execution flow for starting, running, and stopping the system.  
**What lives here:** Live runners, replay runners, qualification runners, or evaluation runners.  
**How it relates to the rest of the code:** The orchestrator decides lifecycle order: connect adapters, warm up, arm, enter offboard mode, start runtime tasks, activate control, cool down, and shut down safely.
