## `drones/`
**Purpose:** Vehicle-specific configuration and helper behavior.  
**What lives here:** Concrete drone profiles such as thrust tuning, hover/takeoff defaults, and adapter-assisted vehicle actions like arm/offboard/disarm.  
**How it relates to the rest of the code:** `drones/` tells the runtime how a particular drone should behave, but it does not own autonomy logic. It works with adapters and runners during bring-up and flight sequencing.
