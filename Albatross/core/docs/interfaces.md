## `core/interfaces/`
**Purpose:** Defines the abstract contracts for the main runtime pieces.  
**What lives here:** Base classes/interfaces for `PlatformAdapter`, `VisionAdapter`, `Drone`, and `Runner`.  
**How it relates to the rest of the code:** Concrete implementations in `adapters/`, `drones/`, and `orchestrators/` implement these interfaces so the runtime can swap components without changing pipeline code.