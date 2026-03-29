## `core/types/`
**Purpose:** Defines the typed data contracts passed between adapters, runtime tasks, and algorithms.  
**What lives here:** Command types, telemetry types, camera/frame types, perception result types, estimation result types, observation/action types, and health/status types.  
**How it relates to the rest of the code:** Every major subsystem should communicate through these types instead of raw dicts. This keeps modules testable, replayable, and easy to swap.