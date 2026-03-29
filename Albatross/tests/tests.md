## `tests/`
**Purpose:** Unit and integration tests for system reliability.  
**What lives here:** Tests for topics, adapters, geometry, trackers, safety logic, and runtime plumbing.  
**How it relates to the rest of the code:** `tests/` should validate both the infrastructure in `core/` and the algorithms in `perception/`, `estimation/`, and `policy/`. It is what keeps refactors from breaking the stack.
