## `core/`
**Purpose:** Runtime infrastructure and shared system plumbing.  
**What lives here:** The pipeline, topic/state abstractions, timing helpers, health tracking, base interfaces, and runtime task wrappers.  
**How it relates to the rest of the code:** `core/` does **not** contain the autonomy algorithms themselves. Instead, it schedules and connects them. Files in `core/runtime/`