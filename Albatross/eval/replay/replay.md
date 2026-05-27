## `replay/`
**Purpose:** Offline re-execution of recorded runs.  
**What lives here:** Replay loaders, replay runners, frame indexing tools, and utilities to feed logged data back through modules.  
**How it relates to the rest of the code:** Replay lets you test perception, estimation, and policy changes without rerunning the simulator or drone every time. It is one of the most important tools for iterating safely and quickly.
