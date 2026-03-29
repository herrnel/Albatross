## `perception/`
**Purpose:** Vision and gate-understanding algorithms.  
**What lives here:** Highlight-based gate detection, neural segmentation inference, adaptive cropping, geometry extraction, corner extraction, and related image-processing logic.  
**How it relates to the rest of the code:** Runtime wrappers in `core/runtime/` call into `perception/`. This folder contains the actual algorithms, so they can be reused in live runtime, offline replay, debugging, and unit tests.
