## `training/`
**Purpose:** Offline training code for learned parts of the stack.  
**What lives here:** Vision training scripts, control/RL training scripts, dataset loaders, augmentations, reward functions, evaluation scripts, and export utilities.  
**How it relates to the rest of the code:** `training/` produces model checkpoints used later by runtime code in `perception/` or `policy/`, but it should stay separate from the live runtime so experimentation does not pollute the control stack.
