## `training/control/`
**Purpose:** Training and evaluating learned control policies.  
**What lives here:** Gym/Gymnasium environments, PPO or other RL training scripts, rewards, evaluation scripts, and checkpoint export.  
**How it relates to the rest of the code:** Produces policy models used by runtime control wrappers. It depends on observation definitions from `core/types/` and simulator logic from training code, but should not directly depend on the live runtime loop.
