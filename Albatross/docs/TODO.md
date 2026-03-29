# TODO

All of this is in reference to our **5. What our architecture is missing** section of our Albatross V2 Design Spec.

- [ ] Timing alignment must be integral to sharedstate and must be handled by modules, logging, and deterministic replay.
- [ ] Split up Shared State Properly for multiple threads.
- [ ] Create a Camera Adapter for Gazebo. Also avoid the idea of having soley a Vision and Ekf Module. In fact, expect there will be many different modules, some engineering and other trained.
- [ ] Implement Replay and offline tooling. This is not optional. I need a run logger, a log Schema, a replay runner, module-level offline eval, perception visualization estimator plots and policy-vs-heuristic comparisions.
- [ ] We need to move away from one global lock but rather per-state objects, rung bufferes for high-rate streams, last-value caches for low-rate outputs explicit timestamps and freshness checks.
