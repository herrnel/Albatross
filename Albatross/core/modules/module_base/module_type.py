from __future__ import annotations

import time
import threading
from abc import ABC, abstractmethod
from typing import Optional
from core.types.shared_data.shared_state import SharedState
from core.types.shared_data.base_types.health.module_health_types import ModuleHealth


def sleep_until_ns(target_ns: int) -> None:
    """
    Best-effort fixed-rate sleep using perf_counter_ns().
    """
    while True:
        now_ns = time.perf_counter_ns()
        remaining_ns = target_ns - now_ns
        if remaining_ns <= 0:
            return

        # Sleep only for the coarse part; let the final bit spin lightly.
        if remaining_ns > 2_000_000:  # > 2 ms
            time.sleep((remaining_ns - 1_000_000) / 1_000_000_000.0)
        else:
            # Final short wait
            time.sleep(0)


class ModuleBase(ABC):
    """
    Fixed-rate runtime wrapper base class.

    Responsibilities:
    - own thread creation
    - own local tick counting
    - own fixed-rate timing
    - call subclass tick(now_ns, local_tick)

    Subclasses should:
    - read from typed topics
    - compute outside topic locks
    - publish to typed topics
    """

    def __init__(self, shared_state: SharedState, hz: float, name: str):
        self.shared_state: SharedState = shared_state
        self.hz = hz
        self.name = name

        self._period_ns = int(1e9 / hz) if hz > 0 else 0
        self._local_tick = 0
        self._thread: Optional[threading.Thread] = None

    def setup(self) -> None:
        """
        Optional one-time setup hook.
        """
        pass

    @abstractmethod
    def tick(self, now_ns: int, local_tick: int) -> None:
        """
            This is where a modules implementation exists. The core logic of a module should go here and should call any required implementation libraries here. 
            
            Each module should use at least one module implemeation from the <projroot>/Library directory. 

        Args:
            now_ns (int): Time as of calling this tick 
            
            local_tick (int): Current tick count
        """
        pass

    def create_thread(self, stop_evt: threading.Event) -> threading.Thread:
        """
        `create_thread` is called by `pipeline` to start up all modules (except for the `Control` and `SafetyModule`)

        Args:
            stop_evt (threading.Event): an emergency "break flag" for all module threads when a catastrophic failure has occured

        Returns:
            threading.Thread: a reference to the the running module implemenation 
        """
        self._thread = threading.Thread(
            target=self._run_loop,
            args=(stop_evt,),
            name=self.name,
            daemon=True,
        )
        return self._thread

    def _run_loop(self, stop_evt: threading.Event) -> None:
        """
        is a thin wrapper for `tick` to keep track of the actual time using `time.perf_counter_ns()` and this time information is passed to the module library for it to use. 
        `create_thread` calls `_run_loop` which in turn calls `tick`. This method also calls `setup` if implemented. 
        
        Args:
            stop_evt (threading.Event): _description_
        """
        self.setup()

        next_ns = time.perf_counter_ns()

        while not stop_evt.is_set():
            now_ns = time.perf_counter_ns()
            self._local_tick += 1

            self.tick(now_ns=now_ns, local_tick=self._local_tick)

            next_ns += self._period_ns
            sleep_until_ns(next_ns)
            
            
    def _publish_health(self, status: str, info: str, now_ns: int) -> None:
        """
        This method serves as a modules status report to the rest of the runtime. Its not part of the racing/control algorithm itself rather 
        Its part of the runtime supervision/ observability layer. Each module is doing real-time work in its own loop. This method uses `ModuleHealth`
        to answer questions like "Is this module running at all?", "is it running recently, or has it stalled?", and "is it getting stale inputs?". 
        It lets us know in which module did the data go bad.

        Args:
            status (str): A short state label. Ex. "init", "ok", "stale", "fault"
            info (str): A human-readable explanation. Helpful information field for debugging. ex. "mode=heuristic", "obs_age_ms=15.2, obs_seq=22", or "stale_attitude age_ms=311.0"
            now_ns (int): _description_
        """
        #Create one ModuleHealth object describing the module’s current condition.
        health = ModuleHealth(
            name=self.name,
            hz_target=self.hz,
            last_tick_time_ns=now_ns,
            # age_ms=0.0, This is not useful because we can calculate the age later when logging it. 
            status=status,
            info=info,
        )

        # module_health is a LastValueTopic[dict[str, ModuleHealth]]
        current, _, _ = self.shared_state.health.module_health.get()
        if current is None:
            current = {}

        # Copy dictionary
        updated = dict(current)
        updated[self.name] = health
        self.shared_state.health.module_health.publish(updated, t_ns=now_ns)