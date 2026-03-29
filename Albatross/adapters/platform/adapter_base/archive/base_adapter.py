from abc import ABC, abstractmethod
from Albatross.core.types.archive.data import Telemetry

class BaseAdapter(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def read_telemetry(self) -> Telemetry: ...
