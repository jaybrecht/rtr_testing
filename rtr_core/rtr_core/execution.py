from abc import ABC, abstractmethod

class StateVar(ABC): ...

class TaskExecutor:
    @property
    def state(self) -> StateVar: ...