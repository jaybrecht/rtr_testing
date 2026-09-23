from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Generic, TypeVar

from pydantic import BaseModel
from rclpy.node import Node

@dataclass
class StateVar(ABC): ...

TStateVar = TypeVar("TStateVar", bound=StateVar)
TGoalParams = TypeVar("TGoalParams", bound=BaseModel)
TResultParams = TypeVar("TResultParams", bound=BaseModel)
TFeedbackParams = TypeVar("TFeedbackParams", bound=BaseModel)


class TaskExecutor(Node, ABC, Generic[TStateVar, TGoalParams, TResultParams, TFeedbackParams]):
    GoalParamModel: type[TGoalParams]
    ResultParamModel: type[TResultParams]
    FeedbackParamModel: type[TFeedbackParams] | None = None

    def __init_subclass__(cls):
        if not hasattr(cls, "GoalParamModel"):
            raise TypeError(f"{cls.__name__} must define a 'GoalParamModel' attribute")
        if not hasattr(cls, "ResultParamModel"):
            raise TypeError(f"{cls.__name__} must define a 'ResultParamModel' attribute")

    def __init__(self, node_name: str):
        Node.__init__(self, node_name)
        self._finished = False

    @property
    def finished(self) -> bool:
        return self._finished

    @abstractmethod
    def validate_goal(self, goal_params: TGoalParams) -> bool: ...

    @abstractmethod
    def step(self) -> TStateVar: ...

    @abstractmethod
    def initialize_goal(self, goal_params: TGoalParams) -> TStateVar: ...

    @abstractmethod
    def get_result(self, canceled: bool) -> tuple[TStateVar, TResultParams]: ...

    def feedback(self) -> TFeedbackParams | None: return None

    def on_cancel(self) -> bool: return True

    