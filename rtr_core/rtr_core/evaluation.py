from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Generic, TypeVar

from pydantic import BaseModel
from dataclasses import dataclass

from rtr_interfaces.msg import Assessment as AssessmentMsg, Outcome, ConstraintEvaluation, TaskSpecification

from .execution import StateVar


class Observation(ABC): ...


TMetricParams = TypeVar("TMetricParams", bound=BaseModel)


class Metric(ABC, Generic[TMetricParams]):
    registry: dict[str, type["Metric"]] = {}
    name: str
    ParamModel: type[TMetricParams]
    params: TMetricParams

    def __init_subclass__(cls):
        if not hasattr(cls, "name"):
            raise TypeError(f"{cls.__name__} must define a 'name' attribute")
        if cls.name in Metric.registry:
            raise ValueError(f"Duplicate metric name: {cls.name!r}")
        Metric.registry[cls.name] = cls

        if not hasattr(cls, "ParamModel"):
            raise TypeError(f"{cls.__name__} must define a 'ParamModel' attribute")

    def __init__(self, metric_params: str):
        self.params = self.ParamModel.model_validate_json(metric_params)

    @classmethod
    def create(cls, name:str, data:str) -> "Metric":
        return Metric.registry[name](data)

    @abstractmethod
    def evaluate(self, observation: Observation) -> int: ...


class Constraint(ABC):
    registry: dict[str, type["Constraint"]] = {}
    name: str

    def __init_subclass__(cls):
        if not hasattr(cls, "name"):
            raise TypeError(f"{cls.__name__} must define a 'name' attribute")
        if cls.name in Constraint.registry:
            raise ValueError(f"Duplicate constraint name: {cls.name!r}")
        Constraint.registry[cls.name] = cls

    def __init__(self, evaluation_function: Metric, active_stages: Sequence[int]):
        self._evaluation_function = evaluation_function
        self._active_stages = frozenset(active_stages)

    @classmethod
    def create(cls, name:str, metric: Metric, active_stages: Sequence[int]) -> "Constraint":
        return Constraint.registry[name](metric, active_stages)

    def applies_to(self, stage: int) -> bool:
        return stage in self._active_stages

    @abstractmethod
    def evaluate(self, observation: Observation) -> int: ...


TGoalParams = TypeVar("TGoalParams", bound=BaseModel)


class TaskMeasures(ABC, Generic[TGoalParams]):
    registry: dict[str, type["TaskMeasures"]] = {}
    name: str
    GoalParamModel: type[TGoalParams]
    goal_params: TGoalParams

    def __init_subclass__(cls):
        if not hasattr(cls, "name"):
            raise TypeError(f"{cls.__name__} must define a 'name' attribute")
        if cls.name in TaskMeasures.registry:
            raise ValueError(f"Duplicate task measures name: {cls.name!r}")
        TaskMeasures.registry[cls.name] = cls

        if not hasattr(cls, "GoalParamModel"):
            raise TypeError(f"{cls.__name__} must define a 'GoalParamModel' attribute")

    def __init__(self, goal_params: str):
        self.goal_params = self.GoalParamModel.model_validate_json(goal_params)

    @classmethod
    def create(cls, name:str, goal_params: str) -> "TaskMeasures":
        return TaskMeasures.registry[name](goal_params)

    @abstractmethod
    def measure(self, state: StateVar) -> Observation: ...


@dataclass
class Assesment:
    metric_status: dict[str, int]
    constraint_status: dict[str, int]

    @property
    def violated(self) -> bool:
        return any(status == ConstraintEvaluation.VIOLATED for status in self.constraint_status.values())

    def is_complete(self, policy: int) -> bool:
        if policy == TaskSpecification.COMPLETE_ON_ANY_METRIC:
            return any(outcome == Outcome.SUCCESS for outcome in self.metric_status.values())
        if policy == TaskSpecification.COMPLETE_ON_ALL_METRICS:
            return all(outcome == Outcome.SUCCESS for outcome in self.metric_status.values())
        return False

    def to_msg(self) -> AssessmentMsg:
        msg = AssessmentMsg()
        msg.metrics = [Outcome(name=name, outcome=outcome) for name, outcome in self.metric_status.items()]
        msg.constraints = [ConstraintEvaluation(name=name, status=status) for name, status in self.constraint_status.items()]
        return msg


class TaskAssessor:
    def __init__(self, metrics: list[Metric], constraints: list[Constraint], measures: TaskMeasures):
        self._metrics = metrics
        self._constraints = constraints
        self._measures = measures

    def assess(self, state: StateVar, stage: int) -> Assesment:
        observation = self._measures.measure(state)

        return Assesment(
            metric_status={m.name: m.evaluate(observation) for m in self._metrics},
            constraint_status={c.name: c.evaluate(observation) for c in self._constraints if c.applies_to(stage)}
        )