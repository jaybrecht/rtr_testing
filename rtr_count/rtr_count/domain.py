from dataclasses import dataclass

from pydantic import BaseModel

from rtr_interfaces.msg import Outcome

from rtr_core.evaluation import Metric, Observation, TaskMeasures
from rtr_core.execution import StateVar


class CountGoalParams(BaseModel):
    target: int


class CountResultParams(BaseModel):
    final: int


class CountFeedbackParams(BaseModel):
    offset: int
    count: int


@dataclass
class CountState(StateVar):
    count: int = 0


@dataclass
class CountObservation(Observation):
    offset: int
    count: int


class CountMeasures(TaskMeasures[CountGoalParams]):
    name = "count"
    GoalParamModel = CountGoalParams

    def __init__(self, goal_params: str):
        super().__init__(goal_params)

    def measure(self, state: CountState) -> CountObservation:
        
        return CountObservation(
            offset=self.goal_params.target - state.count,
            count=state.count
        )


class GreaterThanEqParams(BaseModel):
    pass


class GreaterThanEqMetric(Metric[GreaterThanEqParams]):
    name = "greater_than_eq"
    ParamModel = GreaterThanEqParams

    def evaluate(self, observation: CountObservation) -> int:
        return Outcome.SUCCESS if observation.offset <= 0 else Outcome.INDETERMINATE