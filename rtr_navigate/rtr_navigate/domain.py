"""The navigate task's domain: what it observes, and how it is judged.

Everything a task author defines that is not the executor itself: the goal,
state, result and feedback models, the state->observation measure, the
metrics, and the one constraint.
"""

from dataclasses import dataclass
from math import atan2, cos, hypot, sin

from pydantic import BaseModel, Field

from rtr_interfaces.msg import ConstraintEvaluation, Outcome

from rtr_core.evaluation import Constraint, Metric, Observation, TaskMeasures
from rtr_core.execution import StateVar

def wrap_to_pi(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


class NavigateGoalParams(BaseModel):
    """The goal parameters: where to drive to."""

    target_x: float
    target_y: float
    target_theta: float


class NavigateResultParams(BaseModel):
    """What the attempt left behind, reported in the result."""

    final_x: float
    final_y: float
    final_theta: float


class NavigateFeedbackParams(BaseModel):
    """Optional per-tick feedback, on top of the assessment the executor doesn't see."""

    distance_to_target: float
    heading_error: float
    elapsed: float


@dataclass
class NavigateState(StateVar):
    """Raw state variables, written by the odometry callback."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    stamp: float = 0.0


@dataclass
class NavigateObservation(Observation):
    """What the metrics get to see. One field per measure."""

    distance_to_target: float
    heading_error: float
    speed: float
    omega: float
    elapsed: float
    dt: float


class NavigateMeasures(TaskMeasures[NavigateGoalParams]):
    """Turns raw state into the observation the metrics evaluate against.

    Built per attempt: dt and elapsed depend on the previous tick and the first
    tick's stamp, both tracked here.
    """

    name = "navigate"
    GoalParamModel = NavigateGoalParams

    def __init__(self, goal_params: str):
        super().__init__(goal_params)
        self._start_stamp: float | None = None
        self._prev_stamp: float | None = None

    def measure(self, state: NavigateState) -> NavigateObservation:
        start_stamp = self._start_stamp if self._start_stamp is not None else state.stamp
        prev_stamp = self._prev_stamp if self._prev_stamp is not None else state.stamp

        self._start_stamp = start_stamp
        self._prev_stamp = state.stamp

        return NavigateObservation(
            distance_to_target=hypot(self.goal_params.target_x - state.x, self.goal_params.target_y - state.y),
            heading_error=wrap_to_pi(self.goal_params.target_theta - state.theta),
            speed=hypot(state.vx, state.vy),
            omega=state.omega,
            elapsed=state.stamp - start_stamp,
            dt=state.stamp - prev_stamp,
        )


class WithinRadiusParams(BaseModel):
    radius: float = Field(gt=0)


class WithinRadiusMetric(Metric[WithinRadiusParams]):
    """Satisfied inside a circle around the target. Heading is not considered."""

    name = "within_radius"
    ParamModel = WithinRadiusParams

    def evaluate(self, observation: NavigateObservation) -> int:
        return Outcome.SUCCESS if observation.distance_to_target <= self.params.radius else Outcome.INDETERMINATE


class PoseToleranceParams(BaseModel):
    position_tolerance: float = Field(gt=0)
    heading_tolerance: float = Field(gt=0)


class PoseToleranceMetric(Metric[PoseToleranceParams]):
    """Satisfied when both position and heading are within tolerance."""

    name = "pose_tolerance"
    ParamModel = PoseToleranceParams

    def evaluate(self, observation: NavigateObservation) -> int:
        within_position = observation.distance_to_target <= self.params.position_tolerance
        within_heading = abs(observation.heading_error) <= self.params.heading_tolerance
        return Outcome.SUCCESS if within_position and within_heading else Outcome.INDETERMINATE


class SettledVelocityParams(BaseModel):
    speed_threshold: float = Field(gt=0)
    angular_velocity_threshold: float = Field(gt=0)
    dwell_seconds: float = Field(gt=0)


class SettledVelocityMetric(Metric[SettledVelocityParams]):
    """Satisfied once the robot has been stationary for dwell_seconds.

    Stateful: the accumulator resets on any tick that exceeds a threshold, which
    is why a metric instance belongs to one attempt (TaskAssessor builds a fresh
    one per goal).
    """

    name = "settled_velocity"
    ParamModel = SettledVelocityParams

    def __init__(self, metric_params: str):
        super().__init__(metric_params)
        self._below_for = 0.0

    def evaluate(self, observation: NavigateObservation) -> int:
        below = (
            observation.speed < self.params.speed_threshold
            and abs(observation.omega) < self.params.angular_velocity_threshold
        )
        self._below_for = self._below_for + observation.dt if below else 0.0

        return Outcome.SUCCESS if self._below_for >= self.params.dwell_seconds else Outcome.INDETERMINATE


class TimeLimitParams(BaseModel):
    max_seconds: float = Field(gt=0)


class TimeLimitMetric(Metric[TimeLimitParams]):
    """The metric a time_limit constraint wraps: FAILURE past the deadline."""

    name = "time_limit"
    ParamModel = TimeLimitParams

    def evaluate(self, observation: NavigateObservation) -> int:
        return Outcome.FAILURE if observation.elapsed > self.params.max_seconds else Outcome.SUCCESS


class TimeLimitConstraint(Constraint):
    """The attempt must finish within max_seconds."""

    name = "time_limit"

    def evaluate(self, observation: Observation) -> int:
        outcome = self._evaluation_function.evaluate(observation)
        return ConstraintEvaluation.VIOLATED if outcome == Outcome.FAILURE else ConstraintEvaluation.UPHELD
