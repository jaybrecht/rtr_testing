import time
from typing import Generic, cast
from pydantic import ValidationError
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse

from action_msgs.srv import CancelGoal
from rtr_interfaces.action import TaskAttempt
from rtr_interfaces.msg import Metric as MetricMsg, Constraint as ConstraintMsg, Exit, SerializedData, TaskSpecification

from .evaluation import TaskAssessor, Metric, Constraint, TaskMeasures, Assesment
from .execution import TaskExecutor, TStateVar, TGoalParams, TResultParams, TFeedbackParams


class AtomicTask(Node, Generic[TStateVar, TGoalParams, TResultParams, TFeedbackParams]):
    def __init__(
            self,
            task_name: str,
            executor: TaskExecutor[TStateVar, TGoalParams, TResultParams, TFeedbackParams],
            measures_name: str,
            rate_hz: float = 10.0
        ):

        super().__init__(f'{task_name}_server')

        self._action_server = ActionServer(
            node=self,
            action_type=TaskAttempt,
            action_name=f'{task_name}_attempt',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )

        self._executor = executor
        self._measures: TaskMeasures | None = None
        self._assessor: TaskAssessor | None = None
        self._measures_name = measures_name
        self._goal_params: TGoalParams | None = None
        self._completion_policy = TaskSpecification.COMPLETE_ON_EXECUTOR
        self._goal_active = False

        self._period_s = 1.0 / rate_hz

    def _goal_callback(self, goal: TaskAttempt.Goal) -> GoalResponse:
        if self._goal_active:
            return GoalResponse.REJECT

        try:
            specification = goal.specification
            self._goal_params = self._executor.GoalParamModel.model_validate_json(specification.goal_params.data)
            if not self._executor.validate_goal(self._goal_params):
                return GoalResponse.REJECT

            self._measures = TaskMeasures.create(self._measures_name, specification.goal_params.data)

            metrics = cast(list[MetricMsg], specification.metrics)
            constraints = cast(list[ConstraintMsg], specification.constraints)

            self._assessor = TaskAssessor(
                metrics=[Metric.create(m.name, m.metric_params.data) for m in metrics],
                constraints=[Constraint.create(c.name, Metric.create(c.name, c.constraint_params.data), c.active_stages) for c in constraints],
                measures=self._measures
            )
            self._completion_policy = specification.completion_policy
        except (KeyError, ValidationError):
            return GoalResponse.REJECT

        self._goal_active = True
        return GoalResponse.ACCEPT
        
    def _cancel_callback(self, cancel_request: CancelGoal.Request) -> CancelResponse:
        if self._executor.on_cancel():
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def _build_feedback(self, assessment: Assesment) -> TaskAttempt.Feedback:
        feedback_msg = TaskAttempt.Feedback()
        feedback_msg.assessment = assessment.to_msg()

        feedback_params = self._executor.feedback()
        if feedback_params is not None:
            feedback_msg.feedback.encoding = SerializedData.JSON
            feedback_msg.feedback.data = feedback_params.model_dump_json()

        return feedback_msg

    def _build_result(self, assessor: TaskAssessor, canceled: bool, faulted: bool) -> tuple[TaskAttempt.Result, bool]:
        final_state, result_params = self._executor.get_result(canceled)
        completion_assessment = assessor.assess(final_state, ConstraintMsg.STAGE_COMPLETION)
        faulted = faulted or completion_assessment.violated

        result = TaskAttempt.Result()
        result.exit.exit = Exit.FAULTED_EXIT if (canceled or faulted) else Exit.NOMINAL_EXIT
        result.effect.encoding = SerializedData.JSON
        result.effect.data = result_params.model_dump_json()
        result.assessment = completion_assessment.to_msg()
        return result, faulted

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> TaskAttempt.Result:
        assessor = cast(TaskAssessor, self._assessor)

        try:
            initial_state = self._executor.initialize_goal(cast(TGoalParams, self._goal_params))
            faulted = assessor.assess(initial_state, ConstraintMsg.STAGE_INITIALIZATION).violated

            canceled = False
            while True:
                if goal_handle.is_cancel_requested:
                    canceled = True
                    break

                state = self._executor.step()
                assessment = assessor.assess(state, ConstraintMsg.STAGE_EXECUTION)
                goal_handle.publish_feedback(self._build_feedback(assessment))

                if assessment.violated:
                    faulted = True
                    break

                if self._executor.finished or assessment.is_complete(self._completion_policy):
                    break

                time.sleep(self._period_s)

            result, faulted = self._build_result(assessor, canceled, faulted)
        finally:
            self._goal_active = False

        if canceled:
            goal_handle.canceled()
        elif faulted:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        
        return result