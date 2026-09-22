from typing import cast
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse

from action_msgs.srv import CancelGoal
from rtr_interfaces.action import TaskAttempt
from rtr_interfaces.msg import Metric as MetricMsg, Constraint as ConstraintMsg

from .evaluation import TaskAssessor, Metric, Constraint, TaskMeasures
from .execution import TaskExecutor


class AtomicTask(Node):
    def __init__(
            self, 
            task_name: str, 
            executor: TaskExecutor,
            measures_name: str
        ):

        super().__init__(f'{task_name}_atomic_task')

        self._action_server = ActionServer(
            node=self,
            action_type=TaskAttempt,
            action_name=f'{task_name}_server',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback
        )

        self._executor = executor
        self._measures: TaskMeasures | None = None
        self._assessor: TaskAssessor | None = None
        self._measures_name = measures_name

    def _goal_callback(self, goal: TaskAttempt.Goal) -> GoalResponse:
        self._measures = TaskMeasures.create(self._measures_name, goal.goal_params.data)

        metrics = cast(list[MetricMsg], goal.metrics)
        constraints = cast(list[ConstraintMsg], goal.constraints)

        self._assessor = TaskAssessor(
            metrics=[Metric.create(m.name, m.metric_params.data) for m in metrics],
            constraints=[Constraint.create(c.name, Metric.create(c.name, c.constraint_params.data), c.active_stages) for c in constraints],
            measures=self._measures
        )

        return GoalResponse.ACCEPT
        
    def _handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        goal_handle.execute()

    def _cancel_callback(self, cancel_request: CancelGoal.Request) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> TaskAttempt.Result:
        if self._assessor is None:
            return TaskAttempt.Result()
        
        while True:
            state = self._executor.state

            self._assessor.assess(state, ConstraintMsg.STAGE_EXECUTION)

        return TaskAttempt.Result()