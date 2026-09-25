import rclpy
from typing import cast, Generic, TypeVar

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future 
from pydantic import BaseModel
from collections.abc import Callable

from pydantic import BaseModel, ValidationError

from rtr_interfaces.action import TaskAttempt
from rtr_interfaces.msg import (
    SerializedData,
    Metric,
    Constraint,
    ConstraintEvaluation,
    Exit,
    Assessment,
    Outcome
)

from .execution import TFeedbackParams, TResultParams

ModelT = TypeVar("ModelT", bound=BaseModel)

_OUTCOME_NAMES = {Outcome.SUCCESS: "SUCCESS", Outcome.INDETERMINATE: "INDETERMINATE", Outcome.FAILURE: "FAILURE"}

class TaskClient(Node, Generic[TFeedbackParams, TResultParams]):
    feedback_param_model: type[TFeedbackParams] | None = None
    results_param_model: type[TResultParams]

    def __init__(
        self, 
        action_name: str, 
    ) -> None:
        
        super().__init__("action_client")
        self._client = ActionClient(self, TaskAttempt, action_name)
        
    def build_goal(
        self, 
        goal_params: BaseModel, 
        completion_policy: int, 
        metrics: dict[str, BaseModel], 
        constraints: dict[str, tuple[BaseModel, int]] | None = None
    ) -> TaskAttempt.Goal:
        
        goal = TaskAttempt.Goal()

        specification = goal.specification
        specification.goal_params = self._serialize(goal_params)
        specification.completion_policy = completion_policy
        
        specification.metrics = []
        
        for metric in metrics.keys():
            specification.metrics.append(Metric(
                name=metric,
                metric_params=self._serialize(metrics[metric])
            ))
        
        if constraints is not None:
            for constraint, (params,stages) in constraints.items():
                specification.metrics.append(Constraint(
                    name=constraint,
                    constraint_params=self._serialize(params),
                    active_stages=stages
                ))
                
        return goal
    
    def send(self, goal: TaskAttempt.Goal) -> TResultParams | None:
        self.get_logger().info("Waiting for the task...")
        self._client.wait_for_server()
        
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = cast(ClientGoalHandle, send_future.result())

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return None

        self.get_logger().info("Goal accepted.")
        result_future: Future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        response = result_future.result()
        
        if not response:
            self.get_logger().error("Failed to get result.")
            return None
        
        result = cast(TaskAttempt.Result, response.result)
        
        exit_name = "NOMINAL" if result.exit.exit == Exit.NOMINAL_EXIT else "FAULTED"
        self.get_logger().info(f"Task returned with status: {exit_name}")

        self.get_logger().info(f"Final Assessment: {self.describe(result.assessment)}")

        return self._deserialize(result.effect, self.results_param_model)
    
    def _on_feedback(self, message: TaskAttempt.Impl.FeedbackMessage) -> None:
        if self.feedback_param_model is None:
            return
        
        feedback = cast(SerializedData, message.feedback.feedback)
        
        try:
            params = self._deserialize(feedback, self.feedback_param_model)
        except ValidationError as e:
            self.get_logger().error(str(e))
            return
            
        self.log_feedback(params)
    
    def log_feedback(self, _: TFeedbackParams):
        raise NotImplementedError
    
    def _serialize(self, model: BaseModel) -> SerializedData:
        data = SerializedData()
        data.encoding = SerializedData.JSON
        data.data = model.model_dump_json()
        return data
    
    def _deserialize(self, data: SerializedData, model: type[ModelT]) -> ModelT:
        if data.encoding != SerializedData.JSON:
            raise ValueError(f"unsupported encoding: {data.encoding}")
        return model.model_validate_json(data.data)
    
    def describe(self, assessment: Assessment) -> str:
        outcomes = ", ".join(f"{m.name}={_OUTCOME_NAMES.get(m.outcome, m.outcome)}" for m in assessment.metrics)
        violated = [c.name for c in assessment.constraints if c.status == ConstraintEvaluation.VIOLATED]
        suffix = f", violated: {violated}" if violated else ""
        return f"{outcomes}{suffix}"