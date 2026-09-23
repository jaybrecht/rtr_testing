"""Send a navigate task attempt and report what the evaluator says about it.

Builds a TaskAttempt goal out of the target pose (given on the command line)
and the three metrics and a time-limit constraint, whose parameters come from
ROS params (see config/navigate_metrics.yaml) rather than the CLI. Prints the
assessment as feedback arrives and once more when the attempt ends.
"""

import argparse
import sys
from typing import cast

import rclpy
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from rtr_interfaces.action import TaskAttempt
from rtr_interfaces.msg import (
    Assessment,
    Constraint,
    ConstraintEvaluation,
    Exit,
    Metric,
    Outcome,
    SerializedData,
    TaskSpecification,
)

from rtr_navigate.domain import (
    NavigateFeedbackParams,
    NavigateGoalParams,
    PoseToleranceParams,
    SettledVelocityParams,
    TimeLimitParams,
    WithinRadiusParams,
)

_OUTCOME_NAMES = {Outcome.SUCCESS: "SUCCESS", Outcome.INDETERMINATE: "INDETERMINATE", Outcome.FAILURE: "FAILURE"}


def _serialize(model: BaseModel) -> SerializedData:
    data = SerializedData()
    data.encoding = SerializedData.JSON
    data.data = model.model_dump_json()
    return data


def describe(assessment: Assessment) -> str:
    outcomes = ", ".join(f"{m.name}={_OUTCOME_NAMES.get(m.outcome, m.outcome)}" for m in assessment.metrics)
    violated = [c.name for c in assessment.constraints if c.status == ConstraintEvaluation.VIOLATED]
    suffix = f", violated: {violated}" if violated else ""
    return f"{outcomes}{suffix}"


class NavigateClient(Node):
    """The metric and constraint parameters come from ROS params, not the CLI.

    Load them from a YAML file (see config/navigate_metrics.yaml):

        ros2 run rtr_navigate client 2.0 1.0 --ros-args --params-file <path>
    """

    def __init__(self, action_name: str = "navigate_server") -> None:
        super().__init__("navigate_client")
        self._client = ActionClient(self, TaskAttempt, action_name)

        self.declare_parameter("within_radius.radius", 0.25)
        self.declare_parameter("pose_tolerance.position_tolerance", 0.1)
        self.declare_parameter("pose_tolerance.heading_tolerance", 0.1)
        self.declare_parameter("settled_velocity.speed_threshold", 0.02)
        self.declare_parameter("settled_velocity.angular_velocity_threshold", 0.05)
        self.declare_parameter("settled_velocity.dwell_seconds", 0.5)
        self.declare_parameter("time_limit.max_seconds", 60.0)

    def _param(self, name: str) -> float:
        return cast(float, self.get_parameter(name).value)

    def build_goal(self, x: float, y: float, theta: float) -> TaskAttempt.Goal:
        """Pack the target, the three metrics and the deadline into one goal."""
        goal = TaskAttempt.Goal()
        specification = goal.specification
        specification.goal_params = _serialize(NavigateGoalParams(target_x=x, target_y=y, target_theta=theta))
        specification.completion_policy = TaskSpecification.COMPLETE_ON_ALL_METRICS

        specification.metrics = [
            Metric(
                name="within_radius",
                metric_params=_serialize(WithinRadiusParams(radius=self._param("within_radius.radius"))),
            ),
            Metric(
                name="pose_tolerance",
                metric_params=_serialize(
                    PoseToleranceParams(
                        position_tolerance=self._param("pose_tolerance.position_tolerance"),
                        heading_tolerance=self._param("pose_tolerance.heading_tolerance"),
                    )
                ),
            ),
            Metric(
                name="settled_velocity",
                metric_params=_serialize(
                    SettledVelocityParams(
                        speed_threshold=self._param("settled_velocity.speed_threshold"),
                        angular_velocity_threshold=self._param("settled_velocity.angular_velocity_threshold"),
                        dwell_seconds=self._param("settled_velocity.dwell_seconds"),
                    )
                ),
            ),
        ]

        specification.constraints = [
            Constraint(
                name="time_limit",
                constraint_params=_serialize(TimeLimitParams(max_seconds=self._param("time_limit.max_seconds"))),
                active_stages=[Constraint.STAGE_EXECUTION],
            )
        ]

        return goal

    def send(self, goal: TaskAttempt.Goal) -> int:
        """Send the goal and block until the attempt ends. Returns an exit code."""
        self.get_logger().info("Waiting for the navigate task...")
        self._client.wait_for_server()

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = cast(ClientGoalHandle, send_future.result())

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return 1

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        exit_name = "NOMINAL" if result.exit.exit == Exit.NOMINAL_EXIT else "FAULTED"
        self.get_logger().info(f"Result: {describe(result.assessment)}")
        self.get_logger().info(f"Exit: {exit_name}, effect: {result.effect.data}")

        return 0 if result.exit.exit == Exit.NOMINAL_EXIT else 1

    def _on_feedback(self, message) -> None:
        feedback = message.feedback
        distance = "?"
        if feedback.feedback.data:
            params = NavigateFeedbackParams.model_validate_json(feedback.feedback.data)
            distance = f"{params.distance_to_target:.3f}"
        self.get_logger().info(f"d={distance}  {describe(feedback.assessment)}", throttle_duration_sec=0.5)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a navigate task attempt.")
    parser.add_argument("x", type=float, help="target x in the odom frame")
    parser.add_argument("y", type=float, help="target y in the odom frame")
    parser.add_argument("theta", type=float, nargs="?", default=0.0, help="target heading in radians")

    return parser.parse_args(argv)


def main(args=None) -> None:
    rclpy.init(args=args)
    parsed = parse_args(remove_ros_args(sys.argv)[1:])

    client = NavigateClient()

    try:
        code = client.send(client.build_goal(parsed.x, parsed.y, parsed.theta))
    except KeyboardInterrupt:
        code = 1
    finally:
        client.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == "__main__":
    main()
