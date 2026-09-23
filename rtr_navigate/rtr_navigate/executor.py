"""The navigate task's execution role.

State comes from odometry, commands go out on cmd_vel, and step() closes the
loop once per control cycle. The controller drives to the target position
first, then turns in place to the target heading; whether that counts as
accomplishing the goal is the goal's completion_policy and metrics, not this
node's concern.
"""

from math import atan2, cos, hypot, sin
from typing import cast

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor

from rtr_core.atomic_task import AtomicTask
from rtr_core.execution import TaskExecutor

from rtr_navigate.domain import (
    NavigateFeedbackParams,
    NavigateGoalParams,
    NavigateResultParams,
    NavigateState,
    wrap_to_pi,
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class NavigateExecutor(TaskExecutor[NavigateState, NavigateGoalParams, NavigateResultParams, NavigateFeedbackParams]):
    GoalParamModel = NavigateGoalParams
    ResultParamModel = NavigateResultParams
    FeedbackParamModel = NavigateFeedbackParams

    def __init__(self, node_name: str = "navigate_executor"):
        super().__init__(node_name)

        self.declare_parameter("max_linear_velocity", 0.6)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("linear_gain", 1.2)
        self.declare_parameter("angular_gain", 2.5)
        self.declare_parameter("heading_gate", 0.6)
        self.declare_parameter("approach_tolerance", 0.05)

        self._state = NavigateState()
        self._goal: NavigateGoalParams | None = None
        self._start_stamp: float | None = None
        self._odom_received = False

        self._cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)

        self.get_logger().info("Ready. Publishing cmd_vel, waiting for odom.")

    def _on_odom(self, msg: Odometry) -> None:
        """Write the raw state variables. Nothing derived is computed here."""
        orientation = msg.pose.pose.orientation

        self._state.x = msg.pose.pose.position.x
        self._state.y = msg.pose.pose.position.y
        self._state.theta = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

        # Odometry reports body-frame velocity; the state holds it in the odom
        # frame so that speed is a plain magnitude over both components.
        forward = msg.twist.twist.linear.x
        self._state.vx = forward * cos(self._state.theta)
        self._state.vy = forward * sin(self._state.theta)
        self._state.omega = msg.twist.twist.angular.z

        self._state.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._odom_received = True

    def validate_goal(self, goal_params: NavigateGoalParams) -> bool:
        if not self._odom_received:
            self.get_logger().error("No odometry received yet.")
            return False
        return True

    def initialize_goal(self, goal_params: NavigateGoalParams) -> NavigateState:
        self._goal = goal_params
        self._start_stamp = self._state.stamp
        return self._state

    def _param(self, name: str) -> float:
        return cast(float, self.get_parameter(name).value)

    def step(self) -> NavigateState:
        """One control cycle: read the state, command a velocity."""
        goal = self._goal
        assert goal is not None

        dx = goal.target_x - self._state.x
        dy = goal.target_y - self._state.y
        distance = hypot(dx, dy)

        approach_tolerance = self._param("approach_tolerance")
        max_linear = self._param("max_linear_velocity")
        max_angular = self._param("max_angular_velocity")
        linear_gain = self._param("linear_gain")
        angular_gain = self._param("angular_gain")
        heading_gate = self._param("heading_gate")

        if distance > approach_tolerance:
            heading_error = wrap_to_pi(atan2(dy, dx) - self._state.theta)
            angular = angular_gain * heading_error
            # Drive only once roughly pointed at the target, so the robot does
            # not arc away from it while turning.
            linear = linear_gain * distance if abs(heading_error) < heading_gate else 0.0
        else:
            # Arrived: turn in place to the requested heading.
            angular = angular_gain * wrap_to_pi(goal.target_theta - self._state.theta)
            linear = 0.0

        self._publish_command(_clamp(linear, max_linear), _clamp(angular, max_angular))
        return self._state

    def get_result(self, canceled: bool) -> tuple[NavigateState, NavigateResultParams]:
        self._publish_command(0.0, 0.0)
        result = NavigateResultParams(final_x=self._state.x, final_y=self._state.y, final_theta=self._state.theta)
        return self._state, result

    def feedback(self) -> NavigateFeedbackParams | None:
        if self._goal is None or self._start_stamp is None:
            return None
        return NavigateFeedbackParams(
            distance_to_target=hypot(self._goal.target_x - self._state.x, self._goal.target_y - self._state.y),
            heading_error=wrap_to_pi(self._goal.target_theta - self._state.theta),
            elapsed=self._state.stamp - self._start_stamp,
        )

    def _publish_command(self, linear: float, angular: float) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        command.twist.linear.x = linear
        command.twist.angular.z = angular
        self._cmd_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)

    executor_node = NavigateExecutor()
    task = AtomicTask("navigate", executor_node, measures_name="navigate")

    # MultiThreadedExecutor, not SingleThreadedExecutor: AtomicTask's execute
    # callback blocks (time.sleep) between control cycles for the life of an
    # attempt, and executor_node's odom subscription needs to keep being
    # serviced on another thread while that's happening.
    spinner = MultiThreadedExecutor()
    spinner.add_node(task)
    spinner.add_node(executor_node)

    try:
        spinner.spin()
    except KeyboardInterrupt:
        pass
    finally:
        spinner.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
