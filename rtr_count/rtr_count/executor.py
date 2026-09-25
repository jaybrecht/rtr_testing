import rclpy
from rclpy.executors import MultiThreadedExecutor

from rtr_core.atomic_task import AtomicTask
from rtr_core.execution import TaskExecutor

from rtr_count.domain import (
    CountFeedbackParams,
    CountGoalParams,
    CountResultParams,
    CountState,
)

class CountExecutor(TaskExecutor[CountState, CountGoalParams, CountResultParams, CountFeedbackParams]):
    GoalParamModel = CountGoalParams
    ResultParamModel = CountResultParams
    FeedbackParamModel = CountFeedbackParams

    def __init__(self, node_name: str = "count_executor"):
        super().__init__(node_name)

        self._state = CountState()
        self._goal: CountGoalParams | None = None

    def validate_goal(self, goal_params: CountGoalParams) -> bool:
        if goal_params.target <= 1:
            self.get_logger().error("Target must be greater than 0.")
            return False
        return True

    def initialize_goal(self, goal_params: CountGoalParams) -> CountState:
        self._goal = goal_params
        self._state.count = 0
        return self._state

    def step(self) -> CountState:
        goal = self._goal
        assert goal is not None

        self._state.count += 1
        return self._state

    def get_result(self, canceled: bool) -> tuple[CountState, CountResultParams]:
        result = CountResultParams(final=self._state.count)
        return self._state, result

    def feedback(self) -> CountFeedbackParams | None:
        if self._goal is None:
            return None
        return CountFeedbackParams(
            offset=self._goal.target - self._state.count,
            count=self._state.count
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    executor_node = CountExecutor()
    task = AtomicTask("count", executor_node, measures_name="count", rate_hz=1)

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
