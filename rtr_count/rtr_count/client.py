import argparse
import sys

import rclpy

from rclpy.utilities import remove_ros_args

from rtr_core.client import TaskClient

from rtr_interfaces.msg import TaskSpecification

from rtr_count.domain import (
    CountFeedbackParams,
    CountGoalParams,
    CountResultParams,
    GreaterThanEqParams
)

class CountClient(TaskClient[CountFeedbackParams, CountResultParams]):
    feedback_param_model = CountFeedbackParams
    results_param_model = CountResultParams

    def log_feedback(self, feedback: CountFeedbackParams):
        print(f"Current count: {feedback.count}")

 
def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a count task attempt.")
    parser.add_argument("target", type=int, help="counting target")

    return parser.parse_args(argv)


def main(args=None) -> None:
    rclpy.init(args=args)
    parsed = parse_args(remove_ros_args(sys.argv)[1:])

    client = CountClient('count_attempt')
    
    try:
        specification = client.build_goal(
            CountGoalParams(target=parsed.target),
            TaskSpecification.COMPLETE_ON_ALL_METRICS,
            {'greater_than_eq': GreaterThanEqParams()}
        )
        
        effect = client.send(specification)
        
        if effect is not None:
            client.get_logger().info(f'Final count: {effect.final}')
        
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
