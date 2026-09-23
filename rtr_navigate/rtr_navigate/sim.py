"""A differential drive robot simulated by integrating its own commands.

Subscribes cmd_vel, integrates a unicycle model, and reports the result as
odometry, the odom -> base_link transform, and wheel joint states so RViz can
show the robot moving. There is no physics here: the robot goes exactly where
it is told, which is what makes it useful for exercising a task's control
loop. Entirely independent of rtr_core.
"""

from math import cos, sin
from typing import cast

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation about z."""
    return (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))


class DiffDriveSim(Node):
    def __init__(self) -> None:
        super().__init__("diff_drive_sim")

        self.declare_parameter("rate", 50.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.23)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("command_timeout", 0.5)

        self._wheel_radius = self._float_param("wheel_radius")
        self._wheel_separation = self._float_param("wheel_separation")
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._command_timeout = self._float_param("command_timeout")

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0
        self._omega = 0.0
        self._left_angle = 0.0
        self._right_angle = 0.0
        self._last_command = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self._tf = TransformBroadcaster(self)

        self.create_subscription(TwistStamped, "cmd_vel", self._on_cmd_vel, 10)

        rate = self._float_param("rate")
        self._last_step = self.get_clock().now()
        self.create_timer(1.0 / rate, self._step)

        self.get_logger().info("Diff drive sim ready, waiting for cmd_vel.")

    def _float_param(self, name: str) -> float:
        return cast(float, self.get_parameter(name).value)

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        self._v = msg.twist.linear.x
        self._omega = msg.twist.angular.z
        self._last_command = self.get_clock().now()

    def _step(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_step).nanoseconds / 1e9
        self._last_step = now

        if dt <= 0.0:
            return

        # Stop if commands go stale, so a crashed controller doesn't leave the
        # robot driving forever.
        if (now - self._last_command).nanoseconds / 1e9 > self._command_timeout:
            self._v = 0.0
            self._omega = 0.0

        self._x += self._v * cos(self._theta) * dt
        self._y += self._v * sin(self._theta) * dt
        self._theta += self._omega * dt

        half_track = self._wheel_separation / 2.0
        self._left_angle += (self._v - self._omega * half_track) / self._wheel_radius * dt
        self._right_angle += (self._v + self._omega * half_track) / self._wheel_radius * dt

        self._publish(now)

    def _publish(self, now) -> None:
        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self._theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._v
        odom.twist.twist.angular.z = self._omega
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["left_wheel_joint", "right_wheel_joint"]
        joints.position = [self._left_angle, self._right_angle]
        self._joint_pub.publish(joints)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiffDriveSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
