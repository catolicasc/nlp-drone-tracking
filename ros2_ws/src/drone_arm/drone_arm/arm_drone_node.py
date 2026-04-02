from __future__ import annotations

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool


class ArmDroneNode(Node):
    def __init__(self):
        super().__init__("arm_drone")

        self.declare_parameter("timeout_s", 15.0)
        self.declare_parameter("arm", True)

        self._timeout_s = float(self.get_parameter("timeout_s").value)
        self._arm = bool(self.get_parameter("arm").value)

        self._state: State | None = None

        self._state_sub = self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self._arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        self._start_time = self.get_clock().now()
        self._done = False

        self._timer = self.create_timer(0.1, self._tick)

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _tick(self) -> None:
        if self._done:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self._timeout_s:
            self.get_logger().error("Timeout waiting for MAVROS/PX4")
            self._done = True
            rclpy.shutdown()
            return

        if self._state is None or not self._state.connected:
            return

        if not self._arming_cli.service_is_ready():
            self._arming_cli.wait_for_service(timeout_sec=0.0)
            return

        req = CommandBool.Request()
        req.value = bool(self._arm)

        fut = self._arming_cli.call_async(req)
        fut.add_done_callback(self._on_arm_response)
        self._done = True

    def _on_arm_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Arming call failed: {e}")
            rclpy.shutdown()
            return

        if resp.success:
            self.get_logger().info("ARM command accepted")
        else:
            self.get_logger().error("ARM command rejected")

        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = ArmDroneNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
