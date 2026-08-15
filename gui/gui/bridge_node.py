"""Nó rclpy que faz a ponte entre ROS 2 e a GUI Qt.

Publica tarefas em /v2/task e assina /v2/status, /mavros/state,
/mavros/local_position/pose, /mavros/battery e /drone/camera/image_raw,
repassando tudo à thread Qt via sinais.
"""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String

from .frame_convert import image_msg_to_rgb


class GuiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("v2_gui_bridge")
        self._task_pub = self.create_publisher(String, "/v2/task", 10)

        # latched-compatible: eventos de status são publicados esporadicamente
        self.create_subscription(String, "/v2/status", self._on_status, 10)
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_pose, 10
        )
        self.create_subscription(BatteryState, "/mavros/battery", self._on_battery, 10)
        # câmera usa best-effort para não travar o spin com frames atrasados
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, "/drone/camera/image_raw", self._on_image, cam_qos
        )

    # ---- comandos (chamados da thread Qt) ----

    def send_task(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._task_pub.publish(msg)

    def send_emergency_land(self) -> None:
        self.send_task("land")

    # ---- callbacks ROS (rodam na thread de spin) ----

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"event": "raw", "data": msg.data}
        # o sinais são conectados pela MainWindow (GuiSignals)
        self.signals.status_event.emit(payload)

    def _on_state(self, msg: State) -> None:
        self.signals.mavros_state.emit(msg.connected, msg.armed, msg.mode)

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.signals.pose.emit(p.x, p.y, p.z)

    def _on_battery(self, msg: BatteryState) -> None:
        pct = None
        if msg.percentage >= 0.0 and msg.voltage > 0.0:
            pct = msg.percentage * 100.0
        self.signals.battery.emit(pct)

    def _on_image(self, msg: Image) -> None:
        rgb = image_msg_to_rgb(msg)
        if rgb is not None:
            self.signals.frame.emit(rgb)

    # injetado pela MainWindow antes do spin
    signals: Any = None


def spin_bridge(node: GuiBridgeNode) -> None:
    rclpy.spin(node)
