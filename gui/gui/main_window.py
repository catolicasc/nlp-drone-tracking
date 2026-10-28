"""Janela principal: monta o cockpit e conecta a ponte ROS aos widgets."""

from __future__ import annotations

import sys

import rclpy
from PySide6.QtCore import QThread
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QVBoxLayout,
    QWidget,
)

from .bridge_node import GuiBridgeNode
from .pipeline_manager import PipelineManager
from .state import GuiSignals
from .widgets.camera_panel import CameraPanel
from .widgets.emergency import EmergencyButton
from .widgets.pipeline_panel import PipelinePanel
from .widgets.task_panel import TaskPanel
from .widgets.telemetry_panel import TelemetryPanel


class SpinThread(QThread):
    def __init__(self, node: GuiBridgeNode) -> None:
        super().__init__()
        self._node = node

    def run(self) -> None:
        try:
            rclpy.spin(self._node)
        except Exception:
            # shutdown durante o fechamento da janela
            pass


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Oracle Vision V2 — Controle do Agente")
        self.resize(1280, 800)

        self.signals = GuiSignals()
        self.bridge = GuiBridgeNode()
        self.bridge.signals = self.signals

        self.manager = PipelineManager()
        self._pipeline = PipelinePanel(self.manager)
        self._telemetry = TelemetryPanel()
        self._camera = CameraPanel()
        self._task = TaskPanel(self._send_task)
        self._emergency = EmergencyButton(self._emergency_land)

        central = QWidget()
        root = QHBoxLayout(central)

        left = QVBoxLayout()
        left.addWidget(self._pipeline)
        left.addWidget(self._telemetry)
        left.addWidget(self._emergency)
        root.addLayout(left, 1)

        right = QVBoxLayout()
        right.addWidget(self._camera, 1)
        right.addWidget(self._task, 1)
        root.addLayout(right, 2)

        self.setCentralWidget(central)

        self.signals.status_event.connect(self._task.on_status_event)
        self.signals.frame.connect(self._camera.on_frame)
        self.signals.pose.connect(self._telemetry.on_pose)
        self.signals.mavros_state.connect(self._telemetry.on_state)
        self.signals.battery.connect(self._telemetry.on_battery)

        self._spin = SpinThread(self.bridge)
        self._spin.start()

    def _send_task(self, text: str) -> None:
        self.bridge.send_task(text)

    def _emergency_land(self) -> None:
        answer = QMessageBox.warning(
            self,
            "Emergência",
            "Pousar o drone imediatamente?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if answer == QMessageBox.Yes:
            self.bridge.send_emergency_land()

    def closeEvent(self, event) -> None:  # noqa: N802 (API Qt)
        self.manager.shutdown()
        rclpy.shutdown()
        self._spin.wait(3000)
        self.bridge.destroy_node()
        super().closeEvent(event)


def main() -> None:
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
