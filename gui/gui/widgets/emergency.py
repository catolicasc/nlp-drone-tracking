"""Botão de emergência: publica 'land' em /v2/task (o agente trata interrupt_land)."""

from __future__ import annotations

from typing import Callable

from PySide6.QtWidgets import QPushButton


class EmergencyButton(QPushButton):
    def __init__(self, on_land: Callable[[], None]) -> None:
        super().__init__("EMERGÊNCIA\nPOUSAR")
        self.setMinimumHeight(72)
        self.setStyleSheet(
            "QPushButton { background: #c0392b; color: white; "
            "font-size: 20px; font-weight: bold; border-radius: 8px; }"
            "QPushButton:hover { background: #e74c3c; }"
        )
        self.clicked.connect(lambda: on_land())
