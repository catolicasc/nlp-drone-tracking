"""Painel de telemetria: pose local, estado MAVROS e bateria."""

from __future__ import annotations

from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QLabel,
    QWidget,
)


class TelemetryPanel(QGroupBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Telemetria", parent)
        form = QFormLayout(self)
        self._labels = {
            "pos": QLabel("aguardando dados"),
            "alt": QLabel("aguardando dados"),
            "mode": QLabel("—"),
            "armed": QLabel("—"),
            "connected": QLabel("desconectado"),
            "battery": QLabel("—"),
        }
        form.addRow("Posição (x, y) [m]", self._labels["pos"])
        form.addRow("Altitude [m]", self._labels["alt"])
        form.addRow("Modo", self._labels["mode"])
        form.addRow("Armed", self._labels["armed"])
        form.addRow("MAVROS", self._labels["connected"])
        form.addRow("Bateria", self._labels["battery"])

    def on_pose(self, x: float, y: float, z: float) -> None:
        self._labels["pos"].setText(f"({x:.1f}, {y:.1f})")
        self._labels["alt"].setText(f"{z:.1f}")

    def on_state(self, connected: bool, armed: bool, mode: str) -> None:
        self._labels["connected"].setText("conectado" if connected else "desconectado")
        self._labels["armed"].setText("SIM" if armed else "não")
        self._labels["mode"].setText(mode or "—")

    def on_battery(self, pct: float | None) -> None:
        if pct is None:
            self._labels["battery"].setText("—")
            return
        self._labels["battery"].setText(f"{pct:.0f}%")
        color = "#e74c3c" if pct < 20 else ("#f39c12" if pct < 40 else "#27ae60")
        self._labels["battery"].setStyleSheet(f"color: {color}; font-weight: bold;")
