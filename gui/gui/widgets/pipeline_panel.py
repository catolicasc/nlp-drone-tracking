"""Painel de controle do pipeline: botões iniciar/parar por processo + log."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)

from ..pipeline_manager import PIPELINE_SCRIPTS, PipelineManager

STATUS_COLORS = {
    "parado": "#888888",
    "rodando": "#2ecc71",
    "falhou": "#e74c3c",
}


class PipelinePanel(QGroupBox):
    def __init__(self, manager: PipelineManager, parent: QWidget | None = None) -> None:
        super().__init__("Pipeline", parent)
        self._manager = manager
        self._status_labels: dict[str, QLabel] = {}
        self._buttons: dict[str, QPushButton] = {}
        self._failed: set[str] = set()

        layout = QVBoxLayout(self)
        for name in PIPELINE_SCRIPTS:
            row = QHBoxLayout()
            status = QLabel("●")
            status.setFixedWidth(20)
            status.setAlignment(Qt.AlignCenter)
            self._status_labels[name] = status
            row.addWidget(status)
            row.addWidget(QLabel(name), 1)

            btn_start = QPushButton("Iniciar")
            btn_start.clicked.connect(lambda _=False, n=name: manager.start(n))
            btn_stop = QPushButton("Parar")
            btn_stop.setEnabled(False)
            btn_stop.clicked.connect(lambda _=False, n=name: manager.stop(n))
            self._buttons[name] = (btn_start, btn_stop)
            row.addWidget(btn_start)
            row.addWidget(btn_stop)
            layout.addLayout(row)

        btn_stop_all = QPushButton("Parar tudo")
        btn_stop_all.clicked.connect(manager.stop_all)
        layout.addWidget(btn_stop_all)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(2000)
        self._log.setMinimumHeight(140)
        layout.addWidget(self._log, 1)

        manager.state_changed.connect(self._on_state)
        manager.log_line.connect(self._on_log)
        manager.failed.connect(self._on_failed)
        self._refresh_states()

    def _on_state(self, name: str, running: bool) -> None:
        if running:
            self._failed.discard(name)
        btn_start, btn_stop = self._buttons[name]
        btn_start.setEnabled(not running)
        btn_stop.setEnabled(running)
        self._refresh_states()

    def _on_failed(self, name: str, reason: str) -> None:
        self._failed.add(name)
        self._log.appendPlainText(f"[{name}] FALHA: {reason}")
        self._refresh_states()

    def _on_log(self, name: str, line: str) -> None:
        self._log.appendPlainText(f"[{name}] {line}")

    def _refresh_states(self) -> None:
        for name, label in self._status_labels.items():
            if name in self._failed:
                color = STATUS_COLORS["falhou"]
                tip = "falhou"
            elif self._manager.is_running(name):
                color = STATUS_COLORS["rodando"]
                tip = "rodando"
            else:
                color = STATUS_COLORS["parado"]
                tip = "parado"
            label.setStyleSheet(f"color: {color}; font-size: 16px;")
            label.setToolTip(tip)
