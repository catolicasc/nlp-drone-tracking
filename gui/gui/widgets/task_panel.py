"""Painel de tarefa: entrada em linguagem natural + timeline de eventos /v2/status."""

from __future__ import annotations

from datetime import datetime
from typing import Callable

from PySide6.QtGui import QColor, QTextCharFormat
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ..status_format import EVENT_COLORS, format_status_event


class TaskPanel(QGroupBox):
    def __init__(self, on_send: Callable[[str], None], parent: QWidget | None = None) -> None:
        super().__init__("Tarefa", parent)
        layout = QVBoxLayout(self)

        row = QHBoxLayout()
        self._input = QLineEdit()
        self._input.setPlaceholderText("Ex.: decole 2 metros e procure pessoas na área")
        self._input.returnPressed.connect(self._send)
        btn_send = QPushButton("Enviar")
        btn_send.clicked.connect(self._send)
        row.addWidget(self._input, 1)
        row.addWidget(btn_send)
        layout.addLayout(row)

        self._timeline = QTextEdit()
        self._timeline.setReadOnly(True)
        layout.addWidget(self._timeline, 1)

        self._hint = QLabel("Sem resposta do agente ainda — verifique se ele está rodando.")
        self._hint.setStyleSheet("color: #888;")
        layout.addWidget(self._hint)

        self._on_send = on_send

    def _send(self) -> None:
        text = self._input.text().strip()
        if text:
            self._on_send(text)
            self._input.clear()

    def on_status_event(self, payload: dict) -> None:
        event = payload.get("event", "")
        color = EVENT_COLORS.get(event, "#333333")
        stamp = datetime.now().strftime("%H:%M:%S")
        line = format_status_event(payload)
        cursor = self._timeline.textCursor()
        cursor.movePosition(cursor.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        if event == "started":
            cursor.insertText("\n")
        cursor.insertText(f"[{stamp}] {line}\n", fmt)
        self._timeline.setTextCursor(cursor)
        self._timeline.ensureCursorVisible()
        self._hint.hide()
