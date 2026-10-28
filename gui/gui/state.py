"""Sinais Qt compartilhados entre a ponte ROS e os widgets da GUI."""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal


class GuiSignals(QObject):
    # /v2/status: dict JSON completo do evento
    status_event = Signal(dict)
    # MAVROS: connected, armed, mode
    mavros_state = Signal(bool, bool, str)
    # pose local em metros (ENU)
    pose = Signal(float, float, float)
    # bateria em porcentagem (None se indisponível)
    battery = Signal(object)
    # frame de câmera como array RGB uint8 (h, w, 3)
    frame = Signal(object)
    # log de saída dos processos do pipeline: nome do processo, linha
    process_output = Signal(str, str)
    # estado de um processo do pipeline: nome, rodando
    process_state = Signal(str, bool)
