"""Gerencia os processos do pipeline (sim, MAVROS, agente) via QProcess."""

from __future__ import annotations

import os
from pathlib import Path

from PySide6.QtCore import QObject, QProcess, Signal

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"

# nome de exibição -> script
PIPELINE_SCRIPTS: dict[str, Path] = {
    "Sim (Isaac + PX4)": SCRIPTS_DIR / "v2_start_sim.sh",
    "MAVROS": SCRIPTS_DIR / "v2_start_mavros.sh",
    "Agente VLM": SCRIPTS_DIR / "v2_agent.sh",
}


class PipelineManager(QObject):
    log_line = Signal(str, str)  # nome, linha
    state_changed = Signal(str, bool)  # nome, rodando
    failed = Signal(str, str)  # nome, motivo

    def __init__(self) -> None:
        super().__init__()
        self._processes: dict[str, QProcess] = {}

    def is_running(self, name: str) -> bool:
        proc = self._processes.get(name)
        return proc is not None and proc.state() != QProcess.NotRunning

    def start(self, name: str) -> None:
        if self.is_running(name):
            return
        script = PIPELINE_SCRIPTS.get(name)
        if script is None or not script.is_file():
            self.failed.emit(name, f"script não encontrado: {script}")
            return
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.setWorkingDirectory(str(PROJECT_ROOT))
        proc.readyReadStandardOutput.connect(
            lambda n=name, p=proc: self._on_output(n, p)
        )
        proc.finished.connect(
            lambda code, status, n=name: self._on_finished(n, code, status)
        )
        proc.errorOccurred.connect(
            lambda err, n=name: self.failed.emit(n, f"erro QProcess: {err}")
        )
        proc.start("bash", [str(script)])
        self._processes[name] = proc
        self.state_changed.emit(name, True)
        self.log_line.emit(name, f"==> iniciado: {script.name}")

    def stop(self, name: str) -> None:
        proc = self._processes.get(name)
        if proc is None or proc.state() == QProcess.NotRunning:
            return
        # SIGINT/SIGTERM permitem shutdown limpo do ROS; escala para SIGKILL
        pid = int(proc.processId())
        if pid > 0:
            os.kill(pid, 15)
        if not proc.waitForFinished(5000):
            if pid > 0:
                try:
                    os.kill(pid, 9)
                except ProcessLookupError:
                    pass
            proc.kill()
        self.state_changed.emit(name, False)

    def stop_all(self) -> None:
        # para em ordem reversa: agente -> MAVROS -> sim
        for name in reversed(list(PIPELINE_SCRIPTS)):
            self.stop(name)

    def _on_output(self, name: str, proc: QProcess) -> None:
        data = proc.readAllStandardOutput().data().decode("utf-8", errors="replace")
        for line in data.splitlines():
            self.log_line.emit(name, line)

    def _on_finished(self, name: str, code: int, _status) -> None:
        self.state_changed.emit(name, False)
        self.log_line.emit(name, f"==> finalizado (exit {code})")

    def shutdown(self) -> None:
        for name in list(self._processes):
            self.stop(name)
