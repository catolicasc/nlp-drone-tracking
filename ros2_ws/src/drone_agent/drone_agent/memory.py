"""Memória textual do agente (estilo ARNA)."""

from __future__ import annotations

from dataclasses import dataclass, field
import time


@dataclass
class AgentMemory:
    task: str = ""
    findings: list[str] = field(default_factory=list)
    action_log: list[str] = field(default_factory=list)
    home_pose: dict[str, float] | None = None

    def log_action(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.action_log.append(f"[{stamp}] {text}")

    def add_finding(self, text: str) -> None:
        if text and (not self.findings or self.findings[-1] != text):
            self.findings.append(text)

    def set_home(self, x: float, y: float, z: float, yaw: float) -> None:
        self.home_pose = {"x": x, "y": y, "z": z, "yaw": yaw}

    def summary(self, max_actions: int = 12) -> str:
        lines = [f"Task: {self.task}"]
        if self.home_pose:
            hp = self.home_pose
            lines.append(
                f"Home: x={hp['x']:.2f} y={hp['y']:.2f} z={hp['z']:.2f} yaw={hp['yaw']:.2f}"
            )
        if self.findings:
            lines.append("Findings:")
            for f in self.findings[-8:]:
                lines.append(f"  - {f}")
        if self.action_log:
            lines.append("Recent actions:")
            for a in self.action_log[-max_actions:]:
                lines.append(f"  - {a}")
        return "\n".join(lines)
