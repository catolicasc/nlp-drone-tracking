"""Mission memory for the Oracle Vision V2 VLM agent."""

from __future__ import annotations

from dataclasses import dataclass, field
import time


@dataclass
class V2Memory:
    task: str = ""
    home: dict[str, float] | None = None
    findings: list[str] = field(default_factory=list)
    actions: list[str] = field(default_factory=list)

    def reset_for_task(self, task: str) -> None:
        self.task = task
        self.findings.clear()
        self.actions.clear()

    def set_home(self, *, x: float, y: float, z: float, yaw: float) -> None:
        if self.home is None:
            self.home = {"x": x, "y": y, "z": z, "yaw": yaw}

    def add_action(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.actions.append(f"[{stamp}] {text}")
        self.actions = self.actions[-30:]

    def add_finding(self, text: str) -> None:
        if text:
            self.findings.append(text)
            self.findings = self.findings[-20:]

    def summary(self) -> str:
        lines = [f"Task: {self.task or '(none)'}"]
        if self.home:
            lines.append(
                "Home: "
                f"x={self.home['x']:.2f}, y={self.home['y']:.2f}, "
                f"z={self.home['z']:.2f}, yaw={self.home['yaw']:.2f}"
            )
        if self.findings:
            lines.append("Findings:")
            lines.extend(f"- {item}" for item in self.findings[-8:])
        if self.actions:
            lines.append("Recent actions:")
            lines.extend(f"- {item}" for item in self.actions[-10:])
        return "\n".join(lines)
