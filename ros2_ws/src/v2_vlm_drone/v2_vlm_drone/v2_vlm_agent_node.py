"""Main ROS 2 node for the Oracle Vision V2 VLM-only agent."""

from __future__ import annotations

import json
import threading
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .memory import V2Memory
from .ros_tools import V2RosTools
from .tool_defs import SYSTEM_PROMPT, tool_schemas
from .vlm_client import VlmClient, VlmClientError


class V2VlmAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("v2_vlm_agent")

        self.declare_parameter("task_topic", "/v2/task")
        self.declare_parameter("status_topic", "/v2/status")
        self.declare_parameter("max_steps", 25)
        self.declare_parameter("min_alt_m", 0.5)
        self.declare_parameter("max_alt_m", 10.0)
        self.declare_parameter("max_radius_m", 15.0)
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("pre_setpoints_sec", 5.0)

        try:
            self._vlm = VlmClient.from_env()
        except VlmClientError as exc:
            self.get_logger().error(str(exc))
            raise

        self._memory = V2Memory()
        self._status_pub = self.create_publisher(
            String,
            self.get_parameter("status_topic").get_parameter_value().string_value,
            10,
        )
        self._tools = V2RosTools(
            self,
            self._memory,
            self._vlm,
            min_alt_m=self.get_parameter("min_alt_m").value,
            max_alt_m=self.get_parameter("max_alt_m").value,
            max_radius_m=self.get_parameter("max_radius_m").value,
            rate_hz=self.get_parameter("setpoint_rate_hz").value,
            pre_setpoints_sec=self.get_parameter("pre_setpoints_sec").value,
        )

        self._lock = threading.Lock()
        self._busy = False
        self._thread: threading.Thread | None = None
        self.create_subscription(
            String,
            self.get_parameter("task_topic").get_parameter_value().string_value,
            self._on_task,
            10,
        )
        self._publish_status({"event": "ready", "model": self._vlm.model, "api_base": self._vlm.api_base})
        self.get_logger().info("V2 VLM agent aguardando tarefas em /v2/task")

    def _publish_status(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)
        self.get_logger().info(msg.data)

    @staticmethod
    def _is_emergency_land(text: str) -> bool:
        low = text.lower()
        return any(word in low for word in ("land", "pouse", "pousar", "pare", "stop", "emergência", "emergencia"))

    def _on_task(self, msg: String) -> None:
        task = msg.data.strip()
        if not task:
            return

        with self._lock:
            if self._busy:
                if self._is_emergency_land(task):
                    self._publish_status({"event": "interrupt_land", "task": task})
                    self._tools.land()
                    return
                self._publish_status({"event": "rejected_busy", "task": task})
                return
            self._busy = True

        self._thread = threading.Thread(target=self._run_task, args=(task,), daemon=True)
        self._thread.start()

    def _run_task(self, task: str) -> None:
        self._tools.reset_task()
        self._memory.reset_for_task(task)
        self._publish_status({"event": "started", "task": task})

        messages: list[dict[str, Any]] = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {
                "role": "user",
                "content": (
                    f"Tarefa do operador: {task}\n\n"
                    f"Memória atual:\n{self._memory.summary()}\n\n"
                    "Planeje e execute usando somente as tools disponíveis."
                ),
            },
        ]

        try:
            max_steps = int(self.get_parameter("max_steps").value)
            for step in range(1, max_steps + 1):
                self._publish_status({"event": "vlm_step", "step": step, "max_steps": max_steps})
                response = self._vlm.chat(messages, tools=tool_schemas())
                assistant = self._vlm.assistant_message(response)
                messages.append(assistant)

                tool_calls = assistant.get("tool_calls") or []
                if not tool_calls:
                    content = assistant.get("content") or ""
                    self._publish_status(
                        {
                            "event": "no_tool_call",
                            "content": content,
                            "hint": "O VLM deve finalizar usando complete_task.",
                        }
                    )
                    break

                for call in tool_calls:
                    function = call.get("function") or {}
                    name = str(function.get("name", ""))
                    raw_args = function.get("arguments") or "{}"
                    try:
                        args = json.loads(raw_args) if isinstance(raw_args, str) else dict(raw_args)
                    except json.JSONDecodeError as exc:
                        result = json.dumps({"ok": False, "error": f"JSON inválido: {exc}"}, ensure_ascii=False)
                    else:
                        result = self._tools.execute(name, args)
                    self._publish_status({"event": "tool_result", "tool": name, "result": result})
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": call.get("id", name),
                            "name": name,
                            "content": result,
                        }
                    )
                    if self._tools.task_done:
                        self._publish_status({"event": "finished", **(self._tools.task_result or {})})
                        return

            if not self._tools.task_done:
                self._publish_status(
                    {
                        "event": "finished",
                        "success": False,
                        "summary": "Missão terminou sem complete_task do VLM.",
                    }
                )
        except Exception as exc:  # noqa: BLE001
            self._publish_status({"event": "error", "success": False, "error": str(exc)})
        finally:
            with self._lock:
                self._busy = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = V2VlmAgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
