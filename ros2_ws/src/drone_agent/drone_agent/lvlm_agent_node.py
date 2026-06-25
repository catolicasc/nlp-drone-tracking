"""Agente LVLM estilo ARNA para Oracle Vision."""

from __future__ import annotations

import json
import re
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .llm_client import LlmClient, LlmClientError
from .memory import AgentMemory
from .ros_tools import RosToolBridge
from .tool_defs import SYSTEM_PROMPT, tool_schemas


class HeuristicPlanner:
    """Fallback sem API quando heuristic_mode=true."""

    def plan(self, task: str, status: dict[str, Any]) -> list[dict[str, Any]]:
        t = task.lower()
        steps: list[dict[str, Any]] = [{"name": "get_drone_status", "arguments": {}}]
        planned_action = False

        takeoff_words = (
            "decol",
            "takeoff",
            "subir",
            "suba",
            "sobe",
            "ergu",
            "levantar",
            "levante",
            "voar",
            "voe",
            "sobrevo",
        )
        if any(w in t for w in takeoff_words):
            z = self._extract_altitude_m(t, default=2.0)
            steps.append({"name": "takeoff", "arguments": {"z": z}})
            planned_action = True
        if any(w in t for w in ("procur", "search", "encontr", "pessoa", "person")):
            steps.append({"name": "get_detections", "arguments": {}})
            steps.append({"name": "hover", "arguments": {"duration_sec": 3.0}})
            z = self._extract_altitude_m(t, default=2.0)
            steps.append({"name": "goto_position", "arguments": {"x": 2.0, "y": 0.0, "z": z}})
            steps.append({"name": "get_detections", "arguments": {}})
            planned_action = True
        if any(w in t for w in ("pous", "land")):
            steps.append({"name": "land", "arguments": {}})
            planned_action = True
        if any(w in t for w in ("descer", "abaixar", "desce")):
            z = self._extract_altitude_m(t, default=1.0)
            pose = status.get("pose") or {}
            cx = float(pose.get("x", 0.0))
            cy = float(pose.get("y", 0.0))
            steps.append({"name": "goto_position", "arguments": {"x": cx, "y": cy, "z": z}})
            planned_action = True
        if any(w in t for w in ("frente", "forward", "avanç")):
            dist = self._extract_distance_m(t, default=2.0)
            steps.append({"name": "goto_relative", "arguments": {"dx": dist, "dy": 0.0, "dz": 0.0}})
            planned_action = True
        if any(w in t for w in ("trás", "back", "voltar para trás")):
            dist = self._extract_distance_m(t, default=2.0)
            steps.append({"name": "goto_relative", "arguments": {"dx": -dist, "dy": 0.0, "dz": 0.0}})
            planned_action = True
        if any(w in t for w in ("esquerda", "left")):
            deg = self._extract_degrees(t, default=30.0)
            steps.append({"name": "rotate_yaw", "arguments": {"delta_deg": deg}})
            planned_action = True
        if any(w in t for w in ("direita", "right")):
            deg = self._extract_degrees(t, default=30.0)
            steps.append({"name": "rotate_yaw", "arguments": {"delta_deg": -deg}})
            planned_action = True
        if any(w in t for w in ("parar", "stop", "freeze", "fica parado")):
            steps.append({"name": "hover", "arguments": {"duration_sec": 3.0}})
            planned_action = True
        if any(w in t for w in ("início", "home", "retornar ao", "return home", "volte ao")):
            steps.append({"name": "return_home", "arguments": {}})
            planned_action = True
        if not planned_action:
            steps.append(
                {
                    "name": "complete_task",
                    "arguments": {
                        "summary": (
                            "Não entendi uma ação segura no modo heurístico. "
                            "Comandos disponíveis: decolar, pousar, subir, descer, "
                            "ir para frente, ir para trás, virar à esquerda, "
                            "virar à direita, parar, retornar ao início."
                        ),
                        "success": False,
                    },
                }
            )
            return steps
        steps.append(
            {
                "name": "complete_task",
                "arguments": {"summary": f"Heurística executada: {task}", "success": True},
            }
        )
        if status.get("connected") is False:
            steps = [
                {
                    "name": "complete_task",
                    "arguments": {
                        "summary": "MAVROS não conectado; suba run_mavros_px4.sh",
                        "success": False,
                    },
                }
            ]
        return steps

    @staticmethod
    def _extract_altitude_m(text: str, default: float) -> float:
        text = text.replace(",", ".")

        cm_match = re.search(r"(\d+(?:\.\d+)?)\s*(cm|centimetro|centímetros|centimetros)", text)
        if cm_match:
            return max(0.2, float(cm_match.group(1)) / 100.0)

        m_match = re.search(r"(\d+(?:\.\d+)?)\s*(m|metro|metros)\b", text)
        if m_match:
            return max(0.2, float(m_match.group(1)))

        bare_match = re.search(r"\b(\d+(?:\.\d+)?)\b", text)
        if bare_match:
            value = float(bare_match.group(1))
            if value > 10.0 and ("solo" in text or "baixo" in text):
                return max(0.2, value / 100.0)
            return max(0.2, value)

        return default

    @staticmethod
    def _extract_distance_m(text: str, default: float) -> float:
        text = text.replace(",", ".")
        m_match = re.search(r"(\d+(?:\.\d+)?)\s*(m|metro|metros)\b", text)
        if m_match:
            return max(0.5, float(m_match.group(1)))
        bare_match = re.search(r"\b(\d+(?:\.\d+)?)\b", text)
        if bare_match:
            return max(0.5, float(bare_match.group(1)))
        return default

    @staticmethod
    def _extract_degrees(text: str, default: float) -> float:
        text = text.replace(",", ".")
        deg_match = re.search(r"(\d+(?:\.\d+)?)\s*(graus|°|deg)", text)
        if deg_match:
            return max(5.0, float(deg_match.group(1)))
        bare_match = re.search(r"\b(\d+(?:\.\d+)?)\b", text)
        if bare_match:
            val = float(bare_match.group(1))
            if val <= 360.0:
                return max(5.0, val)
        return default


class LvlmAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("lvlm_agent")

        self.declare_parameter("task", "")
        self.declare_parameter("task_topic", "/drone_agent/task")
        self.declare_parameter("max_steps", 25)
        self.declare_parameter("heuristic_mode", False)
        self.declare_parameter("use_vision", True)
        self.declare_parameter("llm_api_base", "")
        self.declare_parameter("llm_model", "")

        self.declare_parameter("setpoint_topic", "/mavros/setpoint_position/local")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("detections_topic", "/person_detector/detections")
        self.declare_parameter("image_topic", "/drone/camera/image_raw")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("search_radius_max", 10.0)
        self.declare_parameter("pre_setpoints_sec", 2.0)

        task = str(self.get_parameter("task").value).strip()
        self._memory = AgentMemory(task=task)
        self._max_steps = int(self.get_parameter("max_steps").value)
        self._heuristic = bool(self.get_parameter("heuristic_mode").value)
        self._use_vision = bool(self.get_parameter("use_vision").value)

        self._llm: LlmClient | None = None
        if not self._heuristic:
            try:
                base = str(self.get_parameter("llm_api_base").value).strip() or None
                model = str(self.get_parameter("llm_model").value).strip() or None
                self._llm = LlmClient.from_env(api_base=base, model=model)
                self.get_logger().info(f"LLM: {self._llm.model} @ {self._llm.api_base}")
            except LlmClientError as exc:
                self.get_logger().warn(f"{exc}; usando heuristic_mode")
                self._heuristic = True

        self._tools = RosToolBridge(
            self,
            self._memory,
            setpoint_topic=str(self.get_parameter("setpoint_topic").value),
            state_topic=str(self.get_parameter("state_topic").value),
            pose_topic=str(self.get_parameter("local_pose_topic").value),
            detections_topic=str(self.get_parameter("detections_topic").value),
            image_topic=str(self.get_parameter("image_topic").value),
            rate_hz=float(self.get_parameter("rate_hz").value),
            search_radius_max=float(self.get_parameter("search_radius_max").value),
            pre_setpoints_sec=float(self.get_parameter("pre_setpoints_sec").value),
            use_vision=self._use_vision,
            llm_client=self._llm,
        )

        self._status_pub = self.create_publisher(String, "/drone_agent/status", 10)
        self._running = False

        if task:
            self._start_mission(task)
        else:
            topic = str(self.get_parameter("task_topic").value)
            self.create_subscription(String, topic, self._on_task_msg, 10)
            self.get_logger().info(f"Aguardando tarefa em {topic} (std_msgs/String)")

    def _on_task_msg(self, msg: String) -> None:
        task = msg.data.strip()
        if not task or self._running:
            return
        self._start_mission(task)

    def _start_mission(self, task: str) -> None:
        if self._running:
            return
        self._memory.task = task
        self._tools.reset_task_state()
        self._running = True
        thread = threading.Thread(target=self._run_mission, args=(task,), daemon=True)
        thread.start()

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _run_mission(self, task: str) -> None:
        self._publish_status(f"Missão iniciada: {task}")
        try:
            if self._heuristic:
                self._run_heuristic(task)
            else:
                self._run_lvlm(task)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Missão falhou: {exc}")
            self._publish_status(f"ERRO: {exc}")
        finally:
            self._running = False

    def _run_heuristic(self, task: str) -> None:
        planner = HeuristicPlanner()
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.2)
        status = json.loads(self._tools.execute("get_drone_status", {}))
        for step in planner.plan(task, status):
            if self._tools.task_done:
                break
            name = step["name"]
            args = step.get("arguments", {})
            result = self._tools.execute(name, args)
            self._publish_status(f"{name}: {result}")
            if self._tools.task_done:
                break
            time.sleep(0.3)
        res = self._tools.task_result or {"summary": "concluído", "success": True}
        self._publish_status(f"Fim: {res}")

    def _run_lvlm(self, task: str) -> None:
        assert self._llm is not None
        tools = tool_schemas()
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {
                "role": "user",
                "content": (
                    f"Tarefa: {task}\n\n"
                    f"Memória inicial:\n{self._memory.summary()}\n\n"
                    "Execute a tarefa chamando ferramentas."
                ),
            },
        ]

        for step in range(self._max_steps):
            if self._tools.task_done:
                break

            self._publish_status(f"LLM passo {step + 1}/{self._max_steps}")
            mem_block = self._memory.summary()
            messages.append(
                {
                    "role": "user",
                    "content": f"Memória atual:\n{mem_block}",
                }
            )

            response = self._llm.chat(messages, tools=tools)
            msg = LlmClient.assistant_message(response)
            messages.append(msg)

            tool_calls = msg.get("tool_calls") or []
            if not tool_calls:
                content = msg.get("content") or ""
                if content:
                    self._publish_status(f"LLM: {content[:300]}")
                break

            for call in tool_calls:
                fn = call.get("function", {})
                name = fn.get("name", "")
                raw_args = fn.get("arguments", "{}")
                try:
                    args = json.loads(raw_args) if isinstance(raw_args, str) else dict(raw_args)
                except json.JSONDecodeError:
                    args = {}

                result = self._tools.execute(name, args)
                # Erros de arming/OFFBOARD trazem diagnóstico estruturado; não truncar.
                self._publish_status(f"{name}: {result}")

                messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call.get("id", name),
                        "content": result,
                    }
                )
                if self._tools.task_done:
                    break

            if self._tools.task_done:
                break

        res = self._tools.task_result
        if res:
            self._publish_status(f"Fim: {res['summary']} (success={res['success']})")
        else:
            self._publish_status("Fim: max_steps atingido sem complete_task")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LvlmAgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
