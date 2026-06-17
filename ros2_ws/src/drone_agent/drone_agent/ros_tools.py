"""Ferramentas ROS 2 invocáveis pelo agente LVLM."""

from __future__ import annotations

import base64
import json
import math
import os
import shutil
import subprocess
import time
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, StatusText
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from .llm_client import LlmClient
from .memory import AgentMemory


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def _yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp_xy_radius(x: float, y: float, cx: float, cy: float, r_max: float) -> tuple[float, float]:
    if r_max <= 0.0:
        return cx, cy
    dx, dy = x - cx, y - cy
    d = math.hypot(dx, dy)
    if d <= r_max:
        return x, y
    s = r_max / max(d, 1e-9)
    return cx + dx * s, cy + dy * s


class RosToolBridge:
    def __init__(
        self,
        node: Node,
        memory: AgentMemory,
        *,
        setpoint_topic: str,
        state_topic: str,
        pose_topic: str,
        detections_topic: str,
        image_topic: str,
        rate_hz: float,
        search_radius_max: float,
        pre_setpoints_sec: float,
        use_vision: bool,
        llm_client: LlmClient | None,
    ) -> None:
        self._node = node
        self._memory = memory
        self._rate_hz = max(rate_hz, 1.0)
        self._dt = 1.0 / self._rate_hz
        self._search_radius_max = search_radius_max
        self._pre_setpoints_sec = pre_setpoints_sec
        self._use_vision = use_vision
        self._llm = llm_client

        self._state = State()
        self._have_state = False
        self._pose = PoseStamped()
        self._have_pose = False
        self._detections = Detection2DArray()
        self._have_detections = False
        self._last_image: Image | None = None
        self._last_statustext = ""

        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = 1.0
        self._yaw = 0.0
        self._home_set = False
        self._task_done = False
        self._task_result: dict[str, Any] | None = None

        self._setpoint_pub = node.create_publisher(PoseStamped, setpoint_topic, 10)
        self._setpoint_timer = node.create_timer(self._dt, self._on_setpoint_timer)

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        pose_qos.durability = DurabilityPolicy.VOLATILE

        node.create_subscription(State, state_topic, self._on_state, 10)
        node.create_subscription(StatusText, "/mavros/statustext/recv", self._on_statustext, 10)
        node.create_subscription(PoseStamped, pose_topic, self._on_pose, pose_qos)
        node.create_subscription(Detection2DArray, detections_topic, self._on_detections, 10)
        node.create_subscription(Image, image_topic, self._on_image, 10)

        self._handlers: dict[str, Callable[[dict[str, Any]], str]] = {
            "get_drone_status": lambda _: self._get_drone_status(),
            "get_detections": lambda _: self._get_detections(),
            "inspect_camera": self._inspect_camera,
            "takeoff": self._takeoff,
            "goto_position": self._goto_position,
            "hover": self._hover,
            "land": lambda _: self._land(),
            "complete_task": self._complete_task,
        }

    @property
    def task_done(self) -> bool:
        return self._task_done

    @property
    def task_result(self) -> dict[str, Any] | None:
        return self._task_result

    def _on_state(self, msg: State) -> None:
        self._state = msg
        self._have_state = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg
        self._have_pose = True
        if not self._home_set:
            self._cmd_x = float(msg.pose.position.x)
            self._cmd_y = float(msg.pose.position.y)
            self._cmd_z = float(msg.pose.position.z)
            self._yaw = _yaw_from_quat(msg.pose.orientation)
            self._memory.set_home(self._cmd_x, self._cmd_y, self._cmd_z, self._yaw)
            self._home_set = True

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._detections = msg
        self._have_detections = True

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg

    def _on_statustext(self, msg: StatusText) -> None:
        self._last_statustext = str(msg.text)

    def _on_setpoint_timer(self) -> None:
        sp = PoseStamped()
        sp.header.stamp = self._node.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(self._cmd_x)
        sp.pose.position.y = float(self._cmd_y)
        sp.pose.position.z = float(self._cmd_z)
        sp.pose.orientation = yaw_to_quaternion(self._yaw)
        self._setpoint_pub.publish(sp)

    def _spin_wait(self, sec: float) -> None:
        """Aguarda mantendo setpoints (timer no executor principal)."""
        time.sleep(sec)

    def _wait_for_mavros(self, timeout_sec: float = 30.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_sec:
            if self._have_state and self._state.connected:
                return True
            time.sleep(0.2)
        return False

    def _ros2_service(self, service: str, srv_type: str, payload: str, timeout: float = 20.0) -> str:
        """Chama serviço MAVROS via CLI (seguro entre threads)."""
        ros2 = shutil.which("ros2")
        if not ros2:
            return "ros2 não encontrado no PATH"
        try:
            proc = subprocess.run(
                [ros2, "service", "call", service, srv_type, payload],
                capture_output=True,
                text=True,
                timeout=timeout,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired:
            return "timeout"
        out = (proc.stdout or "") + (proc.stderr or "")
        if proc.returncode != 0:
            return f"exit {proc.returncode}: {out.strip()}"
        return out

    def _set_mode_sync(self, mode: str) -> bool:
        payload = f"{{base_mode: 0, custom_mode: '{mode}'}}"
        out = self._ros2_service("/mavros/set_mode", "mavros_msgs/srv/SetMode", payload)
        return "mode_sent=True" in out or "mode_sent: true" in out.lower()

    def _arm_sync(self) -> bool:
        out = self._ros2_service(
            "/mavros/cmd/arming",
            "mavros_msgs/srv/CommandBool",
            "{value: true}",
        )
        return "success=True" in out or "success: true" in out.lower()

    def _ensure_offboard_armed(self, timeout_sec: float = 30.0) -> bool:
        t0 = time.monotonic()
        offboard_ok = False

        while time.monotonic() - t0 < timeout_sec:
            self._set_mode_sync("OFFBOARD")
            time.sleep(0.5)
            if self._have_state and self._state.mode == "OFFBOARD":
                offboard_ok = True
                break

        if not offboard_ok:
            return False

        t1 = time.monotonic()
        while time.monotonic() - t1 < timeout_sec:
            self._arm_sync()
            time.sleep(0.5)
            if self._have_state and self._state.armed:
                return True

        return bool(self._have_state and self._state.armed)

    def _arm_diagnostics(self) -> str:
        parts = []
        if self._have_state:
            parts.append(f"mode={self._state.mode} armed={self._state.armed}")
        if self._last_statustext:
            parts.append(f"px4: {self._last_statustext}")
        return "; ".join(parts) if parts else "sem diagnóstico PX4"

    def execute(self, name: str, arguments: dict[str, Any]) -> str:
        handler = self._handlers.get(name)
        if handler is None:
            return json.dumps({"ok": False, "error": f"unknown tool: {name}"})
        try:
            result = handler(arguments)
            self._memory.log_action(f"{name}({arguments}) -> {result[:200]}")
            return result
        except Exception as exc:  # noqa: BLE001
            err = json.dumps({"ok": False, "error": str(exc)})
            self._memory.log_action(f"{name} FAILED: {exc}")
            return err

    def _get_drone_status(self) -> str:
        if not self._have_state:
            return json.dumps({"ok": False, "error": "sem /mavros/state ainda"})
        pose = None
        if self._have_pose:
            p = self._pose.pose.position
            pose = {
                "x": round(float(p.x), 3),
                "y": round(float(p.y), 3),
                "z": round(float(p.z), 3),
                "yaw": round(_yaw_from_quat(self._pose.pose.orientation), 3),
            }
        return json.dumps(
            {
                "ok": True,
                "connected": bool(self._state.connected),
                "armed": bool(self._state.armed),
                "mode": str(self._state.mode),
                "pose": pose,
                "setpoint": {
                    "x": round(self._cmd_x, 3),
                    "y": round(self._cmd_y, 3),
                    "z": round(self._cmd_z, 3),
                },
            }
        )

    def _get_detections(self) -> str:
        if not self._have_detections:
            return json.dumps({"ok": True, "count": 0, "note": "sem mensagens de detecção ainda"})
        dets = self._detections.detections
        if not dets:
            return json.dumps({"ok": True, "count": 0})
        best = max(
            dets,
            key=lambda d: float(d.results[0].hypothesis.score) if d.results else 0.0,
        )
        score = float(best.results[0].hypothesis.score) if best.results else 0.0
        cx = float(best.bbox.center.position.x)
        cy = float(best.bbox.center.position.y)
        self._memory.add_finding(f"detecção score={score:.2f} cx={cx:.2f} cy={cy:.2f}")
        return json.dumps(
            {
                "ok": True,
                "count": len(dets),
                "best": {"score": round(score, 3), "cx_norm": round(cx, 3), "cy_norm": round(cy, 3)},
            }
        )

    def _inspect_camera(self, args: dict[str, Any]) -> str:
        question = str(args.get("question", "O que você vê?"))
        if not self._use_vision:
            return json.dumps({"ok": False, "error": "use_vision=false"})
        if self._last_image is None:
            return json.dumps({"ok": False, "error": "sem frame da câmera ainda"})
        if self._llm is None:
            return json.dumps({"ok": False, "error": "LLM não configurado para visão"})

        try:
            from cv_bridge import CvBridge
            import cv2
        except ImportError:
            return json.dumps({"ok": False, "error": "cv_bridge/opencv não disponível"})

        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(self._last_image, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            return json.dumps({"ok": False, "error": f"cv_bridge: {exc}"})

        ok, buf = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return json.dumps({"ok": False, "error": "falha ao codificar JPEG"})
        b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        data_url = f"data:image/jpeg;base64,{b64}"

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": question},
                    {"type": "image_url", "image_url": {"url": data_url}},
                ],
            }
        ]
        try:
            resp = self._llm.chat(messages, tools=None)
            text = resp["choices"][0]["message"].get("content", "")
        except Exception as exc:  # noqa: BLE001
            return json.dumps({"ok": False, "error": str(exc)})

        self._memory.add_finding(f"visão: {text[:240]}")
        return json.dumps({"ok": True, "answer": text})

    def _takeoff(self, args: dict[str, Any]) -> str:
        z = float(args["z"])
        if not self._wait_for_mavros():
            return json.dumps({"ok": False, "error": "MAVROS não conectado"})
        if self._have_pose:
            self._cmd_x = float(self._pose.pose.position.x)
            self._cmd_y = float(self._pose.pose.position.y)
        self._cmd_z = z
        self._node.get_logger().info(
            f"takeoff z={z}: aquecendo setpoints {self._pre_setpoints_sec}s..."
        )
        self._spin_wait(self._pre_setpoints_sec)
        if not self._ensure_offboard_armed():
            diag = self._arm_diagnostics()
            return json.dumps(
                {
                    "ok": False,
                    "error": "falha ao armar / OFFBOARD",
                    "hint": "sim em PLAY? aguarde 10s após iniciar",
                    "diagnostics": diag,
                }
            )
        return json.dumps({"ok": True, "armed": True, "z": z})

    def _goto_position(self, args: dict[str, Any]) -> str:
        x = float(args["x"])
        y = float(args["y"])
        z = float(args["z"])
        hold = float(args.get("hold_sec", 4.0))
        home = self._memory.home_pose or {"x": self._cmd_x, "y": self._cmd_y}
        x, y = clamp_xy_radius(x, y, home["x"], home["y"], self._search_radius_max)
        self._cmd_x, self._cmd_y, self._cmd_z = x, y, z
        self._spin_wait(max(hold, 1.0))
        return json.dumps({"ok": True, "x": x, "y": y, "z": z, "held_sec": hold})

    def _hover(self, args: dict[str, Any]) -> str:
        duration = float(args["duration_sec"])
        self._spin_wait(max(duration, 0.5))
        return json.dumps({"ok": True, "duration_sec": duration})

    def _land(self) -> str:
        ok = self._set_mode_sync("AUTO.LAND")
        if not ok:
            return json.dumps({"ok": False, "error": "falha ao enviar AUTO.LAND"})
        return json.dumps({"ok": True, "mode": "AUTO.LAND"})

    def _complete_task(self, args: dict[str, Any]) -> str:
        self._task_done = True
        self._task_result = {
            "summary": str(args.get("summary", "")),
            "success": bool(args.get("success", False)),
        }
        return json.dumps({"ok": True, **self._task_result})
