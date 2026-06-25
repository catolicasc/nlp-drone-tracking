"""Ferramentas ROS 2 invocáveis pelo agente LVLM."""

from __future__ import annotations

import base64
import json
import math
import time
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, SetMode
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
        self._statustext_buf: list[str] = []

        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = 1.0
        self._yaw = 0.0
        self._home_set = False
        self._task_done = False
        self._task_result: dict[str, Any] | None = None
        self._streaming_override = False

        self._setpoint_pub = node.create_publisher(PoseStamped, setpoint_topic, 10)
        self._setpoint_timer = node.create_timer(self._dt, self._on_setpoint_timer)

        self._set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
        self._arm_client = node.create_client(CommandBool, "/mavros/cmd/arming")

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        pose_qos.durability = DurabilityPolicy.VOLATILE

        node.create_subscription(State, state_topic, self._on_state, 10)
        node.create_subscription(StatusText, "/mavros/statustext/recv", self._on_statustext, pose_qos)
        node.create_subscription(PoseStamped, pose_topic, self._on_pose, pose_qos)
        node.create_subscription(Detection2DArray, detections_topic, self._on_detections, 10)
        node.create_subscription(Image, image_topic, self._on_image, 10)

        self._handlers: dict[str, Callable[[dict[str, Any]], str]] = {
            "get_drone_status": lambda _: self._get_drone_status(),
            "get_detections": lambda _: self._get_detections(),
            "inspect_camera": self._inspect_camera,
            "takeoff": self._takeoff,
            "goto_position": self._goto_position,
            "goto_relative": self._goto_relative,
            "set_yaw": self._set_yaw,
            "rotate_yaw": self._rotate_yaw,
            "return_home": lambda _: self._return_home(),
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

    def reset_task_state(self) -> None:
        self._task_done = False
        self._task_result = None

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
        self._statustext_buf.append(str(msg.text))
        if len(self._statustext_buf) > 20:
            self._statustext_buf.pop(0)

    def _on_setpoint_timer(self) -> None:
        if not self._streaming_override and self._have_state and self._state.mode == "AUTO.LAND":
            return
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

    def _call_service(self, client: rclpy.client.Client, request: Any, timeout: float = 5.0) -> tuple[bool, str]:
        future = client.call_async(request)
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                future.cancel()
                return False, "timeout"
            time.sleep(0.05)
        result = future.result()
        if result is not None:
            return True, str(result)
        return True, "null"

    def _set_mode_sync(self, mode: str) -> tuple[bool, str]:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        return self._call_service(self._set_mode_client, req, timeout=10.0)

    def _arm_sync(self) -> tuple[bool, str]:
        req = CommandBool.Request()
        req.value = True
        return self._call_service(self._arm_client, req, timeout=10.0)

    def _arm_diagnostics(self) -> str:
        parts = []
        if self._have_state:
            parts.append(f"mode={self._state.mode} armed={self._state.armed} connected={self._state.connected}")
        if self._have_pose:
            p = self._pose.pose.position
            parts.append(f"pose=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
        parts.append(f"setpoint=({self._cmd_x:.2f},{self._cmd_y:.2f},{self._cmd_z:.2f})")
        if self._last_statustext:
            parts.append(f"px4: {self._last_statustext}")
        return "; ".join(parts) if parts else "sem diagnóstico PX4"

    def _state_snapshot(self) -> dict[str, Any]:
        snap: dict[str, Any] = {}
        if self._have_state:
            snap["mode"] = str(self._state.mode)
            snap["armed"] = bool(self._state.armed)
            snap["connected"] = bool(self._state.connected)
        if self._have_pose:
            p = self._pose.pose.position
            snap["x"] = round(float(p.x), 3)
            snap["y"] = round(float(p.y), 3)
            snap["z"] = round(float(p.z), 3)
        return snap

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
            self._node.get_logger().info("get_drone_status: aguardando /mavros/state...")
            t0 = time.monotonic()
            while not self._have_state and time.monotonic() - t0 < 15.0:
                time.sleep(0.2)
            if not self._have_state:
                return json.dumps({"ok": False, "error": "sem /mavros/state após 15s. MAVROS rodando?"})
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
        log = self._node.get_logger()

        if not self._wait_for_mavros():
            return json.dumps({"ok": False, "error": "MAVROS não conectado",
                               "diagnostics": self._arm_diagnostics()})

        if self._have_pose:
            self._cmd_x = float(self._pose.pose.position.x)
            self._cmd_y = float(self._pose.pose.position.y)
        self._cmd_z = z

        if self._have_state and self._state.mode == "AUTO.LAND":
            log.info("takeoff: AUTO.LAND → STABILIZE...")
            ok, resp = self._set_mode_sync("STABILIZE")
            log.info(f"  STABILIZE: sent={ok} resp={resp}")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self._node, timeout_sec=0.2)
                if self._have_state and self._state.mode != "AUTO.LAND":
                    break
            log.info(f"  mode agora: {self._state.mode if self._have_state else '?'}")

        self._streaming_override = True
        log.info(f"takeoff z={z}: aquecendo setpoints {self._pre_setpoints_sec}s...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < self._pre_setpoints_sec:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            time.sleep(0.1)
        self._streaming_override = False

        current_mode = self._state.mode if self._have_state else "UNKNOWN"
        log.info(f"takeoff: modo={current_mode} setpoint=({self._cmd_x:.2f},{self._cmd_y:.2f},{self._cmd_z:.2f})")

        offboard_ok = False
        offboard_attempts = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0:
            sent, resp = self._set_mode_sync("OFFBOARD")
            offboard_attempts += 1
            time.sleep(0.3)
            for _ in range(3):
                rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._have_state and self._state.mode == "OFFBOARD":
                offboard_ok = True
                log.info(f"takeoff: OFFBOARD aceito em {offboard_attempts} tentativas")
                break
            log.info(f"takeoff: OFFBOARD tentativa={offboard_attempts} modo={self._state.mode if self._have_state else '?'} resp={resp}")

        if not offboard_ok:
            return json.dumps({
                "ok": False,
                "error": "falha ao entrar em OFFBOARD",
                "attempts": offboard_attempts,
                "diagnostics": self._arm_diagnostics(),
            }, ensure_ascii=False)

        log.info("takeoff: OFFBOARD ok, pausando 2s antes de armar...")
        time.sleep(2.0)
        for _ in range(5):
            rclpy.spin_once(self._node, timeout_sec=0.2)

        arm_ok = False
        arm_attempts = 0
        last_arm_resp = ""
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0:
            sent, resp = self._arm_sync()
            last_arm_resp = resp
            arm_attempts += 1
            for _ in range(3):
                rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._have_state and self._state.armed:
                arm_ok = True
                log.info(f"takeoff: ARM aceito em {arm_attempts} tentativas")
                break
            px4_msg = self._last_statustext
            log.info(f"takeoff: ARM tentativa={arm_attempts} armed={self._state.armed if self._have_state else '?'} px4={px4_msg} resp={resp}")
            time.sleep(0.5)

        if not arm_ok:
            return json.dumps({
                "ok": False,
                "error": "falha ao armar",
                "attempts": arm_attempts,
                "arm_response": last_arm_resp,
                "px4_messages": list(self._statustext_buf[-5:]),
                "diagnostics": self._arm_diagnostics(),
            }, ensure_ascii=False)

        log.info(f"takeoff: armed! aguardando subir para z={z}...")
        t0 = time.monotonic()
        reached = False
        last_log = 0.0
        while time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self._node, timeout_sec=0.2)
            current_z = self._pose.pose.position.z if self._have_pose else 0.0
            if current_z >= z * 0.9:
                reached = True
                break
            now = time.monotonic()
            if now - last_log >= 2.0:
                log.info(f"takeoff: z atual={current_z:.2f}m / target={z}m")
                last_log = now
            time.sleep(0.2)

        final_z = self._pose.pose.position.z if self._have_pose else 0.0
        if reached:
            log.info(f"takeoff: altitude {final_z:.2f}m atingida (target={z}m)")
            return json.dumps({"ok": True, "armed": True, "z": round(final_z, 3)})
        else:
            log.warning(f"takeoff: timeout subindo — z atual={final_z:.2f}m target={z}m")
            return json.dumps({"ok": False, "error": f"timeout subindo para z={z}",
                               "current_z": round(final_z, 3)}, ensure_ascii=False)

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

    def _goto_relative(self, args: dict[str, Any]) -> str:
        dx = float(args["dx"])
        dy = float(args["dy"])
        dz = float(args["dz"])
        hold = float(args.get("hold_sec", 4.0))
        x = self._cmd_x + dx
        y = self._cmd_y + dy
        z = max(0.2, self._cmd_z + dz)
        home = self._memory.home_pose or {"x": self._cmd_x, "y": self._cmd_y}
        x, y = clamp_xy_radius(x, y, home["x"], home["y"], self._search_radius_max)
        self._cmd_x, self._cmd_y, self._cmd_z = x, y, z
        self._spin_wait(max(hold, 1.0))
        return json.dumps({"ok": True, "x": round(x, 3), "y": round(y, 3), "z": round(z, 3), "held_sec": hold})

    def _set_yaw(self, args: dict[str, Any]) -> str:
        yaw_deg = float(args["yaw_deg"])
        self._yaw = math.radians(yaw_deg)
        self._spin_wait(1.5)
        return json.dumps({"ok": True, "yaw_deg": round(yaw_deg, 1)})

    def _rotate_yaw(self, args: dict[str, Any]) -> str:
        delta_deg = float(args["delta_deg"])
        self._yaw += math.radians(delta_deg)
        self._spin_wait(1.5)
        return json.dumps({"ok": True, "delta_deg": round(delta_deg, 1), "yaw_deg": round(math.degrees(self._yaw), 1)})

    def _return_home(self) -> str:
        hp = self._memory.home_pose
        if hp is None:
            return json.dumps({"ok": False, "error": "home não definido"})
        self._cmd_x = hp["x"]
        self._cmd_y = hp["y"]
        self._cmd_z = hp["z"]
        self._yaw = hp["yaw"]
        self._spin_wait(4.0)
        return json.dumps({"ok": True, "home": hp})

    def _hover(self, args: dict[str, Any]) -> str:
        duration = float(args["duration_sec"])
        self._spin_wait(max(duration, 0.5))
        return json.dumps({"ok": True, "duration_sec": duration})

    def _land(self) -> str:
        ok, resp = self._set_mode_sync("AUTO.LAND")
        if not ok:
            return json.dumps(
                {
                    "ok": False,
                    "error": "falha ao enviar AUTO.LAND",
                    "response": resp,
                    "diagnostics": self._arm_diagnostics(),
                },
                ensure_ascii=False,
            )

        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            if self._have_state and self._state.mode == "AUTO.LAND":
                return json.dumps(
                    {"ok": True, "mode": "AUTO.LAND", "armed": bool(self._state.armed)},
                    ensure_ascii=False,
                )
            time.sleep(0.2)

        return json.dumps(
            {
                "ok": True,
                "mode_sent": "AUTO.LAND",
                "note": "comando aceito, mas /mavros/state ainda não refletiu AUTO.LAND",
                "state": self._state_snapshot(),
            },
            ensure_ascii=False,
        )

    def _complete_task(self, args: dict[str, Any]) -> str:
        self._task_done = True
        self._task_result = {
            "summary": str(args.get("summary", "")),
            "success": bool(args.get("success", False)),
        }
        return json.dumps({"ok": True, **self._task_result})
