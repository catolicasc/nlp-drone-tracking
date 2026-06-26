"""ROS/MAVROS tools for Oracle Vision V2."""

from __future__ import annotations

import base64
import binascii
import json
import math
import struct
import time
import zlib
from typing import Any, Callable

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .memory import V2Memory
from .vlm_client import VlmClient


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    crc = binascii.crc32(kind + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)


def _rgb_bytes_from_image(msg: Image) -> bytes:
    encoding = (msg.encoding or "").lower()
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    data = bytes(msg.data)
    if width <= 0 or height <= 0:
        raise ValueError(f"dimensões inválidas: width={width} height={height}")

    if encoding in ("rgb8", "8uc3"):
        channels = 3
        mode = "rgb"
    elif encoding == "bgr8":
        channels = 3
        mode = "bgr"
    elif encoding in ("rgba8", "8uc4"):
        channels = 4
        mode = "rgba"
    elif encoding == "bgra8":
        channels = 4
        mode = "bgra"
    elif encoding in ("mono8", "8uc1"):
        channels = 1
        mode = "mono"
    elif not encoding and step % width == 0 and step // width in (3, 4):
        channels = step // width
        mode = "rgb" if channels == 3 else "rgba"
    else:
        raise ValueError(
            f"encoding não suportado: {msg.encoding!r}, width={width}, height={height}, step={step}"
        )

    min_step = width * channels
    if step < min_step:
        raise ValueError(f"step inválido: step={step}, mínimo esperado={min_step}")
    if len(data) < step * height:
        raise ValueError(f"imagem incompleta: bytes={len(data)}, esperado={step * height}")

    rows: list[bytes] = []
    for row_idx in range(height):
        start = row_idx * step
        raw = data[start : start + min_step]
        if mode == "rgb":
            rows.append(raw)
        elif mode == "bgr":
            rows.append(bytes(v for i in range(0, len(raw), 3) for v in (raw[i + 2], raw[i + 1], raw[i])))
        elif mode == "rgba":
            rows.append(bytes(v for i in range(0, len(raw), 4) for v in (raw[i], raw[i + 1], raw[i + 2])))
        elif mode == "bgra":
            rows.append(bytes(v for i in range(0, len(raw), 4) for v in (raw[i + 2], raw[i + 1], raw[i])))
        else:
            rows.append(bytes(v for px in raw for v in (px, px, px)))
    return b"".join(rows)


def _png_data_url_from_image(msg: Image) -> str:
    width = int(msg.width)
    height = int(msg.height)
    rgb = _rgb_bytes_from_image(msg)
    scanlines = b"".join(
        b"\x00" + rgb[row * width * 3 : (row + 1) * width * 3]
        for row in range(height)
    )
    payload = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(scanlines))
        + _png_chunk(b"IEND", b"")
    )
    return "data:image/png;base64," + base64.b64encode(payload).decode("ascii")


class V2RosTools:
    def __init__(
        self,
        node: Node,
        memory: V2Memory,
        vlm: VlmClient,
        *,
        min_alt_m: float,
        max_alt_m: float,
        max_radius_m: float,
        rate_hz: float,
        pre_setpoints_sec: float,
    ) -> None:
        self._node = node
        self._memory = memory
        self._vlm = vlm
        self._min_alt_m = float(min_alt_m)
        self._max_alt_m = float(max_alt_m)
        self._max_radius_m = float(max_radius_m)
        self._rate_hz = max(float(rate_hz), 1.0)
        self._pre_setpoints_sec = max(float(pre_setpoints_sec), 1.0)

        self._state = State()
        self._have_state = False
        self._pose = PoseStamped()
        self._have_pose = False
        self._last_image: Image | None = None
        self._last_statustext = ""
        self._statustext_history: list[str] = []

        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = self._min_alt_m
        self._cmd_yaw = 0.0
        self._stream_setpoints = False
        self._task_done = False
        self._task_result: dict[str, Any] | None = None

        self._setpoint_pub = node.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self._timer = node.create_timer(1.0 / self._rate_hz, self._publish_setpoint)
        self._set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
        self._arm_client = node.create_client(CommandBool, "/mavros/cmd/arming")

        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_best_effort.durability = DurabilityPolicy.VOLATILE

        node.create_subscription(State, "/mavros/state", self._on_state, 10)
        node.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pose, qos_best_effort)
        node.create_subscription(StatusText, "/mavros/statustext/recv", self._on_statustext, qos_best_effort)
        node.create_subscription(Image, "/drone/camera/image_raw", self._on_image, 10)

        self._handlers: dict[str, Callable[[dict[str, Any]], str]] = {
            "get_drone_status": lambda _: self.get_drone_status(),
            "inspect_camera": self.inspect_camera,
            "scan_for_people": self.scan_for_people,
            "takeoff": self.takeoff,
            "goto_position": self.goto_position,
            "goto_relative": self.goto_relative,
            "set_yaw": self.set_yaw,
            "rotate_yaw": self.rotate_yaw,
            "hover": self.hover,
            "land": lambda _: self.land(),
            "complete_task": self.complete_task,
        }

    @property
    def task_done(self) -> bool:
        return self._task_done

    @property
    def task_result(self) -> dict[str, Any] | None:
        return self._task_result

    def reset_task(self) -> None:
        self._task_done = False
        self._task_result = None

    def execute(self, name: str, args: dict[str, Any]) -> str:
        handler = self._handlers.get(name)
        if handler is None:
            return json.dumps({"ok": False, "error": f"tool desconhecida: {name}"}, ensure_ascii=False)
        try:
            result = handler(args)
            self._memory.add_action(f"{name}({args}) -> {result[:220]}")
            return result
        except Exception as exc:  # noqa: BLE001
            result = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self._memory.add_action(f"{name} FAILED: {exc}")
            return result

    def _on_state(self, msg: State) -> None:
        self._state = msg
        self._have_state = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg
        self._have_pose = True
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self._memory.set_home(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
            yaw=float(yaw),
        )

    def _on_statustext(self, msg: StatusText) -> None:
        self._last_statustext = str(msg.text)
        entry = f"severity={int(msg.severity)} text={self._last_statustext}"
        if not self._statustext_history or self._statustext_history[-1] != entry:
            self._statustext_history.append(entry)
            self._statustext_history = self._statustext_history[-12:]

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg

    def _publish_setpoint(self) -> None:
        if not self._stream_setpoints:
            return
        sp = PoseStamped()
        sp.header.stamp = self._node.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(self._cmd_x)
        sp.pose.position.y = float(self._cmd_y)
        sp.pose.position.z = float(self._cmd_z)
        sp.pose.orientation = yaw_to_quaternion(self._cmd_yaw)
        self._setpoint_pub.publish(sp)

    def _wait(self, sec: float) -> None:
        time.sleep(max(sec, 0.0))

    def _call_service(self, client: Any, req: Any, timeout_sec: float = 10.0) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=1.0):
            return False, "serviço indisponível"
        future = client.call_async(req)
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > timeout_sec:
                future.cancel()
                return False, "timeout"
            time.sleep(0.05)
        result = future.result()
        return result is not None, str(result)

    def _set_mode(self, mode: str) -> tuple[bool, str]:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        ok, text = self._call_service(self._set_mode_client, req)
        sent = "mode_sent=True" in text or "mode_sent: true" in text.lower()
        return bool(ok and sent), text

    def _arm(self) -> tuple[bool, str]:
        req = CommandBool.Request()
        req.value = True
        ok, text = self._call_service(self._arm_client, req)
        accepted = "success=True" in text or "success: true" in text.lower()
        return bool(ok and accepted), text

    def _connected(self) -> bool:
        return bool(self._have_state and self._state.connected)

    def _current_pose(self) -> tuple[float, float, float, float]:
        if not self._have_pose:
            return self._cmd_x, self._cmd_y, self._cmd_z, self._cmd_yaw
        return (
            float(self._pose.pose.position.x),
            float(self._pose.pose.position.y),
            float(self._pose.pose.position.z),
            yaw_from_quaternion(self._pose.pose.orientation),
        )

    def _home_xy(self) -> tuple[float, float]:
        if self._memory.home:
            return self._memory.home["x"], self._memory.home["y"]
        x, y, _, _ = self._current_pose()
        return x, y

    def _safe_xyz(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        hx, hy = self._home_xy()
        dx = x - hx
        dy = y - hy
        dist = math.hypot(dx, dy)
        if dist > self._max_radius_m:
            scale = self._max_radius_m / max(dist, 1e-9)
            x = hx + dx * scale
            y = hy + dy * scale
        return x, y, clamp(z, self._min_alt_m, self._max_alt_m)

    def _diagnostics(self) -> dict[str, Any]:
        return {
            "state": self._state_snapshot(),
            "last_statustext": self._last_statustext or None,
            "statustext_history": self._statustext_history[-8:],
            "limits": {
                "min_alt_m": self._min_alt_m,
                "max_alt_m": self._max_alt_m,
                "max_radius_m": self._max_radius_m,
            },
        }

    def _state_snapshot(self) -> dict[str, Any]:
        pose = None
        if self._have_pose:
            x, y, z, yaw = self._current_pose()
            pose = {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3), "yaw": round(yaw, 3)}
        return {
            "have_state": bool(self._have_state),
            "connected": bool(self._state.connected) if self._have_state else False,
            "armed": bool(self._state.armed) if self._have_state else False,
            "mode": str(self._state.mode) if self._have_state else "",
            "pose": pose,
            "setpoint": {
                "x": round(self._cmd_x, 3),
                "y": round(self._cmd_y, 3),
                "z": round(self._cmd_z, 3),
                "yaw": round(self._cmd_yaw, 3),
            },
            "streaming_setpoints": bool(self._stream_setpoints),
        }

    def get_drone_status(self) -> str:
        return json.dumps({"ok": True, **self._state_snapshot(), "limits": self._diagnostics()["limits"]}, ensure_ascii=False)

    def _inspect_image(self, question: str) -> dict[str, Any]:
        if self._last_image is None:
            return {"ok": False, "error": "sem imagem em /drone/camera/image_raw"}
        try:
            data_url = _png_data_url_from_image(self._last_image)
        except Exception as exc:  # noqa: BLE001
            return {
                "ok": False,
                "error": f"falha ao converter imagem ROS para PNG: {exc}",
                "image": {
                    "encoding": self._last_image.encoding,
                    "width": int(self._last_image.width),
                    "height": int(self._last_image.height),
                    "step": int(self._last_image.step),
                },
            }
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": question},
                    {"type": "image_url", "image_url": {"url": data_url}},
                ],
            }
        ]
        response = self._vlm.chat(messages, temperature=0.1)
        answer = self._vlm.assistant_message(response).get("content", "") or ""
        self._memory.add_finding(f"camera: {answer[:300]}")
        return {"ok": True, "answer": answer}

    def inspect_camera(self, args: dict[str, Any]) -> str:
        question = str(args.get("question", "Descreva a cena atual da câmera do drone."))
        return json.dumps(self._inspect_image(question), ensure_ascii=False)

    @staticmethod
    def _parse_person_detection(answer: str) -> dict[str, Any]:
        text = answer.strip()
        if text.startswith("```"):
            text = text.strip("`")
            if text.lower().startswith("json"):
                text = text[4:].strip()
        try:
            parsed = json.loads(text)
            if isinstance(parsed, dict):
                return {
                    "found": bool(parsed.get("found", False)),
                    "confidence": parsed.get("confidence", "unknown"),
                    "description": str(parsed.get("description", "")),
                }
        except json.JSONDecodeError:
            pass
        low = answer.lower()
        negative = any(term in low for term in ("não vejo", "nao vejo", "não há", "nao ha", "sem pessoa", "no person"))
        positive = any(term in low for term in ("pessoa", "person", "humano", "human"))
        return {"found": bool(positive and not negative), "confidence": "unknown", "description": answer[:500]}

    def scan_for_people(self, args: dict[str, Any]) -> str:
        if not (self._have_state and self._state.armed and self._state.mode == "OFFBOARD"):
            return json.dumps(
                {
                    "ok": False,
                    "error": "scan_for_people exige drone armado em OFFBOARD; use takeoff antes.",
                    "diagnostics": self._diagnostics(),
                },
                ensure_ascii=False,
            )

        sweeps = int(clamp(float(args.get("sweeps", 1)), 1.0, 3.0))
        step_deg = clamp(float(args.get("step_deg", 45.0)), 15.0, 120.0)
        settle_sec = clamp(float(args.get("settle_sec", 1.0)), 0.2, 5.0)
        stop_on_found = bool(args.get("stop_on_found", True))
        total_steps = max(1, int(round((360.0 / step_deg) * sweeps)))
        observations: list[dict[str, Any]] = []

        question = (
            "Analise esta imagem da câmera do drone e responda somente JSON válido, sem markdown, "
            'no formato {"found": boolean, "confidence": "low|medium|high", '
            '"description": "descrição curta em português"}. '
            "Marque found=true apenas se houver uma pessoa/humano visível."
        )

        for idx in range(total_steps):
            if idx > 0:
                self._cmd_yaw += math.radians(step_deg)
                self._stream_setpoints = True
                self._wait(settle_sec)
            result = self._inspect_image(question)
            if not result.get("ok"):
                observations.append({"step": idx + 1, "ok": False, "error": result.get("error")})
                continue
            detection = self._parse_person_detection(str(result.get("answer", "")))
            observation = {
                "step": idx + 1,
                "yaw_deg": round(math.degrees(self._cmd_yaw), 1),
                **detection,
            }
            observations.append(observation)
            if detection["found"]:
                self._memory.add_finding(
                    f"Pessoa encontrada no passo {idx + 1}, yaw={observation['yaw_deg']} deg: "
                    f"{detection['description']}"
                )
                if stop_on_found:
                    break

        found_items = [item for item in observations if item.get("found")]
        return json.dumps(
            {
                "ok": True,
                "found": bool(found_items),
                "first_match": found_items[0] if found_items else None,
                "observations": observations,
                "steps_completed": len(observations),
            },
            ensure_ascii=False,
        )

    def takeoff(self, args: dict[str, Any]) -> str:
        if not self._connected():
            return json.dumps({"ok": False, "error": "MAVROS não conectado", "diagnostics": self._diagnostics()}, ensure_ascii=False)
        z = clamp(float(args["z"]), self._min_alt_m, self._max_alt_m)
        x, y, _, yaw = self._current_pose()
        self._cmd_x, self._cmd_y, self._cmd_z = self._safe_xyz(x, y, z)
        self._cmd_yaw = yaw
        self._stream_setpoints = True
        self._wait(self._pre_setpoints_sec)

        mode_ok = False
        arm_ok = bool(self._have_state and self._state.armed)
        mode_text = ""
        arm_text = ""
        for _ in range(15):
            mode_ok, mode_text = self._set_mode("OFFBOARD")
            self._wait(0.35)
            if self._have_state and self._state.mode == "OFFBOARD":
                break
        for _ in range(15):
            arm_ok, arm_text = self._arm()
            self._wait(0.35)
            if self._have_state and self._state.armed:
                break
        ok = bool(self._have_state and self._state.armed and self._state.mode == "OFFBOARD")
        return json.dumps(
            {
                "ok": ok,
                "z": z,
                "mode_ok": mode_ok,
                "arm_ok": arm_ok,
                "mode_response": mode_text[-300:],
                "arm_response": arm_text[-300:],
                "diagnostics": self._diagnostics(),
            },
            ensure_ascii=False,
        )

    def goto_position(self, args: dict[str, Any]) -> str:
        if not (self._have_state and self._state.armed and self._state.mode == "OFFBOARD"):
            return json.dumps({"ok": False, "error": "drone não está armado em OFFBOARD"}, ensure_ascii=False)
        hold = float(args.get("hold_sec", 3.0))
        self._cmd_x, self._cmd_y, self._cmd_z = self._safe_xyz(float(args["x"]), float(args["y"]), float(args["z"]))
        self._stream_setpoints = True
        self._wait(hold)
        return json.dumps({"ok": True, "setpoint": self._state_snapshot()["setpoint"]}, ensure_ascii=False)

    def goto_relative(self, args: dict[str, Any]) -> str:
        x, y, z, _ = self._current_pose()
        return self.goto_position(
            {
                "x": x + float(args["dx"]),
                "y": y + float(args["dy"]),
                "z": z + float(args["dz"]),
                "hold_sec": float(args.get("hold_sec", 3.0)),
            }
        )

    def set_yaw(self, args: dict[str, Any]) -> str:
        self._cmd_yaw = math.radians(float(args["yaw_deg"]))
        self._stream_setpoints = True
        return json.dumps({"ok": True, "yaw_deg": float(args["yaw_deg"])}, ensure_ascii=False)

    def rotate_yaw(self, args: dict[str, Any]) -> str:
        self._cmd_yaw += math.radians(float(args["delta_deg"]))
        self._stream_setpoints = True
        return json.dumps({"ok": True, "yaw_deg": math.degrees(self._cmd_yaw)}, ensure_ascii=False)

    def hover(self, args: dict[str, Any]) -> str:
        self._stream_setpoints = True
        self._wait(float(args["duration_sec"]))
        return json.dumps({"ok": True, "duration_sec": float(args["duration_sec"])}, ensure_ascii=False)

    def land(self) -> str:
        self._stream_setpoints = False
        ok, response = self._set_mode("AUTO.LAND")
        self._wait(1.0)
        return json.dumps(
            {"ok": bool(ok), "response": response[-300:], "diagnostics": self._diagnostics()},
            ensure_ascii=False,
        )

    def complete_task(self, args: dict[str, Any]) -> str:
        self._task_done = True
        self._task_result = {
            "summary": str(args["summary"]),
            "success": bool(args["success"]),
        }
        return json.dumps({"ok": True, **self._task_result}, ensure_ascii=False)
