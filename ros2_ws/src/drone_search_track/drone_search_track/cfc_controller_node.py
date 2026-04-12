import math
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def clamp_xy_to_radius(x: float, y: float, cx: float, cy: float, r_max: float) -> tuple[float, float]:
    if r_max <= 0.0:
        return cx, cy
    dx = x - cx
    dy = y - cy
    d = math.hypot(dx, dy)
    if d <= r_max:
        return x, y
    s = r_max / max(d, 1e-9)
    return (cx + dx * s, cy + dy * s)


class CfcControllerNode(Node):
    def __init__(self):
        super().__init__("cfc_controller")

        self.declare_parameter("detections_topic", "/person_detector/detections")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_position/local")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("takeoff_z", 1.5)
        self.declare_parameter("pre_setpoints_sec", 2.5)

        self.declare_parameter("policy_mode", "tracking_only")  # tracking_only | search_and_track

        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("max_yaw_rate", 0.8)

        self.declare_parameter("search_radius_max", 10.0)
        self.declare_parameter("search_xy_speed", 0.8)
        self.declare_parameter("search_yaw_rate", 0.25)
        self.declare_parameter("search_yaw_limit_deg", 30.0)
        self.declare_parameter("search_yaw_bounce", True)

        self.declare_parameter("pos_smooth_vmax", 1.0)
        self.declare_parameter("yaw_rate_alpha", 0.85)

        self.declare_parameter("track_kp", 0.6)
        self.declare_parameter("track_yaw_on_detect", True)

        self.declare_parameter("model_path", "")
        self.declare_parameter("use_lnn", True)

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(self._rate_hz, 1e-3)

        self._state = State()
        self._have_state = False

        self._local_pose = PoseStamped()
        self._have_local_pose = False

        self._home_x = 0.0
        self._home_y = 0.0
        self._home_yaw = 0.0
        self._search_yaw_dir = 1.0

        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = float(self.get_parameter("takeoff_z").value)
        self._hold_z = float(self.get_parameter("takeoff_z").value)
        self._yaw = 0.0
        self._yaw_rate_filt = 0.0

        self._hover_active = False
        self._hover_x = 0.0
        self._hover_y = 0.0

        self._have_detection = False
        self._last_det_time = 0.0
        self._last_det_cx_norm = 0.0
        self._last_det_score = 0.0

        self._person_found_pub = self.create_publisher(Bool, "/person_found", 10)
        self._last_person_found = False

        self._setpoint_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("setpoint_topic").value),
            10,
        )

        self._state_sub = self.create_subscription(
            State,
            str(self.get_parameter("state_topic").value),
            self._on_state,
            10,
        )

        mavros_pose_qos = QoSProfile(depth=10)
        mavros_pose_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        mavros_pose_qos.durability = DurabilityPolicy.VOLATILE
        self._pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("local_pose_topic").value),
            self._on_local_pose,
            mavros_pose_qos,
        )

        self._det_sub = self.create_subscription(
            Detection2DArray,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )

        self._set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self._arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        self._start_time = time.monotonic()

        self._policy = None
        self._policy_hidden = None
        self._policy_available = False
        self._init_policy()

        self.create_timer(self._dt, self._tick)
        self.get_logger().info("Waiting for MAVROS services...")

    def _init_policy(self) -> None:
        if not bool(self.get_parameter("use_lnn").value):
            self._policy_available = False
            return

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            self.get_logger().warn("LNN enabled but model_path is empty; falling back to heuristic policy")
            self._policy_available = False
            return

        try:
            import torch

            self._torch = torch
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Failed to import torch ({e}); falling back to heuristic policy")
            self._policy_available = False
            return

        try:
            self._policy = self._torch.jit.load(model_path, map_location="cpu")
            self._policy.eval()
            self._policy_available = True
            self.get_logger().info(f"Loaded TorchScript policy: {model_path}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Failed to load TorchScript policy ({e}); falling back to heuristic policy")
            self._policy_available = False

    def _on_state(self, msg: State) -> None:
        self._state = msg
        self._have_state = True

    def _yaw_from_quat(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self._local_pose = msg
        if not self._have_local_pose:
            self._cmd_x = float(msg.pose.position.x)
            self._cmd_y = float(msg.pose.position.y)
            self._cmd_z = float(self._hold_z)
            self._yaw = self._yaw_from_quat(msg.pose.orientation)

            self._home_x = float(msg.pose.position.x)
            self._home_y = float(msg.pose.position.y)
            self._home_yaw = float(self._yaw)

            self._have_local_pose = True

    def _on_detections(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return

        best = None
        best_score = -1.0
        for det in msg.detections:
            score = 0.0
            if det.results:
                score = float(det.results[0].hypothesis.score)
            area = float(det.bbox.size_x) * float(det.bbox.size_y)
            metric = score + 1e-6 * area
            if metric > best_score:
                best_score = metric
                best = det

        if best is None:
            return

        cx = float(best.bbox.center.position.x)
        size_x = float(best.bbox.size_x)

        width = 1280.0
        if size_x > 1.0:
            width = max(width, cx * 2.0)

        cx_norm = (cx - (width / 2.0)) / (width / 2.0)
        cx_norm = max(-1.0, min(1.0, cx_norm))

        score = 0.0
        if best.results:
            score = float(best.results[0].hypothesis.score)

        self._last_det_time = time.monotonic()
        self._last_det_cx_norm = cx_norm
        self._last_det_score = score
        self._have_detection = True

    def _call_set_mode_offboard(self) -> None:
        if not self._set_mode_cli.wait_for_service(timeout_sec=0.0):
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "OFFBOARD"
        self._set_mode_cli.call_async(req)

    def _call_arm(self) -> None:
        if not self._arming_cli.wait_for_service(timeout_sec=0.0):
            return
        req = CommandBool.Request()
        req.value = True
        self._arming_cli.call_async(req)

    def _tick(self) -> None:
        if not self._have_local_pose or not self._have_state:
            return

        now = time.monotonic()
        pre_setpoints_sec = float(self.get_parameter("pre_setpoints_sec").value)
        if now - self._start_time < pre_setpoints_sec:
            self._publish_setpoint(search_mode=True)
            return

        if self._state.mode != "OFFBOARD":
            self._call_set_mode_offboard()
        if not self._state.armed:
            self._call_arm()

        self._publish_setpoint(search_mode=False)

    def _policy_step(self, have_fresh_det: bool) -> tuple[float, float, float]:
        """Returns (vx, vy, yaw_rate) in map frame."""
        max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        max_xy_speed = max(0.0, max_xy_speed)

        max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        policy_mode = str(self.get_parameter("policy_mode").value)
        allow_search = policy_mode == "search_and_track"

        if self._policy_available and (have_fresh_det or allow_search):
            # Inputs: [cx_norm, det_score, have_det, yaw_err, x_rel, y_rel]
            try:
                yaw_err = wrap_pi(self._yaw - self._home_yaw)
                x_rel = float(self._local_pose.pose.position.x) - float(self._home_x)
                y_rel = float(self._local_pose.pose.position.y) - float(self._home_y)

                x_in = self._torch.tensor(
                    [
                        float(self._last_det_cx_norm),
                        float(self._last_det_score),
                        1.0 if have_fresh_det else 0.0,
                        float(yaw_err),
                        float(x_rel),
                        float(y_rel),
                    ],
                    dtype=self._torch.float32,
                )
                y_out = self._policy(x_in)
                y_out = y_out.detach().cpu().numpy().reshape(-1)

                vx = float(y_out[0])
                vy = float(y_out[1])
                yaw_rate = float(y_out[2])
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Policy inference failed ({e}); falling back to heuristic")
                vx, vy, yaw_rate = 0.0, 0.0, 0.0
        else:
            # Heuristic: hover if detected; otherwise do small-radius search.
            if have_fresh_det:
                vx, vy = 0.0, 0.0
                if bool(self.get_parameter("track_yaw_on_detect").value):
                    kp = float(self.get_parameter("track_kp").value)
                    yaw_rate = -kp * float(self._last_det_cx_norm)
                else:
                    yaw_rate = 0.0
            else:
                v_xy = float(self.get_parameter("search_xy_speed").value)
                v_xy = max(0.0, v_xy)
                yaw_rate = float(self.get_parameter("search_yaw_rate").value)

                # simple tangential velocity around home
                dx = float(self._local_pose.pose.position.x) - float(self._home_x)
                dy = float(self._local_pose.pose.position.y) - float(self._home_y)
                r = math.hypot(dx, dy)
                if r < 1e-3:
                    vx, vy = 0.0, v_xy
                else:
                    tx, ty = -dy / r, dx / r
                    vx, vy = v_xy * tx, v_xy * ty

                # limit yaw within a cone
                if bool(self.get_parameter("search_yaw_bounce").value):
                    lim_deg = float(self.get_parameter("search_yaw_limit_deg").value)
                    lim_rad = max(0.0, math.radians(lim_deg))
                    err = wrap_pi(self._yaw - self._home_yaw)
                    if lim_rad > 1e-6:
                        if err >= lim_rad:
                            self._search_yaw_dir = -1.0
                        elif err <= -lim_rad:
                            self._search_yaw_dir = 1.0
                    yaw_rate *= float(self._search_yaw_dir)

        # Saturations
        v = math.hypot(vx, vy)
        if v > max_xy_speed and v > 1e-9:
            s = max_xy_speed / v
            vx *= s
            vy *= s
        yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))

        return vx, vy, yaw_rate

    def _publish_setpoint(self, search_mode: bool) -> None:
        timeout = float(self.get_parameter("detection_timeout_sec").value)
        now = time.monotonic()
        have_fresh_det = self._have_detection and (now - self._last_det_time) <= timeout

        if have_fresh_det and not self._hover_active:
            self._hover_active = True
            self._hover_x = float(self._local_pose.pose.position.x)
            self._hover_y = float(self._local_pose.pose.position.y)
        if (not have_fresh_det) and self._hover_active:
            self._hover_active = False

        if have_fresh_det != self._last_person_found:
            msg = Bool()
            msg.data = bool(have_fresh_det)
            self._person_found_pub.publish(msg)
            self._last_person_found = have_fresh_det

        vx, vy, yaw_rate = self._policy_step(have_fresh_det)

        alpha = float(self.get_parameter("yaw_rate_alpha").value)
        alpha = max(0.0, min(0.99, alpha))
        self._yaw_rate_filt = (alpha * self._yaw_rate_filt) + ((1.0 - alpha) * yaw_rate)
        yaw_rate = self._yaw_rate_filt

        self._yaw = wrap_pi(self._yaw + yaw_rate * self._dt)

        desired_x = float(self._cmd_x)
        desired_y = float(self._cmd_y)
        desired_z = float(self._hold_z)

        if have_fresh_det and self._hover_active:
            desired_x = float(self._hover_x)
            desired_y = float(self._hover_y)
        else:
            desired_x = float(self._cmd_x) + vx * self._dt
            desired_y = float(self._cmd_y) + vy * self._dt

        r_max = float(self.get_parameter("search_radius_max").value)
        desired_x, desired_y = clamp_xy_to_radius(desired_x, desired_y, self._home_x, self._home_y, r_max)

        vmax = float(self.get_parameter("pos_smooth_vmax").value)
        vmax = max(0.1, vmax)
        max_step = vmax * self._dt

        dx = desired_x - self._cmd_x
        dy = desired_y - self._cmd_y
        dist = math.hypot(dx, dy)
        if dist > max_step:
            scale = max_step / max(dist, 1e-9)
            dx *= scale
            dy *= scale

        self._cmd_x += dx
        self._cmd_y += dy
        self._cmd_z = float(desired_z)

        self._cmd_x, self._cmd_y = clamp_xy_to_radius(self._cmd_x, self._cmd_y, self._home_x, self._home_y, r_max)

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(self._cmd_x)
        sp.pose.position.y = float(self._cmd_y)
        sp.pose.position.z = float(self._cmd_z)
        sp.pose.orientation = yaw_to_quaternion(self._yaw)

        self._setpoint_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = CfcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except RCLError:
        pass
