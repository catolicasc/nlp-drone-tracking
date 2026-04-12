import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError

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


def clamp_xy_to_annulus(
    x: float,
    y: float,
    cx: float,
    cy: float,
    r_min: float,
    r_max: float,
) -> tuple[float, float]:
    r_min = max(0.0, float(r_min))
    r_max = max(0.0, float(r_max))
    if r_max > 0.0 and r_min > r_max:
        r_min = r_max

    dx = x - cx
    dy = y - cy
    d = math.hypot(dx, dy)

    if r_max > 0.0 and d > r_max:
        s = r_max / max(d, 1e-9)
        return (cx + dx * s, cy + dy * s)
    if r_min > 0.0 and d < r_min:
        if d < 1e-9:
            return (cx + r_min, cy)
        s = r_min / d
        return (cx + dx * s, cy + dy * s)
    return (x, y)


class SearchAndTrackNode(Node):
    def __init__(self):
        super().__init__("search_and_track")

        self.declare_parameter("detections_topic", "/person_detector/detections")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_position/local")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("takeoff_z", 1.5)
        self.declare_parameter("pre_setpoints_sec", 2.5)

        self.declare_parameter("search_yaw_rate", 0.35)
        self.declare_parameter("track_kp", 0.6)
        self.declare_parameter("max_yaw_rate", 0.8)
        self.declare_parameter("detection_timeout_sec", 0.5)

        self.declare_parameter("search_radius_max", 50.0)
        self.declare_parameter("search_xy_speed", 2.0)
        self.declare_parameter("search_radius_growth_rate", 0.6)
        self.declare_parameter("search_min_radius", 0.0)
        self.declare_parameter("pos_smooth_vmax", 3.0)
        self.declare_parameter("yaw_rate_alpha", 0.85)

        self.declare_parameter("search_yaw_limit_deg", 45.0)
        self.declare_parameter("search_yaw_bounce", True)

        self.declare_parameter("track_yaw_on_detect", True)

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(self._rate_hz, 1e-3)

        self._state = State()
        self._have_state = False

        self._local_pose = PoseStamped()
        self._have_local_pose = False

        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_z = float(self.get_parameter("takeoff_z").value)
        self._yaw = 0.0

        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = float(self.get_parameter("takeoff_z").value)

        self._home_x = 0.0
        self._home_y = 0.0
        self._home_yaw = 0.0
        self._search_yaw_dir = 1.0
        self._search_theta = 0.0
        self._search_radius = float(self.get_parameter("search_min_radius").value)

        self._hover_active = False
        self._hover_x = 0.0
        self._hover_y = 0.0

        self._yaw_rate_filt = 0.0

        self._last_det_time = 0.0
        self._last_det_cx_norm = 0.0
        self._have_detection = False

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
        self._mode_requested = False
        self._arm_requested = False

        self.create_timer(self._dt, self._tick)

        self.get_logger().info("Waiting for MAVROS services...")

    def _on_state(self, msg: State) -> None:
        self._state = msg
        self._have_state = True

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self._local_pose = msg
        if not self._have_local_pose:
            self._hold_x = float(msg.pose.position.x)
            self._hold_y = float(msg.pose.position.y)
            self._yaw = self._yaw_from_quat(msg.pose.orientation)

            self._cmd_x = self._hold_x
            self._cmd_y = self._hold_y
            self._cmd_z = float(self._hold_z)
            self._home_x = self._hold_x
            self._home_y = self._hold_y
            self._home_yaw = float(self._yaw)
            self._have_local_pose = True

    def _yaw_from_quat(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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

        self._last_det_time = time.monotonic()
        self._last_det_cx_norm = cx_norm
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
        now = time.monotonic()

        if not self._have_local_pose or not self._have_state:
            return

        pre_setpoints_sec = float(self.get_parameter("pre_setpoints_sec").value)
        if now - self._start_time < pre_setpoints_sec:
            self._publish_setpoint(search_mode=True)
            return

        if self._state.mode != "OFFBOARD":
            self._call_set_mode_offboard()
        if not self._state.armed:
            self._call_arm()

        self._publish_setpoint(search_mode=False)

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

        max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        yaw_rate = 0.0

        if have_fresh_det:
            if bool(self.get_parameter("track_yaw_on_detect").value):
                kp = float(self.get_parameter("track_kp").value)
                yaw_rate = -kp * float(self._last_det_cx_norm)
                yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
            else:
                yaw_rate = 0.0
        else:
            if bool(self.get_parameter("search_yaw_bounce").value):
                lim_deg = float(self.get_parameter("search_yaw_limit_deg").value)
                lim_rad = max(0.0, math.radians(lim_deg))
                err = self._yaw - self._home_yaw
                while err > math.pi:
                    err -= 2.0 * math.pi
                while err < -math.pi:
                    err += 2.0 * math.pi

                if lim_rad > 1e-6:
                    if err >= lim_rad:
                        self._search_yaw_dir = -1.0
                    elif err <= -lim_rad:
                        self._search_yaw_dir = 1.0
                yaw_rate = float(self.get_parameter("search_yaw_rate").value) * float(self._search_yaw_dir)
            else:
                yaw_rate = float(self.get_parameter("search_yaw_rate").value)
            yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))

        alpha = float(self.get_parameter("yaw_rate_alpha").value)
        alpha = max(0.0, min(0.99, alpha))
        self._yaw_rate_filt = (alpha * self._yaw_rate_filt) + ((1.0 - alpha) * yaw_rate)
        yaw_rate = self._yaw_rate_filt

        self._yaw += yaw_rate * self._dt
        while self._yaw > math.pi:
            self._yaw -= 2.0 * math.pi
        while self._yaw < -math.pi:
            self._yaw += 2.0 * math.pi

        desired_x = float(self._cmd_x)
        desired_y = float(self._cmd_y)
        desired_z = float(self._hold_z)

        if not have_fresh_det:
            r_max = float(self.get_parameter("search_radius_max").value)
            v_xy = float(self.get_parameter("search_xy_speed").value)
            r_grow = float(self.get_parameter("search_radius_growth_rate").value)
            min_r = float(self.get_parameter("search_min_radius").value)

            r_max = max(0.0, r_max)
            v_xy = max(0.0, v_xy)
            r_grow = max(0.0, r_grow)
            min_r = max(0.0, min_r)

            self._search_radius = max(min_r, min(r_max, self._search_radius + r_grow * self._dt))
            denom_r = max(self._search_radius, 1.0)
            omega = 0.0 if v_xy <= 1e-6 else (v_xy / denom_r)
            self._search_theta += omega * self._dt

            desired_x = self._home_x + self._search_radius * math.cos(self._search_theta)
            desired_y = self._home_y + self._search_radius * math.sin(self._search_theta)
        else:
            if self._hover_active:
                desired_x = float(self._hover_x)
                desired_y = float(self._hover_y)
            self._search_theta = 0.0
            self._search_radius = float(self.get_parameter("search_min_radius").value)

        r_max = float(self.get_parameter("search_radius_max").value)
        r_min = float(self.get_parameter("search_min_radius").value)
        desired_x, desired_y = clamp_xy_to_annulus(desired_x, desired_y, self._home_x, self._home_y, r_min, r_max)

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

        self._cmd_x, self._cmd_y = clamp_xy_to_annulus(self._cmd_x, self._cmd_y, self._home_x, self._home_y, r_min, r_max)

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
    node = SearchAndTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except RCLError:
        pass


if __name__ == "__main__":
    main()
