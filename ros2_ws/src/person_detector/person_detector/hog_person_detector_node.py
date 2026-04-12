import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D


class HogPersonDetectorNode(Node):
    def __init__(self):
        super().__init__("hog_person_detector")

        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("detections_topic", "/person_detector/detections")
        self.declare_parameter("debug_image_topic", "/person_detector/debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("hit_threshold", 0.0)
        self.declare_parameter("scale", 1.05)
        self.declare_parameter("group_threshold", 2)

        self._bridge = CvBridge()

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value

        self._pub = self.create_publisher(Detection2DArray, detections_topic, 10)

        self._publish_debug = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        self._debug_pub = None
        if self._publish_debug:
            debug_topic = self.get_parameter("debug_image_topic").get_parameter_value().string_value
            self._debug_pub = self.create_publisher(Image, debug_topic, 10)

        self._sub = self.create_subscription(Image, image_topic, self._on_image, 10)

        import cv2

        self._cv2 = cv2
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Publishing detections: {detections_topic}")
        if self._publish_debug:
            self.get_logger().info(
                f"Publishing debug image: {self.get_parameter('debug_image_topic').value}"
            )

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        hit_threshold = float(self.get_parameter("hit_threshold").value)
        scale = float(self.get_parameter("scale").value)
        group_threshold = int(self.get_parameter("group_threshold").value)

        try:
            rects, weights = self._hog.detectMultiScale(
                frame,
                hitThreshold=hit_threshold,
                winStride=(8, 8),
                padding=(8, 8),
                scale=scale,
                finalThreshold=group_threshold,
            )
        except Exception:
            rects, weights = self._hog.detectMultiScale(
                frame,
                hitThreshold=hit_threshold,
                winStride=(8, 8),
                padding=(8, 8),
                scale=scale,
            )

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        for (x, y, w, h), weight in zip(rects, weights):
            det = Detection2D()
            det.header = msg.header

            bbox = BoundingBox2D()
            bbox.center.position.x = float(x + w / 2.0)
            bbox.center.position.y = float(y + h / 2.0)
            bbox.size_x = float(w)
            bbox.size_y = float(h)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "person"
            hyp.hypothesis.score = float(weight)
            det.results.append(hyp)

            det_arr.detections.append(det)

            if self._publish_debug and self._debug_pub is not None:
                self._cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
                label = f"person {float(weight):.2f}"
                self._cv2.putText(
                    frame,
                    label,
                    (int(x), max(0, int(y) - 5)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    self._cv2.LINE_AA,
                )

        self._pub.publish(det_arr)

        if self._publish_debug and self._debug_pub is not None:
            try:
                dbg_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                dbg_msg.header = msg.header
                self._debug_pub.publish(dbg_msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"debug publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HogPersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
