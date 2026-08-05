"""Standalone YOLO person detector node for Oracle Vision V2."""

from __future__ import annotations

import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .yolo_detector import YoloPersonDetector


class YoloPersonNode(Node):
    def __init__(self) -> None:
        super().__init__("v2_yolo_person_detector")
        self.declare_parameter("camera_topic", "/drone/camera/image_raw")
        self.declare_parameter("detections_topic", "/v2/yolo/detections")
        self.declare_parameter("model", os.environ.get("YOLO_MODEL", "yolov8n.pt"))
        self.declare_parameter("conf", float(os.environ.get("YOLO_CONF", "0.35")))

        try:
            self._detector = YoloPersonDetector(
                self.get_parameter("model").get_parameter_value().string_value,
                conf=self.get_parameter("conf").value,
            )
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise

        self._pub = self.create_publisher(
            String,
            self.get_parameter("detections_topic").get_parameter_value().string_value,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("camera_topic").get_parameter_value().string_value,
            self._on_image,
            10,
        )
        self.get_logger().info("YOLO person detector ativo")

    def _on_image(self, msg: Image) -> None:
        result = self._detector.detect(msg)
        out = String()
        out.data = json.dumps(result, ensure_ascii=False)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloPersonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
