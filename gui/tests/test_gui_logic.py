"""Testes da lógica pura da GUI (sem Qt e sem ROS em execução)."""

import numpy as np
import pytest

pytest.importorskip("sensor_msgs.msg")

from gui.frame_convert import image_msg_to_rgb  # noqa: E402
from gui.status_format import format_status_event  # noqa: E402


def _make_image(width, height, encoding, channels=None, step=None):
    from sensor_msgs.msg import Image

    if channels is None:
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}[encoding]
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    msg.step = step if step is not None else width * channels
    msg.data = bytes(range(256))[: msg.step * height].ljust(msg.step * height, b"\x01")
    return msg


class TestFrameConvert:
    def test_rgb8_shape(self):
        msg = _make_image(4, 3, "rgb8")
        frame = image_msg_to_rgb(msg)
        assert frame is not None
        assert frame.shape == (3, 4, 3)
        assert frame.dtype == np.uint8

    def test_bgr8_swaps_channels(self):
        msg = _make_image(1, 1, "bgr8")
        msg.data = bytes([10, 20, 30])
        msg.step = 3
        frame = image_msg_to_rgb(msg)
        assert list(frame[0, 0]) == [30, 20, 10]

    def test_mono8_replicates_gray(self):
        msg = _make_image(2, 2, "mono8")
        msg.data = bytes([200] * 4)
        msg.step = 2
        frame = image_msg_to_rgb(msg)
        assert frame.shape == (2, 2, 3)
        assert (frame == 200).all()

    def test_unsupported_encoding_returns_none(self):
        msg = _make_image(2, 2, "rgb8")
        msg.encoding = "bayerrggb8"
        assert image_msg_to_rgb(msg) is None

    def test_step_padding_is_discarded(self):
        msg = _make_image(2, 2, "rgb8", step=2 * 3 + 1)  # 1 byte de padding/linha
        frame = image_msg_to_rgb(msg)
        assert frame.shape == (2, 2, 3)


class TestFormatStatusEvent:
    def test_vlm_step(self):
        assert "1/25" in format_status_event({"event": "vlm_step", "step": 1, "max_steps": 25})

    def test_tool_result_truncates(self):
        line = format_status_event({"event": "tool_result", "tool": "takeoff", "result": "x" * 500})
        assert line.startswith("tool takeoff")
        assert len(line) < 200

    def test_error_falls_back_to_payload(self):
        assert "boom" in format_status_event({"event": "error", "detail": "boom"})
