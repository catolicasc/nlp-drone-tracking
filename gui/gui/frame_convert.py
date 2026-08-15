"""Conversão de sensor_msgs/Image para array RGB numpy (sem depender de Qt)."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image


def image_msg_to_rgb(msg: Image) -> np.ndarray | None:
    """Retorna frame RGB uint8 (h, w, 3) ou None se o encoding não for suportado."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    w, h = msg.width, msg.height
    if w == 0 or h == 0 or data.size == 0:
        return None

    enc = msg.encoding.lower()
    # msg.step pode incluir padding por linha; descartamos ao final
    if enc == "rgb8":
        return data.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
    if enc == "bgr8":
        return data.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)[:, :, ::-1]
    if enc == "rgba8":
        return data.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
    if enc == "bgra8":
        return data.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, 2::-1]
    if enc in ("mono8", "8uc1"):
        gray = data.reshape(h, msg.step)[:, :w]
        return np.stack([gray] * 3, axis=-1)
    # encodings não suportados (ex. bayer) são ignorados
    return None
