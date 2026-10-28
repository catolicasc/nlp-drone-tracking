"""ROS Image helpers without cv_bridge."""

from __future__ import annotations

import base64
import binascii
import struct
import zlib

from sensor_msgs.msg import Image


def rgb_bytes_from_image(msg: Image) -> bytes:
    encoding = (msg.encoding or "").lower()
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    data = bytes(msg.data)
    if width <= 0 or height <= 0:
        raise ValueError(f"dimensões inválidas: width={width} height={height}")

    if encoding in ("rgb8", "8uc3"):
        channels, mode = 3, "rgb"
    elif encoding == "bgr8":
        channels, mode = 3, "bgr"
    elif encoding in ("rgba8", "8uc4"):
        channels, mode = 4, "rgba"
    elif encoding == "bgra8":
        channels, mode = 4, "bgra"
    elif encoding in ("mono8", "8uc1"):
        channels, mode = 1, "mono"
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


def image_msg_to_bgr_numpy(msg: Image):
    import numpy as np

    width = int(msg.width)
    height = int(msg.height)
    rgb = rgb_bytes_from_image(msg)
    arr = np.frombuffer(rgb, dtype=np.uint8).reshape(height, width, 3)
    return arr[:, :, ::-1].copy()


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    crc = binascii.crc32(kind + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)


def png_bytes_from_image(msg: Image) -> bytes:
    width = int(msg.width)
    height = int(msg.height)
    rgb = rgb_bytes_from_image(msg)
    scanlines = b"".join(
        b"\x00" + rgb[row * width * 3 : (row + 1) * width * 3]
        for row in range(height)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(scanlines))
        + _png_chunk(b"IEND", b"")
    )


def png_data_url_from_image(msg: Image) -> str:
    return "data:image/png;base64," + base64.b64encode(png_bytes_from_image(msg)).decode("ascii")
