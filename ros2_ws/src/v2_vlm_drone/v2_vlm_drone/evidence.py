"""Search evidence storage for Oracle Vision V2."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from sensor_msgs.msg import Image

from .image_utils import png_bytes_from_image


def save_search_evidence(
    image: Image,
    metadata: dict[str, Any],
    *,
    base_dir: Path,
    label: str = "detection",
) -> dict[str, str]:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = base_dir / f"search_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    image_path = run_dir / f"{label}.png"
    meta_path = run_dir / f"{label}.json"
    image_path.write_bytes(png_bytes_from_image(image))
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"run_dir": str(run_dir), "image_path": str(image_path), "meta_path": str(meta_path)}
