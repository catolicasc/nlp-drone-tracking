"""YOLO person detection for Oracle Vision V2."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sensor_msgs.msg import Image

from .image_utils import image_msg_to_bgr_numpy

PERSON_CLASS_IDS = {0}


@dataclass
class PersonDetection:
    confidence: float
    bbox_xyxy: list[float]
    class_id: int
    class_name: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": round(self.confidence, 4),
            "bbox_xyxy": [round(v, 1) for v in self.bbox_xyxy],
        }


class YoloPersonDetector:
    def __init__(self, model_name: str = "yolov8n.pt", *, conf: float = 0.35) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics não instalado. Rode: ./scripts/install_yolo_deps.sh"
            ) from exc
        self._model = YOLO(model_name)
        self._conf = float(conf)
        self._model_name = model_name

    def detect(self, msg: Image) -> dict[str, Any]:
        try:
            bgr = image_msg_to_bgr_numpy(msg)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": f"falha ao converter imagem: {exc}"}

        results = self._model.predict(bgr, conf=self._conf, verbose=False)
        detections: list[PersonDetection] = []
        for result in results:
            names = result.names or {}
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls.item())
                if class_id not in PERSON_CLASS_IDS:
                    continue
                xyxy = [float(v) for v in box.xyxy[0].tolist()]
                detections.append(
                    PersonDetection(
                        confidence=float(box.conf.item()),
                        bbox_xyxy=xyxy,
                        class_id=class_id,
                        class_name=str(names.get(class_id, "person")),
                    )
                )

        detections.sort(key=lambda item: item.confidence, reverse=True)
        best = detections[0].to_dict() if detections else None
        return {
            "ok": True,
            "found": bool(detections),
            "count": len(detections),
            "best": best,
            "detections": [item.to_dict() for item in detections[:10]],
            "backend": "yolo",
            "model": self._model_name,
        }
