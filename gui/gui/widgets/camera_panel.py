"""Painel de câmera: exibe frames de /drone/camera/image_raw."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QGroupBox, QLabel, QVBoxLayout, QWidget

CAM_FPS_LIMIT_MS = 66  # ~15 fps para não saturar a thread de UI


class CameraPanel(QGroupBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Câmera (/drone/camera/image_raw)", parent)
        layout = QVBoxLayout(self)
        self._view = QLabel("sem imagem")
        self._view.setAlignment(Qt.AlignCenter)
        self._view.setMinimumSize(480, 270)
        self._view.setStyleSheet("background: #111; color: #666;")
        layout.addWidget(self._view, 1)

    def on_frame(self, rgb: np.ndarray) -> None:
        h, w, ch = rgb.shape
        image = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
        pix = QPixmap.fromImage(image).scaled(
            self._view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self._view.setPixmap(pix)
