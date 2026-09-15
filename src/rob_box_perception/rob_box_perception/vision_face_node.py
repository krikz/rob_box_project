#!/usr/bin/env python3
"""vision_face_node — детекция лица на AI HAT+ (RetinaFace, ADR-0089 Phase 2).

Сабкласс ``VisionHailoNode``: переиспользует всю инфраструктуру ноды
(шов «Взгляд» gaze.py, поллинг кадров, degraded-state policy, публикацию
``VisionEvent``), меняя только две вещи:

  1. HEF loader — ``make_face_loader`` (RetinaFace вместо YOLOv8n),
  2. имя ноды и дефолтный confidence threshold.

Публикует ``VisionEvent`` с ``event_type="face"`` + bbox на тот же топик
``/vision/hailo/events`` (issue #2599 acceptance: face-события видны в
``ros2 topic echo /vision/hailo/events``). ``embedding_id`` /
``display_name`` остаются пустыми — это PR-B (ArcFace).

PR-A scope: только детекция. Эмбеддинги не считаются, БД нет →
privacy-review не требуется.

Stub-режим (hailo_enabled=false): ``StubHEFLoader`` публикует
``event_type="stub"`` (маркер выдуманного события, ADR-0089 §2.2) —
лицевой продюсер обязан играть по тем же правилам, что и person-нода:
выдумка не должна доезжать до Личности.

Touchpoints:
- ADR-0089 §2.1 Phase 2 (vision_face_node).
- Issue #2599 PR-A.
- rob_box_perception.vision_hailo_node (базовый класс).
- rob_box_perception.vision_face_loader (RetinaFaceLoader, make_face_loader).
"""

from __future__ import annotations

from typing import List, Optional

from rob_box_perception.vision_face_loader import make_face_loader
from rob_box_perception.vision_hailo_loader import HEFLoader
from rob_box_perception.vision_hailo_node import VisionHailoNode


class VisionFaceNode(VisionHailoNode):
    """Detect-лица нода (RetinaFace) на том же контракте, что vision_hailo."""

    NODE_NAME: str = 'vision_face'

    #: RetinaFace confidence threshold (hailo_models.yaml: 0.6).
    DEFAULT_CONFIDENCE_THRESHOLD: float = 0.6

    def _make_loader(self) -> HEFLoader:
        return make_face_loader(
            hailo_enabled=self.hailo_enabled,
            hef_path=self.hef_path,
            stub_period_sec=self.stub_period_sec,
            confidence_threshold=self.confidence_threshold,
            nms_iou_threshold=self.nms_iou_threshold,
        )


def main(args: Optional[List[str]] = None) -> None:
    """Entrypoint console_script (см. setup.py)."""
    import rclpy

    rclpy.init(args=args)
    node = VisionFaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
