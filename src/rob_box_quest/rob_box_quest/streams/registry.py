"""Registry доступных стримов rob_box_quest.

Источник истины: docs/architecture/meta-quest-api.md §4 (topic_id map)
+ :mod:`rob_box_core.bridge_protocol` (single source of truth, voice-vr 07
/ issue #2192). ``STREAM_CATALOG`` и :class:`StreamSpec` тут — re-export
из каталога; локальный :class:`StreamKind` оставлен как backward-compat
для кода, который его импортирует (Phase 1.4 v2 коды «ros_topic» /
«camera_direct»).

Стримы делятся на 2 класса:
- ROS2 стримы (lidar_2d, lidar_3d, map_2d, robot_status, voice_state,
  person_detections, camera_rear, camera_front, camera_ceiling): идут
  через Zenoh, payload формируется в protocol/topics.py и streams/*.
- Камеры мимо ROS (camera_oak_color, camera_oak_depth): читаются
  CameraProvider'ом в Vision Pi напрямую (depthai SDK), payload =
  JPEG/H.264 bytes.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Optional

# [voice-vr 07] / issue #2192: ``STREAM_CATALOG`` и ``StreamSpec`` —
# re-export из rob_box_core.bridge_protocol. Раньше локально объявлялся
# dataclass-каталог из 9 записей, что расходилось с ``protocol/topics.py``
# (только topic_id) и не включало ``camera_front`` / ``lidar_3d`` /
# ``person_detections`` (последние — Phase 2). Канон — bridge_protocol.
from rob_box_core.bridge_protocol import (
    STREAMS as _BRIDGE_STREAMS,
)
from rob_box_core.bridge_protocol import (
    StreamKind,
    StreamSpec,
    get_stream,
    get_stream_by_topic_id,
)


# Read-only dict-view канона, совместимый со старым контрактом
# ``dict[str, StreamSpec]``. Код, который делает ``STREAM_CATALOG["x"]
# .source``, продолжает работать; присваивание в каталог теперь упадёт
# (этого и нельзя было делать, см. ADR-0080 §2.2 инвариант 3).
STREAM_CATALOG: "MappingProxyType[str, StreamSpec]" = MappingProxyType(
    {s.ui_name: s for s in _BRIDGE_STREAMS}
)


def list_streams() -> list[StreamSpec]:
    """Все стримы — для stream_select list cmd (Phase 2 R10)."""
    return list(STREAM_CATALOG.values())


def topic_id_for(ui_name: str) -> Optional[int]:
    """Shortcut: ``StreamSpec.topic_id`` или ``None``."""
    spec = STREAM_CATALOG.get(ui_name)
    return spec.topic_id if spec else None
