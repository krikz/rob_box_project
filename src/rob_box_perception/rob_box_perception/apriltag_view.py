"""apriltag_view.py - какие AprilTag-метки видны сейчас (issue #4352).

apriltag_node публикует ``/detections`` ~5 Гц (замер 04.11.2026: ``average rate: 4.996``).
Метка считается видимой, пока приходит; ушла из кадра - через ``FRESH_S`` секунд её нет.

Чистый Python, без rclpy.
"""

from __future__ import annotations

from typing import Iterable, List, Optional

#: Метка видна, если её детекция пришла не раньше стольких секунд назад.
FRESH_S = 2.0


class VisibleTags:
    """Последний набор id меток и время его прихода."""

    def __init__(self) -> None:
        """Пустой набор: меток не видно."""
        self._ids: List[int] = []
        self._seen_at: Optional[float] = None

    def see(self, ids: Iterable[int], now: float) -> bool:
        """Кадр детектора. True - набор видимых меток сменился (стоит записать в память)."""
        current = sorted({int(i) for i in ids})
        if not current:
            return False
        changed = current != self.visible(now)
        self._ids = current
        self._seen_at = now
        return changed

    def visible(self, now: float) -> List[int]:
        """Id меток, видимых на момент ``now``."""
        if self._seen_at is None or now - self._seen_at > FRESH_S:
            return []
        return list(self._ids)
