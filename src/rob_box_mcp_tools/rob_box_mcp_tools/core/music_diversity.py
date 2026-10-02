"""music_diversity.py — реэкспорт :mod:`rob_box_music.diversity` (ADR-0149 §8.1, PR-3d).

``weighted_pick`` и ``MusicHistory`` перенесены в чистый пакет ``rob_box_music``: одна реализация на
v1 (club, ADR-0146) и v2. Модуль удаляется вместе со старым путём (ADR-0149 PR-13…15).
"""

from rob_box_music.diversity import (  # noqa: F401 — реэкспорт для старого пути
    DEFAULT_DECAY,
    DEFAULT_FLOOR,
    HISTORY_FIELDS,
    MusicHistory,
    weighted_pick,
)

__all__ = ["DEFAULT_DECAY", "DEFAULT_FLOOR", "HISTORY_FIELDS", "MusicHistory", "weighted_pick"]
