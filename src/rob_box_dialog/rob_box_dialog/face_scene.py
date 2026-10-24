"""Сцена лица для LLM: enum-слоты, проверка кодом, одна JSON-схема (ADR-0150 §5.5.9, В15 (а); ADR-0148).

LLM не рисует пиксели и не пишет код: она заполняет узкую структуру из :data:`.knowledge.FACE_SCENE_SLOTS`, а
``face_node`` рендерит её. Структура одна на два вызывающих: ``Command.expression.face`` (v2) и
``play_animation(face=...)`` (v1). Невалидная сцена отбрасывается целиком, причина идёт в ``ExpressionEvent``
(``skipped{invalid_face}``) и в ответ тулу; остальная команда исполняется.

Чистый модуль, без ROS.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple

from . import knowledge as kn

#: Причина в ``ExpressionEvent.reason`` и в ответе тулу.
INVALID_REASON = "invalid_face"
#: Поля, которые вправе быть в сцене: слоты и длительность.
FIELDS: Tuple[str, ...] = (*kn.FACE_SCENE_SLOTS, "duration_s")

#: Одна JSON-схема сцены (каталог тулов и ``Command.expression.face``).
schema = kn.face_schema


@dataclass(frozen=True)
class FaceScene:
    """Сцена лица для LLM (§5.5.9): только enum-слоты ``knowledge.FACE_SCENE_SLOTS``; пусто — из рефлекса."""

    eyes: Optional[str] = None
    gaze: Optional[str] = None
    blink: Optional[str] = None
    mouth: Optional[str] = None
    tint: Optional[str] = None
    tempo: Optional[str] = None
    rear: Optional[str] = None
    bg: Optional[str] = None
    fx: Optional[str] = None
    feat: Optional[str] = None
    duration_s: float = kn.FACE_SCENE_DEFAULT_S

    def slots(self) -> Dict[str, str]:
        """Заданные слоты (без длительности), в порядке таблицы."""
        return {s: getattr(self, s) for s in kn.FACE_SCENE_SLOTS if getattr(self, s) is not None}

    def to_dict(self) -> Dict[str, Any]:
        """Слоты и ``duration_s`` - как их ждёт ``ExpressionRequest.face_json``."""
        return {**self.slots(), "duration_s": self.duration_s}


def problem(face: Any) -> str:
    """Что не так со сценой (``""`` - всё в порядке): не сцена, слот вне enum, длительность вне ``(0, 30]``."""
    if not isinstance(face, FaceScene):
        return "not_a_scene"
    for slot, allowed in kn.FACE_SCENE_SLOTS.items():
        value = getattr(face, slot)
        if value is not None and value not in allowed:
            return f"{slot}={value}"
    duration = face.duration_s
    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not 0 < duration <= kn.FACE_SCENE_MAX_S:
        return f"duration_s={duration}"
    return ""


def validate(data: Any) -> Tuple[Optional[FaceScene], str]:
    """Словарь (или его JSON-строка) -> ``(FaceScene, "")`` либо ``(None, причина)``; исключений не бросает.

    Неизвестное поле, значение вне enum, длительность вне ``(0, 30]``, пустая сцена - ``None``.
    """
    if isinstance(data, (str, bytes)):
        try:
            data = json.loads(data)
        except ValueError:
            return None, "not_json"
    if not isinstance(data, Mapping):
        return None, "not_an_object"
    unknown = sorted(str(k) for k in data if k not in FIELDS)
    if unknown:
        return None, f"unknown_field={unknown[0]}"
    kwargs = {k: data[k] for k in FIELDS if k in data}
    kwargs.setdefault("duration_s", kn.FACE_SCENE_DEFAULT_S)
    scene = FaceScene(**kwargs)
    why = problem(scene) or ("" if scene.slots() else "empty")
    return (None, why) if why else (scene, "")


__all__ = ["FIELDS", "INVALID_REASON", "FaceScene", "problem", "schema", "validate"]
