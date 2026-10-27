"""Тулы хода v1 глазами рефлексов: ``executing{cls}``, эмоция из ``animation``, ошибки тулов (ADR-0150 §5.5.3, PR-3b #3690).

:class:`ToolWatch` смотрит на вызов тула до ``/mcp/execute`` и на его итог после. Тул лицо не трогает: имя сцены из аргумента
``animation`` (``speak_text`` / ``play_animation``) видит диалог, красит ею рефлексы ``speaking``/``done``, а
``play_animation`` показывает сцену сразу и на заказанное время (правило длительности - одно, у тула).
Без ROS; словарь анимаций тулов подгружается лениво (пакета может не быть в образе).
"""

from __future__ import annotations

import json
from dataclasses import replace
from typing import Any, Callable, Dict, Optional

from rob_box_dialog import expression as ex
from rob_box_dialog import face_scene as fs
from rob_box_dialog import knowledge as kn

from .turn_reflexes import ReflexOut, Turn

#: Тулы речи: транспорт, а не действие (``knowledge.SPEECH_CHANNEL_TOOLS``) - стадию ``executing`` не открывают.
SPEECH_TOOLS = kn.SPEECH_CHANNEL_TOOLS
SHOW_TOOL = kn.SHOW_TOOL
#: Сколько символов причины отказа тула хранить (``TurnEvent.reason``).
REASON_MAX = 60


def scene_for_animation(name: str) -> str:
    """Имя анимации тулов речи -> сцена лица (``""`` - неизвестно); псевдонимы приводит общий словарь тулов."""
    try:
        from rob_box_mcp_tools.animations import normalize_animation  # noqa: PLC0415 - пакет есть не в каждом образе
    except ImportError:
        return ""
    return normalize_animation(name, fallback="")


def show_seconds(duration: Any) -> float:
    """Сколько держать сцену ``play_animation``: правило тула (``clamp_show_seconds``); ``0`` - по умолчанию лица."""
    if duration is None:
        return 0.0
    try:
        from rob_box_mcp_tools.animations import clamp_show_seconds  # noqa: PLC0415 - пакет есть не в каждом образе
        return clamp_show_seconds(float(duration)) or 0.0
    except (ImportError, TypeError, ValueError):
        return 0.0


def face_dict(raw: Any, seconds: float = 0.0) -> Optional[Dict[str, Any]]:
    """Сцена лица из аргумента ``face`` тула -> ``ExpressionRequest.face_json``; ``None`` - нет или невалидна.

    Проверка одна, с лицом (``face_scene.validate``); заказанная ``duration`` тула важнее ``duration_s`` сцены.
    """
    if raw is None:
        return None
    face, _why = fs.validate(raw)
    if face is None:
        return None
    return replace(face, duration_s=seconds).to_dict() if seconds > 0 else face.to_dict()


def error_reason(content: Any) -> str:
    """Короткая причина отказа тула из тела результата: ``error`` / ``message`` JSON-ответа, иначе начало текста."""
    text = str(content or "")
    try:
        body = json.loads(text)
    except ValueError:
        body = None
    if isinstance(body, dict):
        text = str(body.get("error") or body.get("message") or text)
    return " ".join(text.split())[:REASON_MAX]


class ToolWatch:
    """Вызовы и итоги тулов хода -> стадия ``executing``, эмоция хода, причины ``failed``."""

    def __init__(self, out: ReflexOut, normalize: Callable[[str], str]) -> None:
        self._out, self._normalize = out, normalize

    def call(self, turn: Turn, name: str, args: Dict[str, Any]) -> None:
        """Перед ``/mcp/execute``: ``executing{cls}``; ``speak_text`` обещает речь, ``animation`` красит ход."""
        if name in SPEECH_TOOLS:
            turn.expects_speech = True
            turn.emotion = self._normalize(str(args.get("animation") or "")) or turn.emotion
        elif name == SHOW_TOOL:
            self._show(turn, args)
        elif name in kn.TOOL_CLASSES:
            turn.cls = kn.TOOL_CLASSES[name]
            self._out.enter(turn, "executing", {"direction": args.get("direction")})

    def result(self, turn: Turn, name: str, is_error: bool, reason: str) -> None:
        """Итог тула: ошибка действия - причина ``failed``; успех того же тула позже её снимает."""
        if name not in kn.TOOL_CLASSES:
            return
        if is_error:
            turn.errors[name] = (reason or "error")[:REASON_MAX]
        else:
            turn.errors.pop(name, None)

    def _show(self, turn: Turn, args: Dict[str, Any]) -> None:
        """``play_animation``: показать сцену и/или сцену лица LLM сразу и на заказанное время (до конца хода сцена красит ``done``)."""
        scene = self._normalize(str(args.get("animation") or ""))
        face = face_dict(args.get("face"), show_seconds(args.get("duration")))
        if scene:
            turn.emotion = scene
        if scene or face:
            self._out.send(turn, ex.ExpressionPlan(
                emotion=scene, ttl_ms=int(show_seconds(args.get("duration")) * 1000), face=face))


__all__ = ["REASON_MAX", "SHOW_TOOL", "SPEECH_TOOLS", "ToolWatch", "error_reason", "face_dict", "scene_for_animation",
           "show_seconds"]
