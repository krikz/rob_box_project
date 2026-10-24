#!/usr/bin/env python3
"""Встреча лицом в v1 ``dialogue_node`` (ADR-0123 §3) — только обвязка ноды.

Правила и фразы перенесены в ``rob_box_dialog.meeting`` (#3912): одна реализация на v1 и v2 (хост v2 —
``core.dialog_v2_parity``). Здесь — :func:`greet_node_on_meeting` для ``DialogueNode`` (DSM, ``OccasionGate``);
модуль уходит вместе с v1 в PR-15.
"""

from __future__ import annotations

from typing import Any, Dict

from rob_box_dialog.meeting import (  # noqa: F401 — публичные имена v1 (dialogue_node, тесты) остаются здесь
    DEFAULT_LONG_ABSENCE_SEC,
    MEETING_SOURCE_COOLDOWN_SEC,
    MeetingGreeter,
    MeetingMarker,
    greet_on_meeting,
    parse_meeting_marker,
    time_of_day_greeting,
)


def greet_node_on_meeting(node: Any, marker: MeetingMarker) -> None:
    """Обвязка :func:`greet_on_meeting` для ``DialogueNode`` (держит ROS-часть вне класса, ADR-0145).

    Ошибка приветствия не должна ронять колбэк лица (ADR-0139 §2.5): логируем и идём дальше.
    """
    from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
    from rob_box_voice.core.greeting_window import open_window
    from rob_box_voice.core.occasion import Occasion, VerdictKind

    def allowed(m: MeetingMarker) -> bool:
        occasion = Occasion(kind='meeting', is_user_initiated=False, payload={
            'event_type': 'face', 'source_camera': m.source_camera, 'person_id': m.person_id, 'name': m.name})
        return node._occasion.may_speak(occasion).kind == VerdictKind.ALLOW

    spoken: Dict[str, str] = {}

    def speak(text: str) -> None:
        spoken['speech_id'] = node._publish_response(text, animation='happy')

    try:
        phrase = greet_on_meeting(
            node._meeting_greeter, marker,
            idle=node._dsm.current_state == DialogueStateKind.IDLE, allowed=allowed, speak=speak,
        )
        if phrase:
            # #3776: после приветствия окно 15 с — «привет» в ответ без «робот» (кадр ждёт конца речи).
            open_window(node, marker.person_id, marker.name.strip(), str(spoken.get('speech_id') or ''))
            node.get_logger().info(f'🗣 [встреча] {marker.name} (person={marker.person_id[:8]}): {phrase!r}')
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warning(f'⚠️ [встреча] приветствие упало: {exc!r}')
