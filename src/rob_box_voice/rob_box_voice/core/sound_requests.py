"""Запрос звука -> запуск или честный пропуск, с событием на каждый исход (ADR-0150 §5.5.2, V33, PR-3b #3690).

Раньше ``sound_node`` молча делал ``return``, если канал занят («Звук уже играет (button_click), пропускаю thinking»,
живой лог 20.10): человек не слышал «услышал», а код об этом не знал. Теперь запрос от ``play_sound``
(``/voice/sound/trigger``) и earcon рефлекса (``ExpressionRequest.earcon``) идут одним путём, и у каждого есть
``SoundEvent``: ``started`` -> ``finished``, либо ``skipped{busy|voice_stream|not_found|error}``.

Миксин ``SoundNode``: ждёт у хозяина ``_voice_stream``, ``is_playing``, ``current_sound``, ``_play_lock``,
``select_sound()``, ``play_sound_thread()``, издателей ``sound_event_pub`` / ``expression_event_pub`` и логгер.
"""

from __future__ import annotations

import json
import threading
import uuid
from typing import Dict, Tuple

from rob_box_dialog_msgs.msg import ExpressionEvent, ExpressionRequest, SoundEvent

#: Earcon рефлексов (``rob_box_dialog.knowledge.EARCONS``) -> файл звукового пакета; знание о звуках живёт здесь.
EARCON_SOUNDS: Dict[str, str] = {"heard": "button_click", "done": "ui_confirm", "fail": "error"}
#: Причины пропуска звука (``SoundEvent.reason``).
SKIP_BUSY, SKIP_VOICE_STREAM, SKIP_NOT_FOUND, SKIP_ERROR = "busy", "voice_stream", "not_found", "error"


def parse_trigger(data: str) -> Tuple[str, str]:
    """``/voice/sound/trigger``: ``{"sound", "request_id"}`` от ``play_sound`` -> ``(звук, id)``.

    Голое имя звука (ручной ``ros2 topic pub``, сценарный раннер E2E) - тоже запрос, id выдаёт владелец.
    """
    text = (data or "").strip()
    if text.startswith("{"):
        try:
            body = json.loads(text)
        except ValueError:
            body = None
        if isinstance(body, dict):
            return str(body.get("sound") or "").strip(), str(body.get("request_id") or "") or uuid.uuid4().hex[:8]
    return text, uuid.uuid4().hex[:8]


class SoundRequestsMixin:
    """Решение «играть или пропустить» и события исхода."""

    def start_sound(self, trigger: str, request_id: str) -> str:
        """Запустить звук по триггеру; вернуть ``""`` (играет) или причину пропуска - она же уходит в ``SoundEvent``."""
        # Рация: голос оператора приоритетнее эффектов — не перебиваем стрим.
        if self._voice_stream is not None:
            return self._skip(request_id, trigger, SKIP_VOICE_STREAM)
        sound_name = self.select_sound(trigger)
        if sound_name is None:
            return self._skip(request_id, trigger, SKIP_NOT_FOUND)
        with self._play_lock:
            if self.is_playing:
                return self._skip(request_id, trigger, SKIP_BUSY)
            # Занято с этой секунды, а не с первой строки потока: два earcon'а подряд не пройдут оба.
            self.is_playing = True
            self.current_sound = sound_name
        self.play_thread = threading.Thread(
            target=self.play_sound_thread, args=(sound_name, trigger, request_id), daemon=True
        )
        self.play_thread.start()
        return ""

    def expression_callback(self, msg: ExpressionRequest) -> None:
        """Earcon рефлекса хода: звук по ``EARCON_SOUNDS``; исход - ``SoundEvent`` и ``ExpressionEvent{sound}``."""
        if not msg.earcon:
            return
        self.get_logger().info(f"🔔 Earcon: {msg.earcon} (turn {msg.turn_id[:8]})")
        sound = EARCON_SOUNDS.get(msg.earcon)
        reason = self.start_sound(sound, msg.turn_id or uuid.uuid4().hex[:8]) if sound else SKIP_NOT_FOUND
        event = ExpressionEvent()
        event.executor, event.turn_id, event.reason = "sound", msg.turn_id, reason
        event.kind = ExpressionEvent.KIND_SKIPPED if reason else ExpressionEvent.KIND_STARTED
        self.expression_event_pub.publish(event)

    def _skip(self, request_id: str, trigger: str, reason: str) -> str:
        self.get_logger().warn(f'⚠️ Звук "{trigger}" пропущен: {reason} (играет: {self.current_sound})')
        self.publish_sound_event(request_id, SoundEvent.KIND_SKIPPED, trigger, reason)
        return reason

    def publish_sound_event(self, request_id: str, kind: int, sound: str, reason: str = "") -> None:
        event = SoundEvent()
        event.request_id, event.kind, event.sound, event.reason = request_id, kind, sound or "", reason
        self.sound_event_pub.publish(event)
