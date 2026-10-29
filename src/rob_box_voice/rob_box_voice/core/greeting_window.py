"""Окно ответа после приветствия при встрече (``core/meeting.py``; #3776, решение Шифу 21.10, эпик #3683).

Приветствие ушло в речь — кадр ``GreetingReply`` ждёт его конца (``SpeechEvent finished`` этой речи); окно
``REPLY_WINDOW_S`` (одна константа с вопросом об имени) открывается после конца речи + грейс микрофона и снимается
первой адресованной голосовой фразой или по TTL. В окне фраза без вейка от приветствованного — ход
(``address.GREETING_REPLY``, правило — ``speaker_adapter.answer_to_greeting``); иначе — бэклог.
Функции, а не методы: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

from typing import Any, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.speaker_adapter import greeting_frame, greeting_open
from rob_box_voice.core.identity_question import session

KIND = "GreetingReply"
#: ``SpeechEvent.kind``: finished / cancelled / rejected.
_FINISHED, _CANCELLED, _REJECTED = 4, 5, 7


def open_window(node: Any, person_ref: str, name: str, speech_id: str) -> None:
    """Приветствие отдано в речь (``speech_id``): кадр ждёт конца речи. Нет ``speech_id`` — окна нет."""
    if not speech_id:
        return
    current = session(node)
    current.pop_scene(KIND)
    current.push_scene(KIND, "code", ttl_s=float(kn.IDENTITY_WAIT_QUESTION_S),
                       payload=greeting_frame(person_ref, name, speech_id))
    node.get_logger().info(f"💬 [reply-window] greeting {name}: кадр ждёт конца приветствия speech_id={speech_id[:8]}")


def on_speech_event(node: Any, kind: int, speech_id: str, agent: str) -> None:
    """Приветствие договорено — открыть окно; отменено/отклонено — снять кадр."""
    if agent != "personality" or not speech_id or kind not in (_FINISHED, _CANCELLED, _REJECTED):
        return
    current = session(node)
    frame = current.snapshot.top(KIND)
    if frame is None or frame.payload.get("speech_id") != speech_id or greeting_open(frame):
        return
    name = frame.payload.get("name")
    if kind != _FINISHED:
        current.pop_scene(KIND)
        node.get_logger().info(f"💬 [reply-window] greeting {name}: кадр снят, приветствие не прозвучало (kind={kind})")
        return
    current.arm_scene(KIND, float(kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S),
                      {"asked_at": current.now() + kn.IDENTITY_MIC_GRACE_S})
    node.get_logger().info(
        f"💬 [reply-window] greeting {name}: окно открыто ttl={kn.REPLY_WINDOW_S}s "
        f"(микрофон после речи +{kn.IDENTITY_MIC_GRACE_S}s)")


def window(node: Any) -> Optional[Any]:
    """Кадр ``GreetingReply`` с открытым окном (истёкшие сессия снимает) или ``None``."""
    current = session(node)
    current.expire()
    frame = current.snapshot.top(KIND)
    return frame if greeting_open(frame) else None


def take_reply(node: Any, source: str, heard: Any) -> None:
    """Адресованная голосовая фраза при открытом окне снимает кадр (первая же фраза, как у вопроса об имени)."""
    frame = window(node)
    if frame is None or (getattr(heard, "source", "") or "respeaker") != "respeaker":
        return
    session(node).pop_scene(KIND)
    node.get_logger().info(
        f"💬 [reply-window] greeting {frame.payload.get('name')}: принят source={source} "
        f"text={str(getattr(heard, 'text', ''))[:60]!r}")


def not_a_reply(node: Any, heard: Any) -> None:
    """Окно открыто, а фраза без вейка не ответ (грамматика не та, лица приветствованного нет в кадре) — в бэклог."""
    frame = window(node)
    if frame is not None:
        node.get_logger().info(
            f"💬 [reply-window] greeting {frame.payload.get('name')}: не ответ → бэклог "
            f"faces={len(getattr(heard, 'faces', ()) or ())} text={str(getattr(heard, 'text', ''))[:60]!r}")
