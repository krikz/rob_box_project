"""Окно ответа после приветствия при встрече (``rob_box_dialog.meeting``; #3776, решение Шифу 21.10, эпик #3683).

Приветствие ушло в речь — кадр ``GreetingReply`` ждёт его конца (``SpeechEvent finished`` этой речи); окно
``REPLY_WINDOW_S`` (одна константа с вопросом об имени) открывается после конца речи + грейс микрофона и снимается
первой адресованной голосовой фразой или по TTL. В окне фраза без вейка — ход (``address.GREETING_REPLY``, правило —
``speaker_adapter.answer_to_greeting``); иначе — бэклог. Голое «робот» в окне (``address.GREETING_WAKE``, #4509) окно
не снимает, а открывает заново: робот говорит «Слушаю.» и ждёт ответа.

Окно ответа знает и ``stt_node`` (#4509): владелец окна — этот модуль — шлёт событие ``echo_window.REPLY_WINDOW_TOPIC``
(сколько секунд окно открыто, ``0`` — закрыто), и фраза из эхо-окна после приветствия не выбрасывается, а сверяется
словами с речью робота. Публикатор события — ``node.announce_reply_window`` (его заводит ``dialog_node``; у ноды из
тестов его нет — событие не шлётся).
Функции, а не методы: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

from typing import Any, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import GREETING_WAKE
from rob_box_dialog.session import SceneFrame
from rob_box_dialog.speaker_adapter import greeting_frame, greeting_open
from rob_box_dialog.window_reset import reset_if_idle
from rob_box_voice.core.identity_question import session

KIND = "GreetingReply"
#: ``SpeechEvent.kind``: finished / cancelled / rejected.
_FINISHED, _CANCELLED, _REJECTED = 4, 5, 7
#: Сколько живёт открытое окно: грейс микрофона после речи + само окно ответа.
_OPEN_TTL_S = float(kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S)


def _announce(node: Any, seconds: float) -> None:
    """Событие окна ответа для ``stt_node`` (#4509): открыто ещё ``seconds``, ``0`` — закрыто."""
    announce = getattr(node, "announce_reply_window", None)
    if announce is not None:
        announce(float(seconds))


def _arm(node: Any, spoken_text: Optional[str] = None) -> Optional[SceneFrame]:
    """Открыть окно ответа сейчас: кадр с ``asked_at`` после грейса микрофона и событие для ``stt_node``."""
    current = session(node)
    payload: dict[str, Any] = {"asked_at": current.now() + kn.IDENTITY_MIC_GRACE_S}
    if spoken_text is not None:
        payload["text"] = spoken_text
    frame = current.arm_scene(KIND, _OPEN_TTL_S, payload)
    if frame is not None:
        _announce(node, _OPEN_TTL_S)
    return frame


def open_window(node: Any, person_ref: str, name: str, speech_id: str, text: str = "") -> None:
    """Приветствие отдано в речь (``speech_id``): кадр ждёт конца речи. Нет ``speech_id`` — окна нет."""
    if not speech_id:
        return
    current = session(node)
    current.pop_scene(KIND)
    payload = greeting_frame(person_ref, name, speech_id)
    payload["text"] = text
    current.push_scene(KIND, "code", ttl_s=float(kn.IDENTITY_WAIT_QUESTION_S),
                       payload=payload)
    node.get_logger().info(f"💬 [reply-window] greeting {name}: кадр ждёт конца приветствия speech_id={speech_id[:8]}")


def _record_spoken_greeting(node: Any, speech_id: str, text: str, epoch: int) -> None:
    """Подтверждённая реплика робота занимает отдельный ход в обычной истории."""
    engine = node._engine
    if epoch != engine.session.snapshot.epoch:
        return
    engine.log.begin(speech_id, epoch, "")
    engine.log.said(speech_id, speech_id, text, delivered=True)


def on_speech_event(node: Any, kind: int, speech_id: str, agent: str, spoken_chars: int = 0) -> None:
    """Приветствие договорено — открыть окно; отменено/отклонено — снять кадр."""
    if agent != "personality" or not speech_id or kind not in (_FINISHED, _CANCELLED, _REJECTED):
        return
    current = session(node)
    snapshot = current.snapshot
    frame = snapshot.top(KIND)
    if frame is None or frame.payload.get("speech_id") != speech_id or greeting_open(frame):
        return
    if frame.ttl_until is not None and frame.ttl_until <= current.now():
        current.pop_scene(KIND)
        return
    name = frame.payload.get("name")
    if kind != _FINISHED:
        current.pop_scene(KIND)
        node.get_logger().info(f"💬 [reply-window] greeting {name}: кадр снят, приветствие не прозвучало (kind={kind})")
        return
    spoken_text = str(frame.payload.get("text") or "")[:max(0, spoken_chars)].strip()
    if spoken_text:
        # Запись добавляет ход, поэтому проходит в том же цикле, где LLM читает историю.
        node._loop.call_soon_threadsafe(_record_spoken_greeting, node, speech_id, spoken_text, snapshot.epoch)
    _arm(node, spoken_text)
    node.get_logger().info(
        f"💬 [reply-window] greeting {name}: окно открыто ttl={kn.REPLY_WINDOW_S}s "
        f"(микрофон после речи +{kn.IDENTITY_MIC_GRACE_S}s)")


def window(node: Any) -> Optional[Any]:
    """Кадр ``GreetingReply`` с открытым окном (истёкшие сессия снимает) или ``None``."""
    current = session(node)
    current.expire()
    frame = current.snapshot.top(KIND)
    return frame if greeting_open(frame) else None


def take_reply(node: Any, source: str, heard: Any) -> Optional[SceneFrame]:
    """Вернуть кадр адресованной голосовой фразе и снять его. Голое «робот» переоткрывает окно (#4509)."""
    frame = window(node)
    if frame is None or (getattr(heard, "source", "") or "respeaker") != "respeaker":
        return None
    name = frame.payload.get("name")
    if source == GREETING_WAKE:
        rearmed = _arm(node)
        node.get_logger().info(f"💬 [reply-window] greeting {name}: голое «робот» → «слушаю», окно открыто заново "
                               f"ttl={kn.REPLY_WINDOW_S}s")
        return rearmed
    session(node).pop_scene(KIND)
    _announce(node, 0.0)
    node.get_logger().info(
        f"💬 [reply-window] greeting {name}: принят source={source} "
        f"text={str(getattr(heard, 'text', ''))[:60]!r}")
    return frame


def keep_greeting_after_idle_reset(node: Any, frame: Optional[SceneFrame], epoch: Optional[int]) -> None:
    """Перед принятым ответом сохранить только новое приветствие при сбросе устаревшей истории."""
    if frame is None or epoch != session(node).snapshot.epoch:
        return
    spoken_text = str(frame.payload.get("text") or "")
    if not spoken_text:
        return
    engine = node._engine
    current = engine.session
    now = engine._clock()
    rearmed = current.snapshot.top(KIND) is frame
    if reset_if_idle(current, engine.log, engine._window, now, engine._log):
        _record_spoken_greeting(node, str(frame.payload["speech_id"]), spoken_text, current.snapshot.epoch)
        remaining = (frame.ttl_until or 0.0) - current.now()
        if rearmed and remaining > 0.0:
            current.push_scene(KIND, frame.set_by, ttl_s=remaining, payload=frame.payload)
    engine._window.touch(now)


def not_a_reply(node: Any, heard: Any) -> None:
    """Окно открыто, а фраза без вейка не ответ (грамматика не та, лица приветствованного нет в кадре) — в бэклог."""
    frame = window(node)
    if frame is not None:
        node.get_logger().info(
            f"💬 [reply-window] greeting {frame.payload.get('name')}: не ответ → бэклог "
            f"faces={len(getattr(heard, 'faces', ()) or ())} text={str(getattr(heard, 'text', ''))[:60]!r}")
