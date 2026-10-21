"""Кадр ``Identity(ask)`` и ``/dialog/session`` в ``dialogue_node`` v1 (ADR-0150 §7.1, §7.2, §7.5 п.2; #3695).

Вопрос об имени — кадр стека сессии (:class:`rob_box_dialog.session.Session`): код кладёт его, когда вопрос ушёл в речь (до конца вопроса он ждёт),
окно ответа ``IDENTITY_TTL_S`` (15 с) открывается, когда вопрос договорён (``SpeechEvent finished``), и снимается первой адресованной фразой. Пока кадр жив, фраза без вейка с той же
стороны — ответ на него (В17(б), правило — ``speaker_adapter.answers_question``). Ответ «меня зовут X» регистрирует
голос кодом (#2925, ``_register_self_intro``), голое имя на «Как тебя зовут?» — тоже (:func:`answer_name`).

Каждое изменение сессии (собеседник, кадр) уходит latched ``SessionState`` в ``/dialog/session``: собеседник с
азимутом его фразы — для взгляда лица (``face_node``), до PR-7 пишет v1 ``dialogue_node``.

Функции, а не методы: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import Session, SessionChanged, SessionSnapshot, SpeakerRef
from rob_box_dialog.speaker_adapter import AskCandidate, question_frame, question_open
from rob_box_voice.core.self_intro import KNOWN_FIRST_NAMES, extract_self_intro_name

#: Топик снимка сессии (``architecture/ownership.yml``).
SESSION_TOPIC = "/dialog/session"
#: ``Identity.kind`` кадров, где ответ — имя: голое имя («Гриша») — представление.
ASKS_NAME = ("unknown", "contested")
#: ``SpeechEvent.kind``: finished / cancelled / rejected.
_FINISHED, _CANCELLED, _REJECTED = 4, 5, 7


def session(node: Any) -> Session:
    """Сессия ноды (лениво — для нод из тестов через ``object.__new__``); изменения — в ``/dialog/session``."""
    current = getattr(node, "_session", None)
    if current is None:
        current = node._session = Session("personality", clock=time.time)
        current.subscribe(lambda event: publish(node, event))
    return current


def set_speaker(node: Any, ref: SpeakerRef) -> None:
    session(node).set_speaker(ref)


def open_question(node: Any, cand: AskCandidate, speech_id: str) -> None:
    """Вопрос отдан в речь — кадр ``Identity`` с азимутом фразы собеседника, пока ЖДЁТ конца речи ``speech_id``.

    Окно ответа открывает :func:`on_speech_event` (``SpeechEvent finished`` этого вопроса): ход мог отвечать 12 с,
    вопрос звучит после, и 15 с от конца хода истекли бы до вопроса (#3771)."""
    ref = getattr(node, "_speaker_ref", None)
    azimuth = ref.azimuth_deg if ref is not None else None
    frame = question_frame(cand.key, cand.kind, cand.name, azimuth, None, speech_id)
    session(node).pop_scene("Identity")
    session(node).push_scene("Identity", "code", ttl_s=float(kn.IDENTITY_WAIT_QUESTION_S), payload=frame)
    node.get_logger().info(
        f"❓ [Identity #3695] кадр ждёт конца вопроса: kind={cand.kind} key={cand.key[:8]} "
        f"azimuth={'-' if azimuth is None else f'{azimuth:.0f}'} speech_id={speech_id[:8]}"
    )


def on_speech_event(node: Any, kind: int, speech_id: str, agent: str) -> None:
    """``SpeechEvent`` речи личности: вопрос договорён — открыть окно ответа; отменён/отклонён — снять кадр."""
    if agent != "personality" or not speech_id or kind not in (_FINISHED, _CANCELLED, _REJECTED):
        return
    current = session(node)
    frame = current.snapshot.top("Identity")
    if frame is None or frame.payload.get("speech_id") != speech_id or question_open(frame):
        return
    if kind != _FINISHED:
        current.pop_scene("Identity")
        node.get_logger().info(
            f"❓ [Identity #3695] кадр снят: вопрос не прозвучал (SpeechEvent kind={kind} speech_id={speech_id[:8]})")
        return
    ttl = float(kn.IDENTITY_MIC_GRACE_S + kn.IDENTITY_TTL_S)
    current.arm_scene("Identity", ttl, {"asked_at": current.now() + kn.IDENTITY_MIC_GRACE_S})
    node.get_logger().info(
        f"❓ [Identity #3695] кадр открыт для ответа: kind={frame.payload.get('kind')} "
        f"azimuth={frame.payload.get('azimuth_deg')} ttl={kn.IDENTITY_TTL_S}s (вопрос договорён, "
        f"микрофон после речи +{kn.IDENTITY_MIC_GRACE_S}s)")


def question(node: Any) -> Optional[Any]:
    """Кадр ``Identity`` с открытым окном ответа (истёкшие сессия снимает) или ``None``; ждущий конца вопроса — ``None``."""
    current = session(node)
    current.expire()
    frame = current.snapshot.top("Identity")
    return frame if question_open(frame) else None


def take_answer(node: Any, source: str, heard: Any) -> None:
    """Адресованная голосовая фраза при живом кадре — ответ на вопрос: кадр снимается (первая же фраза, как #2809),
    данные вопроса остаются на ход (``_identity_answer``) — для :func:`answer_name`."""
    node._identity_answer = None
    frame = question(node)
    if frame is None or (getattr(heard, "source", "") or "respeaker") != "respeaker":
        return
    session(node).pop_scene("Identity")
    node._identity_answer = dict(frame.payload)
    doa = getattr(heard, "doa_deg", None)
    azimuth = frame.payload.get("azimuth_deg")
    node.get_logger().info(
        f"❓ [Identity #3695] ответ на вопрос: source={source} doa={'-' if doa is None else f'{doa:.0f}'} "
        f"azimuth={'-' if azimuth is None else f'{azimuth:.0f}'} "
        f"age={float(getattr(heard, 't', 0.0) or 0.0) - float(frame.payload.get('asked_at') or 0.0):.1f}s"
    )


def answer_name(node: Any, text: str) -> Optional[str]:
    """Голое имя («Гриша», «Гриша.») в ответ на «Как тебя зовут?» — представление; иначе ``None``.

    Только имя из словаря ``self_intro.KNOWN_FIRST_NAMES`` (точность, а не полнота — как короткое «я X», #2925).
    """
    asked = getattr(node, "_identity_answer", None)
    words = [w.strip(".,!?…").lower().replace("ё", "е") for w in (text or "").split()]
    words = [w for w in words if w]
    if not asked or asked.get("kind") not in ASKS_NAME or len(words) != 1 or words[0] not in KNOWN_FIRST_NAMES:
        return None
    return words[0].capitalize()


def introduced_name(node: Any, text: str) -> Optional[str]:
    """Имя, которым человек назвал себя в реплике хода: «меня зовут X» (#2925) или голое имя в ответ на вопрос."""
    return extract_self_intro_name(text) or answer_name(node, text)


def publish(node: Any, event: SessionChanged) -> None:
    pub = getattr(node, "_session_pub", None)
    if pub is None:
        return
    try:
        pub.publish(session_state(event.snapshot))
    except Exception as exc:  # noqa: BLE001 — снимок для взгляда и панели не должен ронять ход
        node.get_logger().warning(f"⚠️ [#3695] /dialog/session не опубликован: {type(exc).__name__}: {exc}")


def session_state(snap: SessionSnapshot) -> Any:
    """``SessionSnapshot`` → ``rob_box_dialog_msgs/SessionState`` (§10.1)."""
    from rob_box_dialog_msgs.msg import SceneFrame, SessionState

    msg = SessionState()
    msg.agent, msg.session_id, msg.epoch = snap.agent, snap.session_id, int(snap.epoch)
    msg.scenes = [_scene(SceneFrame(), f) for f in snap.scenes]
    msg.voice_requested, msg.voice_applied = snap.voice.requested or "", snap.voice.applied or ""
    speaker = snap.speaker or SpeakerRef()
    msg.speaker_id, msg.speaker_name = speaker.person_ref or "", speaker.name or ""
    msg.speaker_name_state = speaker.name_state
    msg.speaker_azimuth_deg = math.nan if speaker.azimuth_deg is None else float(speaker.azimuth_deg)
    msg.pending_ask = str(getattr(snap.pending_ask, "id", "") or "")
    _time(msg.stamp, time.time())
    return msg


def _scene(msg: Any, frame: Any) -> Any:
    msg.kind, msg.set_by = frame.kind, frame.set_by
    _time(msg.ttl_until, frame.ttl_until or 0.0)
    msg.payload_json = json.dumps(dict(frame.payload), ensure_ascii=False)
    return msg


def _time(msg: Any, t: float) -> Any:
    msg.sec, msg.nanosec = int(t), int((t - int(t)) * 1e9)
    return msg


__all__ = ["SESSION_TOPIC", "answer_name", "introduced_name", "on_speech_event", "open_question", "question", "session", "session_state",
           "set_speaker", "take_answer"]
