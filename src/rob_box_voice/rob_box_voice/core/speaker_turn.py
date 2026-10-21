"""Собеседник хода и вопрос об имени в ``dialogue_node`` v1 (ADR-0150 §7.5, решение В17; карточка #3695).

Связка ноды с :mod:`rob_box_dialog.speaker_adapter`: правила (кто это, когда спрашивать) — там, здесь только
чтение состояния ноды, лог и речь. Вопрос об имени больше не звучит посреди хода и не заменяет его ответ
(живой лог 20.10: три «Как тебя зовут?» за 45 минут, два — вместо команды): кандидат копится репликами, а
решение принимается после хода — без действия, с речью, не раньше N-й фразы, один раз на собеседника.

Функции, а не методы: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

import time
from typing import Any, Mapping, Optional

from rob_box_dialog.knowledge import IDENTITY_FACE_RECENT_S
from rob_box_dialog.speaker_adapter import (
    ASK,
    STRANGER,
    AskCandidate,
    AskPolicy,
    ask_after_turn,
    speaker_ref,
)
from rob_box_voice.core.identity_ack import tentative_plan


def ask_policy(node: Any) -> AskPolicy:
    """Правило вопроса ноды; лениво — для нод из тестов, собранных через ``object.__new__``."""
    policy = getattr(node, "_identity_ask", None)
    if policy is None:
        policy = node._identity_ask = AskPolicy()
    return policy


def note_face(node: Any, msg: Any) -> None:
    """``/vision/hailo/events`` → лица в кадре (``FaceSightings``) для сверки собеседника."""
    faces = getattr(node, "_faces", None)
    if faces is not None:
        faces.note_event(msg, time.time())


def note_backlog_voice(node: Any, utterance_id: str, data: Mapping[str, Any]) -> None:
    """Биометрия приходит после STT: узнанный голос (id и имя) дописывается фразе в бэклоге."""
    backlog = getattr(node, "_backlog", None)
    if backlog is not None and data.get("is_known"):
        backlog.note_voice(utterance_id, str(data.get("speaker_id") or ""),
                           str(data.get("name") or data.get("tentative_name") or ""))


def note_interlocutor(node: Any, sp: Mapping[str, Any], utterance_id: Optional[str]) -> None:
    """Живая фраза хода: ``SpeakerRef`` через адаптер, +1 фраза собеседнику; незнакомец — кандидат на вопрос.

    Незнакомцем считается только голос, оценённый по ЭТОЙ фразе (не наследованный и не inconclusive), и только
    если в кадре нет лица с именем — тогда собеседник опознан лицом и спрашивать нечего.
    """
    node._identity_ask_candidate = None
    if not utterance_id:
        return
    now = time.time()
    faces = getattr(node, "_faces", None)
    names = faces.names_during(now - IDENTITY_FACE_RECENT_S, now) if faces is not None else frozenset()
    ref = speaker_ref(sp, now=now, face_names=names, prev=getattr(node, "_speaker_ref", None))
    node._speaker_ref = ref
    count = ask_policy(node).note(ref.person_ref or STRANGER, now)
    node.get_logger().info(
        f"👤 [SpeakerRef] person={(ref.person_ref or '-')[:8]} state={ref.name_state} "
        f"channels={','.join(ref.channels)} faces_named={len(names)} phrases={count}"
    )
    own = sp.get("utterance_id") == utterance_id and not sp.get("inconclusive")
    if ref.person_ref is None and own and not names:
        node._identity_ask_candidate = AskCandidate(STRANGER, "unknown", None, utterance_id)


def plan_identity_question(node: Any, result: Any, *, other_question: bool) -> Optional[AskCandidate]:
    """Конец хода, до post-turn гардов: спросить ли имя (правило В17). ``other_question`` — в этом ходе уже
    звучит другой вопрос о личности (#2828). Вернувшийся кандидат гарды видят как «вопрос будет» и не шлют
    ретрай поверх него (#2914); прозвучит он в :func:`ask_planned` после ответа хода."""
    cand = getattr(node, "_identity_ask_candidate", None)
    node._identity_ask_candidate = None
    if cand is None or result is None or other_question or getattr(node, "_identity_question_asked_in_turn", False):
        return None
    verdict = ask_after_turn(ask_policy(node), cand, t=time.time(),
                             tools=list(getattr(result, "tools_called", None) or ()),
                             spoken_text=getattr(result, "spoken_text", None))
    node.get_logger().info(f"👥 [В17 #3695] вопрос об имени: {verdict} (kind={cand.kind} key={cand.key[:8]})")
    if verdict != ASK:
        return None
    state = (getattr(node, "_identity_confirmations", None) or {}).get(cand.key)
    if state is not None:
        state.update(asked=True, asked_utterance_id=cand.utterance_id)
    node._identity_question_asked_in_turn = True  # гарды: вопрос в этом ходе будет — ретрая поверх нет (#2914)
    return cand


def ask_planned(node: Any, cand: Optional[AskCandidate]) -> None:
    """Задать вопрос, решённый :func:`plan_identity_question`, последней репликой хода."""
    if cand is not None:
        node._speak_identity_question(tentative_plan(cand.question(), cand.kind, cand.name), after_answer=True)
