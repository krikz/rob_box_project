"""Собеседник хода и вопрос об имени в ``dialogue_node`` v1 (ADR-0150 §7.5, решение В17; карточка #3695).

Связка ноды с :mod:`rob_box_dialog.speaker_adapter`: правила (кто это, когда спрашивать) — там, здесь только
чтение состояния ноды, лог и речь. Вопрос об имени больше не звучит посреди хода и не заменяет его ответ
(живой лог 20.10: три «Как тебя зовут?» за 45 минут, два — вместо команды): кандидат копится репликами, а
решение принимается после хода — не на команду (по запросу человека, не по тулам LLM), с речью, не раньше N-й
фразы, один раз на собеседника.

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
    TentativeStates,
    ask_after_turn,
    speaker_block,
    speaker_ref,
)
from rob_box_voice.core.dialogue_helpers import sanitize_speaker_name


def ask_policy(node: Any) -> AskPolicy:
    """Правило вопроса ноды; лениво — для нод из тестов, собранных через ``object.__new__``."""
    policy = getattr(node, "_identity_ask", None)
    if policy is None:
        policy = node._identity_ask = AskPolicy()
    return policy


def tentative_states(node: Any) -> TentativeStates:
    """Гипотезы имени по собеседникам (#2809); окна — параметры ноды; лениво — для нод из ``object.__new__``."""
    states = getattr(node, "_tentative", None)
    if states is None:
        states = node._tentative = TentativeStates(getattr(node, "_identity_answer_window_sec", 180.0),
                                                   getattr(node, "_identity_question_session_gap_sec", 120.0))
    return states


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


def note_interlocutor(node: Any, sp: Mapping[str, Any], utterance_id: Optional[str], request: str = "") -> None:
    """Живая фраза хода: ``SpeakerRef`` через адаптер, +1 фраза собеседнику; незнакомец — кандидат на вопрос.
    ``request`` — реплика человека: по ней после хода решается, была ли это команда.

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
    own = sp.get("utterance_id") == utterance_id and not sp.get("inconclusive")
    if ref.person_ref is None and own and not names:
        node._identity_ask_candidate = AskCandidate(STRANGER, "unknown", None, utterance_id, request)
    node.get_logger().info(
        f"👤 [SpeakerRef] person={(ref.person_ref or '-')[:8]} state={ref.name_state} "
        f"channels={','.join(ref.channels)} faces_named={len(names)} phrases={count} {_biometry(sp, utterance_id)} "
        f"ask_candidate={'yes' if node._identity_ask_candidate else 'no'}"
    )


def _biometry(sp: Mapping[str, Any], utterance_id: str) -> str:
    """Что биометрия сказала о ЭТОЙ фразе — то, что доходит до диалога в ``SpeakerResult``. Лучший кандидат
    незнакомого голоса и порог в сообщении не передаются — они в строке ``🔍 identify candidates`` speaker_id_node."""
    if sp.get("utterance_id") != utterance_id:
        return f"bio=stale(utt={str(sp.get('utterance_id') or '-')[:8]})"
    if sp.get("inconclusive"):
        return f"bio=inconclusive({sp.get('reason') or '-'})"
    if not sp.get("is_known"):
        return "bio=unknown_voice"
    tentative = ""
    if sp.get("tentative_kind"):
        tentative = f" tentative={sp.get('tentative_name')}@{float(sp.get('tentative_conf') or 0):.3f}"
    return f"bio=known conf={float(sp.get('confidence') or 0):.3f} name={sp.get('name') or '-'}{tentative}"


def speaker_section(node: Any) -> dict:
    """Раздел ``speaker`` снимка хода (ADR-0150 §7.3, #3695): собеседник по текущему результату биометрии и лицам
    в кадре через адаптер; имя — только при ``confirmed``. Словесное «да, это я» (#2809) уже записало имя в
    ``_current_speaker`` — оно здесь и становится подтверждённым."""
    with node._speaker_lock:
        sp = dict(getattr(node, "_current_speaker", None) or {})
    sp["name"] = sanitize_speaker_name(sp.get("name")) or None  # live 12.08: «Null»/«None» в БД — не имя
    now = time.time()
    faces = getattr(node, "_faces", None)
    names = faces.names_during(now - IDENTITY_FACE_RECENT_S, now) if faces is not None else frozenset()
    return speaker_block(speaker_ref(sp, now=now, face_names=names, prev=getattr(node, "_speaker_ref", None)))


def plan_identity_question(node: Any, result: Any, *, other_question: bool) -> Optional[AskCandidate]:
    """Конец хода, до post-turn гардов: спросить ли имя (правило В17). ``other_question`` — в этом ходе уже
    звучит другой вопрос о личности (#2828). Вернувшийся кандидат гарды видят как «вопрос будет» и не шлют
    ретрай поверх него (#2914); прозвучит он в :func:`ask_planned` после ответа хода."""
    cand = getattr(node, "_identity_ask_candidate", None)
    node._identity_ask_candidate = None
    if cand is None or result is None or other_question or getattr(node, "_identity_question_asked_in_turn", False):
        return None
    now = time.time()
    verdict = ask_after_turn(ask_policy(node), cand, t=now,
                             tools=list(getattr(result, "tools_called", None) or ()),
                             spoken_text=getattr(result, "spoken_text", None))
    node.get_logger().info(
        f"👥 [В17 #3695] вопрос об имени: {verdict.describe(now)} (kind={cand.kind} key={cand.key[:8]})")
    if verdict.verdict != ASK:
        return None
    state = tentative_states(node).get(cand.key)
    if state is not None:
        state.update(asked=True, asked_utterance_id=cand.utterance_id)
    node._identity_question_asked_in_turn = True  # гарды: вопрос в этом ходе будет — ретрая поверх нет (#2914)
    return cand


def ask_planned(node: Any, cand: Optional[AskCandidate]) -> None:
    """Задать вопрос, решённый :func:`plan_identity_question`, последней репликой хода."""
    if cand is not None:
        node._speak_identity_question({"kind": "tentative", "question": cand.question(), "name": cand.name},
                                      after_answer=True)
