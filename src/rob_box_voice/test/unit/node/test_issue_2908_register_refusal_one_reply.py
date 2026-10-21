"""test_issue_2908_register_refusal_one_reply.py

Issue #2908: отказ регистрации (``too_short``/``utterance_not_found``) приходит, пока LLM дописывает «Рад знакомству,
Борис!». Раньше «Не расслышал» и приветствие звучали подряд, потом (#2908) просьба ЗАМЕНЯЛА ответ хода.

#3765 (ADR-0150 PR-5c): просьба звучит ПОСЛЕ ответа хода — одной согласованной формой (имя услышано, не сохранён только
голос, фраза ``phrases`` identity.retry_voice); кадра ``Identity`` не открывает и склейку не ждёт.
"""

from __future__ import annotations

import asyncio

from rob_box_dialog import phrases
from rob_box_voice.core import identity_question as iq
from ros_stubs import make_speaker_result

from . import test_issue_2828_identity_question_collision as base

GREETING = base.GREETING
REFUSAL_ACK = {
    "event": "register_error",
    "error": "too_short",
    "name": "Борис",
    "duration_s": 1.2,
    "min_required_s": 1.5,
    "utterance_id": "5505eca40cea",
}
RETRY = phrases.text("identity.retry_voice")


def test_refusal_mid_turn_reply_then_retry_request():
    n = base._turn_node(ack_during_llm=REFUSAL_ACK)

    asyncio.run(n._run_turn(base.INTRO))

    assert n.tts == [GREETING, RETRY]
    assert "Не расслышал" not in " ".join(n.tts)


def test_refusal_after_reply_is_the_same_follow_up():
    n = base._turn_node(ack_after_result=REFUSAL_ACK)

    asyncio.run(n._run_turn(base.INTRO))

    assert n.tts == [GREETING, RETRY]


def test_refusal_outside_turn_speaks_immediately():
    n = base._turn_node()

    n._on_speaker_result(make_speaker_result(REFUSAL_ACK))

    assert n.tts == [RETRY]


def test_register_retry_is_not_an_identity_question():
    """Просьба повторить — не вопрос склейки: кадра нет, «да, это я» — обычный ход, ничего не склеено."""
    n = base._turn_node(ack_during_llm=REFUSAL_ACK)
    asyncio.run(n._run_turn(base.INTRO))

    assert iq.session(n).snapshot.top("Identity") is None
    asyncio.run(n._run_turn("да, это я"))
    n._speaker_merge_pub.publish.assert_not_called()
    assert n.llm_calls == [base.INTRO, "да, это я"]


def test_unspoken_error_code_stays_silent():
    n = base._turn_node()

    n._on_speaker_result(make_speaker_result(dict(REFUSAL_ACK, error="db_error")))

    assert n.tts == []
