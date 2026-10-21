"""#3793: исход регистрации по самопредставлению — в снимке хода структурой (ADR-0148).

Живой робот 21.10 (комм. 6327740694): «Робот, я Глеб» (1.3 с) — код сам отправил регистрацию, ``speaker_id_node``
отказал ``too_short``, а LLM, не видя ни тула ``register_speaker``, ни исхода, сказала «запомнил тебя», и следом код
попросил повторить. Теперь ход ждёт исход (``settle_identity``) и кладёт его в раздел ``speaker``.
"""

from __future__ import annotations

import asyncio
import json
import threading
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog.context import build_context  # noqa: E402
from rob_box_dialog.speaker_adapter import voice_registration  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402
from rob_box_voice.core.speaker_turn import speaker_section  # noqa: E402

TOO_SHORT = {"event": "register_error", "error": "too_short", "name": "Глеб", "utterance_id": "u1",
             "duration_s": 1.29, "min_required_s": 3.0}
REGISTERED = {"event": "registered", "name": "Глеб", "speaker_id": "fe3ef104", "utterance_id": "u1"}


def _node(outcome, sent=True, plan=None):
    logger = MagicMock()
    return types.SimpleNamespace(get_logger=lambda: logger, _speaker_lock=threading.Lock(), _speaker_ref=None,
                                 _current_speaker={"is_known": False, "utterance_id": "u1"},
                                 _turn_intro_registered=sent, _turn_register_outcome=outcome,
                                 _after_turn_question=plan, _turn_self_intro="Глеб")


def _turn_context(node):
    """То, что видит LLM: снимок хода после ожидания исхода (как ``_prepare_user_input_context``)."""
    asyncio.run(iq.settle_identity(node, timeout_s=0.05))
    return speaker_section(node), build_context({"speaker": speaker_section(node)})


def test_too_short_context_says_voice_not_saved():
    section, text = _turn_context(_node(TOO_SHORT))
    assert section["voice_registration"] == {"registered": False, "reason": "too_short"}
    assert '"voice_registration":{"registered":false,"reason":"too_short"}' in text


def test_registered_context_says_voice_saved():
    section, text = _turn_context(_node(REGISTERED))
    assert section["voice_registration"] == {"registered": True}
    assert '"registered":true' in text


def test_no_answer_in_time_is_unknown():
    section, text = _turn_context(_node(None))
    assert section["voice_registration"] == {"registered": None, "reason": "no_answer"}
    assert '"registered":null' in text


def test_late_outcome_is_seen_by_turn():
    """Исход пришёл, пока ход ждал (живой лог: 20–50 мс после отправки) — в снимке уже он, а не «неизвестно»."""
    node = _node(None)
    threading.Timer(0.05, lambda: setattr(node, "_turn_register_outcome", TOO_SHORT)).start()
    asyncio.run(iq.settle_identity(node, timeout_s=2.0))
    assert speaker_section(node)["voice_registration"]["registered"] is False


def test_turn_without_self_intro_has_no_field():
    section, text = _turn_context(_node(None, sent=False))
    assert "voice_registration" not in section and "voice_registration" not in text


def test_identity_turn_resolved_by_robot_has_no_voice_info():
    """#3765: голос похож на другого — личность выясняет робот, сведений о голосе в ходе нет."""
    plan = {"kind": "conflict", "question": "Вы разные люди?", "new_id": "n", "new_name": "Глеб",
            "known_id": "o", "known_name": "Борис"}
    section, _ = _turn_context(_node(dict(REGISTERED, voice_conflict={"name": "Борис"}), plan=plan))
    assert "voice_registration" not in section


@pytest.mark.parametrize("outcome, expected", [
    ({"event": "register_error", "error": "utterance_not_found"}, {"registered": False,
                                                                   "reason": "utterance_not_found"}),
    ({"event": "register_error"}, {"registered": False, "reason": "error"}),
    ({"event": "registered", "already_known": True}, {"registered": True}),
])
def test_voice_registration_outcomes(outcome, expected):
    assert voice_registration(True, outcome) == expected
    assert json.dumps(voice_registration(False, outcome)) == "null"
