"""Личность хода с конфликтом голоса (ADR-0148). Chooser «уже спросила?» (#3765, ``core/identity_dedup``) удалён
в #4354 A1, вопрос после хода без вызывающего (``start_after_turn_question``/``ask_after_turn``) — в #4354 A2."""

from __future__ import annotations

import asyncio
import threading
import types
from unittest.mock import MagicMock

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402

PLAN = {"kind": "conflict", "question": "Вы разные люди или это ты под другим именем?", "new_id": "new",
        "new_name": "Матвей", "known_id": "old", "known_name": "Аркадий"}


# -- личность хода с конфликтом голоса выясняет робот, не LLM (п.1 координатора) ----------------------------


def _intro_node(plan=None, outcome=True):
    logger = MagicMock()
    node = types.SimpleNamespace(get_logger=lambda: logger, _speaker_lock=threading.Lock(),
                                 _current_speaker={"is_known": True, "speaker_id": "old", "name": "Аркадий"},
                                 _turn_intro_registered=True, _after_turn_question=plan,
                                 _turn_register_outcome={"event": "registered"} if outcome else None,
                                 _turn_self_intro="Матвей")
    return node


def test_conflict_hides_voice_match_and_register_speaker_from_llm_turn():
    node = _intro_node(PLAN)
    asyncio.run(iq.settle_identity(node, timeout_s=0.1))
    assert node._current_speaker == {"is_known": False, "introduced_name": "Матвей"}
    assert node._turn_hidden_tools == ("register_speaker",)


def test_plain_registration_keeps_speaker_and_tools():
    node = _intro_node(None)
    asyncio.run(iq.settle_identity(node, timeout_s=0.1))
    assert node._current_speaker["name"] == "Аркадий"
    assert not getattr(node, "_turn_hidden_tools", ())


def test_outcome_not_arrived_in_time_turn_goes_on():
    node = _intro_node(None, outcome=False)
    asyncio.run(iq.settle_identity(node, timeout_s=0.05))
    assert node._current_speaker["name"] == "Аркадий"


def test_wait_constant_lives_in_knowledge():
    assert kn.REGISTER_OUTCOME_WAIT_S == 2.5


def test_real_speaking_watch_property_is_read_not_called():
    """Живой прогон 3: SpeakingWatch.speaking — свойство, читается, а не вызывается."""
    from rob_box_core.speech_client import SpeakingWatch

    node = types.SimpleNamespace(_speaking_watch=SpeakingWatch("personality"))
    assert iq._speaking(node) is False
