"""#3765: Chooser «LLM в этом ходе уже спросила о том же?» и личность хода с конфликтом голоса (ADR-0148)."""

from __future__ import annotations

import asyncio
import threading
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_music.choice import Choice  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402
from rob_box_voice.core.identity_dedup import ALREADY_SAID, NOT_SAID, IdentityDedupChooser  # noqa: E402

QUESTION = "Твой голос очень похож на голос, который я запомнил как Аркадий. Вы разные люди или это ты под другим именем?"
PLAN = {"kind": "conflict", "question": QUESTION, "new_id": "new", "new_name": "Матвей", "known_id": "old",
        "known_name": "Аркадий"}


class _Model:
    def __init__(self, answer=None, exc=None):
        self.answer, self.exc, self.calls = answer, exc, 0

    def choose(self, question, *, deadline_ms):
        self.calls += 1
        if self.exc is not None:
            raise self.exc
        return Choice(self.answer, 1.0, "stub")


def _chooser(model):
    logger = MagicMock()
    return IdentityDedupChooser(model, log=lambda: logger), logger


@pytest.mark.parametrize("model, verdict, reason", [
    (_Model(ALREADY_SAID), ALREADY_SAID, "model"),
    (_Model(NOT_SAID), NOT_SAID, "model"),
    (_Model(exc=TimeoutError()), NOT_SAID, "timeout:TimeoutError"),
    (_Model("что-то своё"), NOT_SAID, "out"),
    (None, NOT_SAID, "off"),
])
def test_verdicts_and_safe_default(model, verdict, reason):
    chooser, logger = _chooser(model)
    assert chooser.decide(["Голос у меня записан как Аркадий — вы один человек?"], QUESTION) == (verdict, reason)
    line = logger.info.call_args[0][0]
    assert line.startswith(f"🔁 [identity-dedup] verdict={verdict} ms=") and f"reason={reason}" in line


def test_nothing_said_by_llm_needs_no_model():
    model = _Model(ALREADY_SAID)
    chooser, _ = _chooser(model)
    assert chooser.decide(["", "  "], QUESTION) == (NOT_SAID, "no_llm_speech")
    assert model.calls == 0


def _node(dedup_answer=None, speaking=False):
    logger = MagicMock()
    node = types.SimpleNamespace(get_logger=lambda: logger, _session_pub=None, _speaker_ref=None,
                                 _turn_llm_said=["Голос у меня записан как Аркадий. Ты Аркадий или Матвей?"],
                                 _speaking_watch=types.SimpleNamespace(speaking=lambda: speaking),
                                 _identity_question_asked_in_turn=False)
    node._speak_identity_question = MagicMock(return_value="q1")
    if dedup_answer is not None:
        node._identity_dedup = IdentityDedupChooser(_Model(dedup_answer) if dedup_answer != "timeout"
                                                    else _Model(exc=TimeoutError()), log=lambda: logger)
    return node


def _result(text="done"):
    return types.SimpleNamespace(spoken_text=text)


def test_already_said_skips_code_phrase_but_opens_answer_window():
    node = _node(ALREADY_SAID)
    asyncio.run(iq.ask_after_turn(node, PLAN, _result()))
    node._speak_identity_question.assert_not_called()
    frame = iq.question(node)
    assert frame is not None and frame.payload["kind"] == "conflict" and frame.payload["known_id"] == "old"
    assert node._identity_question_asked_in_turn is True


def test_already_said_while_robot_still_speaks_opens_window_at_end_of_speech():
    node = _node(ALREADY_SAID, speaking=True)
    asyncio.run(iq.ask_after_turn(node, PLAN, _result()))
    assert iq.question(node) is None  # окно ещё не открыто — речь хода звучит
    node._speaking_watch = types.SimpleNamespace(speaking=lambda: False)
    iq.on_speech_event(node, 4, "llm-chunk-2", "personality")
    assert iq.question(node) is not None


@pytest.mark.parametrize("answer", [NOT_SAID, "timeout", None])
def test_not_said_timeout_or_no_chooser_speaks_code_phrase(answer):
    node = _node(answer)
    asyncio.run(iq.ask_after_turn(node, PLAN, _result()))
    node._speak_identity_question.assert_called_once_with(PLAN)


def test_retry_request_already_said_is_silent_and_opens_nothing():
    node = _node(ALREADY_SAID)
    plan = {"kind": "register_retry", "question": "Только голос твой я запомнить не успел — скажи ещё пару слов."}
    asyncio.run(iq.ask_after_turn(node, plan, _result()))
    node._speak_identity_question.assert_not_called()
    assert iq.session(node).snapshot.top("Identity") is None


# -- личность хода с конфликтом голоса выясняет робот, не LLM (п.1 координатора) ----------------------------


def _intro_node(plan=None, outcome=True):
    logger = MagicMock()
    node = types.SimpleNamespace(get_logger=lambda: logger, _speaker_lock=threading.Lock(),
                                 _current_speaker={"is_known": True, "speaker_id": "old", "name": "Аркадий"},
                                 _turn_intro_registered=True, _after_turn_question=plan,
                                 _turn_register_outcome={"event": "registered"} if outcome else None)
    return node


def test_conflict_hides_voice_match_and_register_speaker_from_llm_turn():
    node = _intro_node(PLAN)
    asyncio.run(iq.settle_identity(node, timeout_s=0.1))
    assert node._current_speaker == {"is_known": False}
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
    assert kn.REGISTER_OUTCOME_WAIT_S == 2.5 and kn.IDENTITY_DEDUP_DEADLINE_MS == kn.STOP_CHOOSER_DEADLINE_MS
