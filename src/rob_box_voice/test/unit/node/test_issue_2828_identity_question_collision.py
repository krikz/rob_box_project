"""test_issue_2828_identity_question_collision.py

Issue #2828 — переспрос «вы разные люди?» и ответ LLM звучали подряд.

Живой случай (run 35854757524, шаг n204): Борис представился, LLM вызвала
``register_speaker``, ack пришёл с ``voice_conflict`` (голос похож на
Сашу) — и робот сказал «…Вы разные люди или это ты под другим именем?», а
через секунду сам же ответил «Привет, Борис! Приятно познакомиться».
Вдобавок ответ человека никто не читал: в ``dialogue_node`` не было ни
одной публикации в ``/voice/speaker/merge``.

Контракт после фикса:

* ack посреди хода → в TTS уходит ровно одна реплика — вопрос;
* ack после выдачи ответа хода → вопрос звучит после него, последним;
* ack вне хода → вопрос сразу (как в PR #2798);
* ответ «это я» → ``/voice/speaker/merge`` new→known; «мы разные» и
  непонятный ответ → ничего не склеиваем; LLM в любом случае получает
  подсказку с вопросом и исходом.

Тест не поднимает ROS2 — ``object.__new__``, как остальные тесты ноды.
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import types
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice.core.identity_ack import (  # noqa: E402
    IdentityAckQuestion,
    classify_identity_ack_answer,
)
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

BORIS_ID = "f65d3050-0000-0000-0000-000000000000"
SASHA_ID = "5d99f474-0000-0000-0000-000000000000"

CONFLICT_ACK = {
    "event": "registered",
    "name": "Борис",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
    "voice_conflict": {"name": "Саша", "speaker_id": SASHA_ID, "score": 0.907},
}
PLAIN_ACK = {
    "event": "registered",
    "name": "Борис",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
}
GREETING = "Привет, Борис! Приятно познакомиться — друг Саши с пиццей."


def _msg(payload: dict):
    return type("Msg", (), {"data": json.dumps(payload, ensure_ascii=False)})()


class _Result:
    spoken_text = GREETING
    tools_called = ("register_speaker",)
    error = None


def _turn_node(ack_during_llm=None, ack_after_result=None):
    """Нода, у которой ход проходит по-настоящему через ``_run_turn``.

    Замоканы только внешние стадии (LLM, гарды, DSM). TTS — это список
    ``tts``: туда пишут и ``_speak_direct``, и выдача ответа хода.
    """
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": True, "speaker_id": BORIS_ID, "name": "Борис"}
    n._speaker_merge_pub = MagicMock()
    n.tts = []
    n._speak_direct = MagicMock(side_effect=lambda text, language=None: n.tts.append(text))

    def handle_result(result, **_kw):
        n.tts.append(result.spoken_text)
        if ack_after_result is not None:
            n._on_speaker_result(_msg(ack_after_result))

    n._handle_result = MagicMock(side_effect=handle_result)

    async def invoke_llm(**_kw):
        # register_speaker отработал, ack прилетает, пока LLM дописывает
        # приветствие — ровно тайминг run 35854757524.
        if ack_during_llm is not None:
            n._on_speaker_result(_msg(ack_during_llm))
        return _Result()

    n._invoke_llm_with_telemetry = invoke_llm
    n._reset_turn_retry_budgets = MagicMock()
    n._prepare_user_input_context = AsyncMock(
        side_effect=lambda **kw: (kw["user_input"], "<system_context/>")
    )
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._drain_pending_user_messages = MagicMock(return_value=False)
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    return n


INTRO = "добрый вечер, меня зовут Борис, я друг Саши"


class TestQuestionDoesNotCollideWithTurnReply:
    def test_ack_mid_turn_only_question_is_spoken(self):
        """Регресс n204: вопрос и приветствие больше не звучат подряд."""
        n = _turn_node(ack_during_llm=CONFLICT_ACK)

        asyncio.run(n._run_turn(INTRO))

        assert len(n.tts) == 1, f"в TTS ушло {len(n.tts)} реплик: {n.tts!r}"
        assert "Вы разные люди" in n.tts[-1]
        assert GREETING not in n.tts
        n._handle_result.assert_not_called()

    def test_ack_after_reply_published_question_goes_last(self):
        """Ack опоздал к выдаче ответа — вопрос всё равно последний."""
        n = _turn_node(ack_after_result=CONFLICT_ACK)

        asyncio.run(n._run_turn(INTRO))

        assert n.tts[0] == GREETING
        assert "Вы разные люди" in n.tts[-1]
        assert len(n.tts) == 2

    def test_plain_registration_keeps_turn_reply(self):
        """n204 без конфликта: «Привет, Борис» звучит, вопроса нет."""
        n = _turn_node(ack_during_llm=PLAIN_ACK)

        asyncio.run(n._run_turn(INTRO))

        assert n.tts == [GREETING]

    def test_ack_outside_turn_speaks_immediately(self):
        """Хода нет — поведение PR #2798: вопрос сразу."""
        n = _turn_node()

        n._on_speaker_result(_msg(CONFLICT_ACK))

        assert len(n.tts) == 1
        assert "Вы разные люди" in n.tts[0]


class TestAnswerResolvesConflict:
    def _asked_node(self):
        n = _turn_node(ack_during_llm=CONFLICT_ACK)
        asyncio.run(n._run_turn(INTRO))
        return n

    def test_different_people_keeps_profiles(self):
        n = self._asked_node()

        n._resolve_identity_ack_answer("Робот, мы разные люди, я Борис")

        n._speaker_merge_pub.publish.assert_not_called()
        hint = "\n".join(n._identity_ack_state().pop_hint_lines())
        assert "разные люди" in hint
        assert "Вы разные люди" in hint, "LLM должна узнать, что спросили"

    def test_same_person_merges_new_into_known(self):
        n = self._asked_node()

        n._resolve_identity_ack_answer("Да, это я, просто назвался иначе")

        n._speaker_merge_pub.publish.assert_called_once()
        payload = json.loads(n._speaker_merge_pub.publish.call_args[0][0].data)
        assert payload == {"src_speaker_id": BORIS_ID, "dst_speaker_id": SASHA_ID}
        assert n._current_speaker["speaker_id"] == SASHA_ID
        assert n._current_speaker["name"] == "Саша"

    def test_bare_yes_on_conflict_is_ambiguous_and_merges_nothing(self):
        """«Вы разные люди или это ты?» — «да» ни о чём не говорит."""
        n = self._asked_node()

        n._resolve_identity_ack_answer("Да")

        n._speaker_merge_pub.publish.assert_not_called()
        hint = "\n".join(n._identity_ack_state().pop_hint_lines())
        assert "ничего не склеено" in hint

    def test_answer_is_read_once(self):
        n = self._asked_node()
        n._resolve_identity_ack_answer("мы разные")

        n._resolve_identity_ack_answer("да, это я")

        n._speaker_merge_pub.publish.assert_not_called()

    def test_prepare_context_reads_answer_before_llm(self):
        """Ответ читается в штатной прелюдии хода, по сырой реплике."""
        n = self._asked_node()
        n._speaker_id_enabled = True
        n._apply_speaker_identity = AsyncMock(side_effect=lambda ui, _ctx: f"[Spkr:Борис] {ui}")
        n._build_dynamic_system_context = MagicMock(return_value="")

        asyncio.run(DialogueNode._prepare_user_input_context(
            n, user_input="это я", from_tg=False, was_dj_auto=False,
            speaker_context=None,
        ))

        n._speaker_merge_pub.publish.assert_called_once()


class TestClassifier:
    @pytest.mark.parametrize("text, kind, yes_no, expected", [
        ("мы разные люди", "conflict", None, False),
        ("нет, я другой человек", "conflict", False, False),
        ("да, это я под другим именем", "conflict", True, True),
        ("да", "conflict", True, None),
        ("нет", "conflict", False, None),
        ("да", "twin", True, True),
        ("нет", "twin", False, False),
        ("я тот самый Дэнчик", "twin", None, True),
        ("не тот самый", "twin", None, None),
        ("пицца будет в пятницу", "conflict", None, None),
    ])
    def test_answers(self, text, kind, yes_no, expected):
        assert classify_identity_ack_answer(text, kind, yes_no) is expected

    def test_answer_expires(self):
        now = [0.0]
        state = IdentityAckQuestion(ttl_s=60.0, clock=lambda: now[0])
        state.arm({"kind": "twin", "question": "?", "new_id": "a",
                   "new_name": "X", "known_id": "b", "known_name": "X"})
        now[0] = 61.0

        assert state.consume("да, это я", True) is None
