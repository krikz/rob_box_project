"""test_issue_2888_identity_question_deterministic.py

Issue #2888 — переспрос «<Имя>, это ты?» был на усмотрение LLM.

Живой случай (E2E акт 2b, run 35903232434): ``classify_name_confidence``
вернул ``single``, ``dialogue_node`` выставил подсказку-гипотезу
(``identity question hint set: kind=single``), но вопрос в
``<name_hypothesis_rule>`` звучал как «Если уместно, ОДИН раз уточни…».
LLM не спросила ни разу из двух (n703/n706 FAIL ``robot_did_not_ask``) и
назвала имя-гипотезу как факт: «С возвращением, Саша. Узнал, приятель.»

Контракт после фикса (механизм #2828 — hold/replace переспроса, без
нового):

* ``single`` → в TTS этого хода ровно «<Имя>, это ты?», ответ LLM хода
  (с именем как фактом) не звучит, и имени-гипотезы нет в контексте LLM;
* ``contested`` → ровно «Как тебя зовут?», ни одного имени;
* один раз за сессию — следующий ход вопроса не повторяет, ответ LLM
  звучит как обычно;
* ответ на tentative-вопрос читает путь #2809, а не #2828 (склеивать
  профили нечего).

Тест не поднимает ROS2 — ``object.__new__``, как остальные тесты ноды.
"""

from __future__ import annotations

import asyncio
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

SASHA_ID = "5baad325-0000-0000-0000-000000000000"
GENA_ID = "9c0ffee0-0000-0000-0000-000000000000"
# Дословно то, что сказал робот в n702 run 35903232434.
NAME_AS_FACT = "С возвращением, Саша. Узнал, приятель."
PLAIN_REPLY = "Конечно, рад тебя слышать."


def _single():
    return {
        "is_known": True,
        "speaker_id": SASHA_ID,
        "name": None,
        "tentative_name": "Саша",
        "tentative_conf": 0.928,
        "tentative_kind": "single",
        "confidence": 0.928,
    }


def _contested():
    return {
        "is_known": True,
        "speaker_id": GENA_ID,
        "name": None,
        "tentative_conf": 0.80,
        "tentative_kind": "contested",
        "confidence": 0.80,
    }


class _Result:
    def __init__(self, text):
        self.spoken_text = text
        self.tools_called = ()
        self.error = None


def _node(llm_reply):
    """Ход идёт по-настоящему через ``_run_turn``; биометрия — через
    настоящий ``_apply_speaker_identity``. Замоканы LLM, гарды, DSM и TTS
    (список ``tts``: туда пишут ``_speak_direct`` и выдача ответа хода)."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = _single()
    n._publish_speaker_observation = MagicMock()
    n._speaker_register_pub = MagicMock()
    n._speaker_merge_pub = MagicMock()
    n._identity_confirmations = {}
    n._pending_identity_hint = None
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    n.tts = []
    n.llm_context = []
    n.llm_reply = llm_reply
    n._speak_direct = MagicMock(
        side_effect=lambda text, language=None: n.tts.append(text)
    )
    n._handle_result = MagicMock(
        side_effect=lambda result, **_kw: n.tts.append(result.spoken_text)
    )

    async def prepare(**kw):
        user_input = await n._apply_speaker_identity(kw["user_input"], None)
        dynamic = "\n".join(n._pending_identity_hint_lines())
        n.llm_context.append(dynamic)
        return user_input, dynamic

    async def invoke_llm(**_kw):
        return _Result(n.llm_reply)

    n._prepare_user_input_context = prepare
    n._invoke_llm_with_telemetry = invoke_llm
    n._reset_turn_retry_budgets = MagicMock()
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._drain_pending_user_messages = MagicMock(return_value=False)
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    return n


def _turn(n, text, speaker=None):
    n._current_speaker = speaker or _single()
    asyncio.run(n._run_turn(text))


class TestSingleAsksDeterministically:
    def test_question_with_name_replaces_reply_with_name_as_fact(self):
        """Регресс n702: вопрос звучит, «С возвращением, Саша» — нет."""
        n = _node(NAME_AS_FACT)

        _turn(n, "Робот, ты меня узнал по голосу?")

        assert n.tts == ["Саша, это ты?"], n.tts
        assert NAME_AS_FACT not in n.tts
        n._handle_result.assert_not_called()

    def test_name_hypothesis_not_given_to_llm(self):
        """Имя-гипотеза не уходит в LLM — ни в контекст, ни в user_input."""
        n = _node(NAME_AS_FACT)

        _turn(n, "Робот, ты меня узнал по голосу?")

        assert n.llm_context[0], "LLM должна знать, что вопрос задан"
        assert "Саш" not in n.llm_context[0]

    def test_not_asked_again_in_same_session(self):
        n = _node(NAME_AS_FACT)
        _turn(n, "Робот, ты меня узнал по голосу?")

        n.llm_reply = PLAIN_REPLY
        _turn(n, "Робот, ну как у тебя дела?")

        assert n.tts == ["Саша, это ты?", PLAIN_REPLY]
        assert "это ты" not in " ".join(n.tts[1:])

    def test_yes_is_read_by_2809_path_not_merged_by_2828(self):
        """«Да» подтверждает имя (#2809); профили не склеиваются (#2828)."""
        n = _node(NAME_AS_FACT)
        _turn(n, "Робот, ты меня узнал по голосу?")

        n._resolve_identity_ack_answer("да, это я")
        n.llm_reply = "Рад тебя слышать, Саша."
        _turn(n, "да, это я")

        n._speaker_merge_pub.publish.assert_not_called()
        assert n._identity_ack_state().pop_hint_lines() == []
        assert n._current_speaker["name"] == "Саша"
        assert n.tts == ["Саша, это ты?", "Рад тебя слышать, Саша."]


class TestContestedAsksWithoutNames:
    def test_how_are_you_called_without_names(self):
        n = _node("Привет, Борис! Или ты Саша?")

        _turn(n, "Робот, представляться пока не буду.", _contested())

        assert n.tts == ["Как тебя зовут?"], n.tts
        joined = " ".join(n.tts) + n.llm_context[0]
        assert "Борис" not in joined
        assert "Саш" not in joined

    def test_not_asked_again_in_same_session(self):
        n = _node("Привет!")
        _turn(n, "Робот, привет.", _contested())

        n.llm_reply = PLAIN_REPLY
        _turn(n, "Робот, это пока не важно.", _contested())

        assert n.tts == ["Как тебя зовут?", PLAIN_REPLY]
