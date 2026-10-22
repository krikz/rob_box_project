"""test_issue_2828_identity_question_collision.py

Issue #2828 — вопрос «вы разные люди или это ты?» после регистрации; #3765 (ADR-0150 PR-5c) — на кадре ``Identity``.

Живой случай (run 35854757524, шаг n204): Борис представился, LLM вызвала ``register_speaker``, ack пришёл с
``voice_conflict`` (голос похож на Сашу). Робот спросил и сам себе ответил «Привет, Борис!», а ответ человека никто
не читал.

Контракт (#3765, как вопрос об имени по В17′):

* ack посреди хода → ответ хода звучит, вопрос — ПОСЛЕ него, последней репликой (не вместо);
* ack вне хода → вопрос сразу;
* вопрос открывает кадр ``Identity`` (kind twin/conflict) с окном ``REPLY_WINDOW_S`` от конца вопроса;
* ответ «это я» → ``/voice/speaker/merge`` new→known, «мы разные»/непонятно → ничего; что сказать — фраза
  ``phrases`` кодом, LLM на этот ход не зовётся (ADR-0148, вместо подсказки ``<identity_answer>``);
* фраза не по теме снимает кадр и идёт в LLM как обычный ход, ничего не склеено.

Тест не поднимает ROS2 — ``object.__new__``, как остальные тесты ноды.
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
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

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from ros_stubs import make_speaker_result  # noqa: E402

BORIS_ID = "f65d3050-0000-0000-0000-000000000000"
SASHA_ID = "5d99f474-0000-0000-0000-000000000000"

CONFLICT_ACK = {
    "event": "registered",
    "name": "Борис",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
    "voice_conflict": {"name": "Саша", "speaker_id": SASHA_ID, "score": 0.907},
}
TWIN_ACK = {
    "event": "registered",
    "name": "Дэнчик",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
    "name_twin": {"name": "Дэнчик", "speaker_id": SASHA_ID},
}
PLAIN_ACK = {"event": "registered", "name": "Борис", "speaker_id": BORIS_ID, "reused_profile": False}
GREETING = "Привет, Борис! Приятно познакомиться — друг Саши с пиццей."
INTRO = "добрый вечер, меня зовут Борис, я друг Саши"
_FINISHED = 4


class _Result:
    spoken_text = GREETING
    tools_called = ("register_speaker",)
    error = None


def _turn_node(ack_during_llm=None, ack_after_result=None):
    """Нода, у которой ход проходит по-настоящему через ``_run_turn``; замоканы только LLM, гарды, DSM. TTS — список
    ``tts``: туда пишут и ``_speak_direct``, и выдача ответа хода. ``llm_calls`` — сколько раз звали LLM."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": True, "speaker_id": BORIS_ID, "name": "Борис"}
    n._speaker_merge_pub = MagicMock()
    n.tts = []
    n.llm_calls = []

    def speak(text, language=None):
        n.tts.append(text)
        return f"speech-{len(n.tts)}"

    n._speak_direct = MagicMock(side_effect=speak)

    def handle_result(result, **_kw):
        n.tts.append(result.spoken_text)
        if ack_after_result is not None:
            n._on_speaker_result(make_speaker_result(ack_after_result))

    n._handle_result = MagicMock(side_effect=handle_result)

    async def invoke_llm(**kw):
        n.llm_calls.append(kw["user_input"])
        # ack прилетает, пока LLM дописывает приветствие — тайминг run 35854757524.
        if ack_during_llm is not None and len(n.llm_calls) == 1:
            n._on_speaker_result(make_speaker_result(ack_during_llm))
        return _Result()

    n._invoke_llm_with_telemetry = invoke_llm
    n._prepare_user_input_context = AsyncMock(side_effect=lambda **kw: (kw["user_input"], "<system_context/>"))
    n._llm = MagicMock()
    n._finalize_turn_dsm = MagicMock()
    return n


def _asked(ack=CONFLICT_ACK):
    """Ход с регистрацией прошёл, вопрос прозвучал и договорён (``SpeechEvent finished``) — окно ответа открыто."""
    n = _turn_node(ack_during_llm=ack)
    asyncio.run(n._run_turn(INTRO))
    iq.on_speech_event(n, _FINISHED, f"speech-{len(n.tts)}", "personality")
    return n


def _answer(n, text, speaker=None, dt=3.0):
    """Человек ответил: адресованная фраза снимает кадр (admission), затем её ход."""
    heard = types.SimpleNamespace(t=time.time() + dt, doa_deg=None, source="respeaker", text=text)
    iq.take_answer(n, "identity_answer", heard)
    if speaker is not None:
        n._current_speaker = speaker
    asyncio.run(n._run_turn(text))


def _merged(n):
    return [json.loads(c.args[0].data) for c in n._speaker_merge_pub.publish.call_args_list]


class TestQuestionAfterTurnReply:
    def test_ack_mid_turn_reply_first_then_question(self):
        """Регресс n204 в новой форме: ответ хода звучит, вопрос — после него, а не вместо."""
        n = _turn_node(ack_during_llm=CONFLICT_ACK)

        asyncio.run(n._run_turn(INTRO))

        assert n.tts[0] == GREETING
        assert len(n.tts) == 2, n.tts
        assert "Вы разные люди" in n.tts[1]

    def test_ack_after_reply_published_question_goes_last(self):
        n = _turn_node(ack_after_result=CONFLICT_ACK)

        asyncio.run(n._run_turn(INTRO))

        assert n.tts[0] == GREETING
        assert "Вы разные люди" in n.tts[-1]
        assert len(n.tts) == 2

    def test_plain_registration_keeps_turn_reply(self):
        n = _turn_node(ack_during_llm=PLAIN_ACK)

        asyncio.run(n._run_turn(INTRO))

        assert n.tts == [GREETING]
        assert iq.session(n).snapshot.top("Identity") is None

    def test_ack_outside_turn_speaks_immediately(self):
        n = _turn_node()

        n._on_speaker_result(make_speaker_result(CONFLICT_ACK))

        assert len(n.tts) == 1
        assert "Вы разные люди" in n.tts[0]

    def test_question_opens_identity_frame_with_reply_window(self):
        """Окно ответа — та же константа, что у вопроса об имени (В17′), от конца речи вопроса."""
        n = _asked()

        frame = iq.question(n)
        assert frame is not None and frame.payload["kind"] == "conflict"
        assert (frame.payload["new_id"], frame.payload["known_id"]) == (BORIS_ID, SASHA_ID)
        assert kn.IDENTITY_TTL_S == kn.REPLY_WINDOW_S == 15
        # два чтения часов сессии (time.time) — сравнение с допуском
        assert frame.ttl_until - frame.payload["asked_at"] == pytest.approx(kn.IDENTITY_TTL_S, abs=0.5)


class TestAnswerDecidedByCode:
    def test_same_person_merges_and_robot_says_phrase_without_llm(self):
        n = _asked()

        _answer(n, "Да, это я, просто назвался иначе")

        assert _merged(n) == [{"src_speaker_id": BORIS_ID, "dst_speaker_id": SASHA_ID}]
        assert n._current_speaker["speaker_id"] == SASHA_ID
        assert n._current_speaker["name"] == "Саша"
        assert n.tts[-1] == phrases.text("identity.merged", name="Саша")
        assert n.llm_calls == [INTRO], "на ответ LLM не зовётся"

    def test_code_phrase_bypasses_llm_text_guards(self):
        """Живой прогон 21.10 15:18: фраза «…Запомню обоих.» шла через _handle_result, гард #2549 счёл «Запомню» без
        тула галлюцинацией и отправил CRITICAL-ретрай в LLM. Фраза кода — мимо гардов LLM-текста."""
        n = _asked()
        handled = n._handle_result.call_count

        _answer(n, "мы разные")

        assert n._handle_result.call_count == handled
        assert n._speak_direct.call_args_list[-1].args[0] == phrases.text("identity.kept_apart")

    def test_different_people_keeps_profiles(self):
        n = _asked()

        _answer(n, "Робот, мы разные люди, я Борис")

        assert _merged(n) == []
        assert n.tts[-1] == phrases.text("identity.kept_apart")
        assert n.llm_calls == [INTRO]

    def test_bare_yes_on_conflict_is_ambiguous_and_merges_nothing(self):
        """«Вы разные люди или это ты?» — «да» ни о чём не говорит."""
        n = _asked()

        _answer(n, "Да")

        assert _merged(n) == []
        assert n.tts[-1] == phrases.text("identity.merge_unclear")

    def test_bare_yes_on_twin_merges(self):
        """«Ты тот самый Дэнчик?» — «да» значит «тот самый»."""
        n = _asked(TWIN_ACK)

        _answer(n, "да")

        assert _merged(n) == [{"src_speaker_id": BORIS_ID, "dst_speaker_id": SASHA_ID}]

    def test_answer_is_read_once(self):
        n = _asked()
        _answer(n, "мы разные")

        _answer(n, "да, это я")

        assert _merged(n) == []
        assert n.llm_calls == [INTRO, "да, это я"], "второй раз — обычный ход"

    def test_off_topic_phrase_closes_frame_and_goes_to_llm(self):
        n = _asked()

        _answer(n, "Робот, какая завтра погода?")

        assert _merged(n) == []
        assert iq.question(n) is None
        assert n.llm_calls[-1] == "Робот, какая завтра погода?"

    def test_long_addressed_answer_e2e_n723(self):
        """Шаг n723 акта 2c: длинный адресованный ответ — решение по смыслу, не по длине."""
        n = _asked()

        _answer(n, "Робот, мы разные люди, я Борис, а Саша мой друг.")

        assert _merged(n) == []
        assert n.tts[-1] == phrases.text("identity.kept_apart")
        assert n.llm_calls == [INTRO]
