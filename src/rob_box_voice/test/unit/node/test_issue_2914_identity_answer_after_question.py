"""test_issue_2914_identity_answer_after_question.py

Issue #2914 — повторный вопрос «<Имя>, это ты?» «отвечен» через 24 мс,
до реплики человека.

Живой случай (E2E акт 2b, run 35923951507): после словесного «да» (n703),
переиспользования подтверждения (n704) и паузы 150 с (n705) робот
заново спросил «Саша, это ты?», и через 24 мс dialogue_node записал
``identity answer read: answer=False``. Настоящий ответ «нет, я не Саша»
(n706) ответом уже не читался — строки ``answer read`` на нём не было.

Причина (проверено этим тестом на develop): ответ читает
``_resolve_pending_tentative_answer`` из ``_handle_tentative_speaker``,
то есть на ЛЮБОМ ходе, прошедшем через ``_apply_speaker_identity``.
Сразу после выдачи вопроса ``_run_turn`` в ``finally`` запускает
следующий ход без новой реплики человека — дренаж очереди фраз,
пришедших ПОКА шёл ход (S7, ``_drain_pending_user_messages``), или
синтетический ретрай гарда. У такого хода ``utterance_id=None``, снимок
диктора — всё тот же tentative «Саша», и его текст (фраза, сказанная ДО
вопроса, или служебный промпт ретрая) читался как ответ и закрывал
вопрос (``None`` тоже трактуется как отказ). В живом логе
``answer=False`` — значит, в тексте хода-читателя было отрицание; реплика
n705 даёт ``None``, так что там, вероятнее, был промпт ретрая. Какой
именно гард — по доступному логу харнесса не установлено.

Контракт после фикса: ответ на вопрос читается только из реплики
человека (со своим ``utterance_id``), отличной от той, на которой вопрос
задан; ход без новой реплики вопрос не закрывает.

Тест не поднимает ROS2 — ``object.__new__``, ход идёт через настоящий
``_run_turn`` (харнесс как в test_issue_2888_*).
"""

from __future__ import annotations

import asyncio
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import rob_box_voice.dialogue_node as dialogue_node_module  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

SASHA_ID = "3450f4fb-0000-0000-0000-000000000000"
# Дословно из run 35923951507.
N705_TEXT = "робот ты еще здесь помнишь меня"
N705_LLM = "Здравствуй! Да, я здесь. Ты мне знаком."
N706_TEXT = "робот нет я не саша ты обознался"
QUESTION = "Саша, это ты?"


def _single():
    return {
        "is_known": True,
        "speaker_id": SASHA_ID,
        "name": None,
        "tentative_name": "Саша",
        "tentative_conf": 0.93,
        "tentative_kind": "single",
        "confidence": 0.93,
    }


class _Result:
    def __init__(self, text):
        self.spoken_text = text
        self.tools_called = ()
        self.error = None


class _Clock:
    """``time`` модуля dialogue_node: ручной ``monotonic``, остальное —
    настоящий ``time`` (event loop ``asyncio.run`` живёт на глобальном)."""

    def __init__(self, now: float) -> None:
        self.now = now

    def monotonic(self) -> float:
        return self.now

    def __getattr__(self, name):
        import time as _time

        return getattr(_time, name)


@pytest.fixture()
def clock(monkeypatch):
    fake = _Clock(now=1790199838.0)
    monkeypatch.setattr(dialogue_node_module, "time", fake)
    return fake


class _UtteranceSpeaker:
    """Биометрия по utterance_id: каждая реплика человека — tentative
    «Саша» (как в живом прогоне, голос Антона похож на Сашу)."""

    async def resolve(self, utterance_id, timeout):
        return _single()


def _node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._utterance_speaker = _UtteranceSpeaker()
    n._speaker_resolve_timeout_sec = 1.0
    n._publish_speaker_observation = MagicMock()
    n._speaker_register_pub = MagicMock()
    n._speaker_merge_pub = MagicMock()
    n._identity_confirmations = {}
    n._pending_identity_hint = None
    # Прод-значения из config/dialogue_node.yaml.
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    n.tts = []
    n.llm_reply = "Хорошо."
    n._speak_direct = MagicMock(
        side_effect=lambda text, language=None: n.tts.append(text)
    )
    n._handle_result = MagicMock(
        side_effect=lambda result, **_kw: n.tts.append(result.spoken_text)
    )

    async def prepare(**kw):
        user_input = await n._apply_speaker_identity(
            kw["user_input"], None, kw.get("utterance_id")
        )
        dynamic = "\n".join(n._pending_identity_hint_lines())
        return user_input, dynamic

    async def invoke_llm(**_kw):
        return _Result(n.llm_reply)

    n._prepare_user_input_context = prepare
    n._invoke_llm_with_telemetry = invoke_llm
    n._reset_turn_retry_budgets = MagicMock()
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    # S7 — настоящий дренаж очереди; следующий ход ловим, а не пускаем.
    n._pending_user_messages = []
    n.dispatched = []
    n._dispatch_turn = MagicMock(
        side_effect=lambda text, **kw: n.dispatched.append((text, kw))
    )
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    return n


def _turn(n, text, **kw):
    asyncio.run(n._run_turn(text, **kw))


def _answer_lines(n):
    return [
        str(c.args[0])
        for c in n.get_logger().info.call_args_list
        if "identity answer read" in str(c.args[0])
    ]


def _state(n):
    return n._identity_confirmations[SASHA_ID]


def _second_question_asked(n, clock):
    """n702 → n703 → n704 → пауза 150 с → n705 (повторный вопрос)."""
    n.llm_reply = "С возвращением!"
    _turn(n, "робот ты меня узнаешь", utterance_id="utt-702")
    assert n.tts == [QUESTION]

    clock.now += 57.0
    n.llm_reply = "Рад тебя слышать, Саша."
    _turn(n, "да это я", utterance_id="utt-703")
    assert _state(n)["confirmed"] is True

    clock.now += 66.5
    n.llm_reply = "Помню, ты Саша."
    _turn(n, "что ты обо мне запомнил", utterance_id="utt-704")

    clock.now += 150.0 + 60.0
    n.llm_reply = N705_LLM
    _turn(n, N705_TEXT, utterance_id="utt-705")
    assert n.tts[-1] == QUESTION, n.tts
    assert _state(n)["asked"] is True
    assert _state(n)["confirmed"] is None
    assert len(_answer_lines(n)) == 1  # только n703


class TestAnswerOnlyFromReplyAfterQuestion:
    def test_drained_phrase_from_asking_turn_is_not_the_answer(self, clock):
        """Регресс n705/n706: фраза, пришедшая пока шёл ход n705 (S7),
        дренируется сразу после вопроса — это не ответ на него."""
        n = _node()
        n.llm_reply = "С возвращением!"
        _turn(n, "робот ты меня узнаешь", utterance_id="utt-702")
        clock.now += 57.0
        _turn(n, "да это я", utterance_id="utt-703")
        clock.now += 66.5
        _turn(n, "что ты обо мне запомнил", utterance_id="utt-704")
        clock.now += 210.0
        # Во время хода n705 STT прислал ещё одну фразу — она в очереди S7.
        n._pending_user_messages.append((N705_TEXT, clock.now))
        n.llm_reply = N705_LLM
        _turn(n, N705_TEXT, utterance_id="utt-705")
        assert n.tts[-1] == QUESTION
        # Дренаж сразу после вопроса — ход без utterance_id.
        assert len(n.dispatched) == 1
        text, kw = n.dispatched[0]
        assert kw.get("utterance_id") is None

        clock.now += 0.024
        n.llm_reply = "Я тут."
        _turn(n, text, **kw)

        assert len(_answer_lines(n)) == 1, _answer_lines(n)
        assert _state(n)["confirmed"] is None

        # Настоящий ответ n706 — следующая реплика человека.
        clock.now += 62.5
        n.llm_reply = "Понял, извини."
        _turn(n, N706_TEXT, utterance_id="utt-706")

        lines = _answer_lines(n)
        assert len(lines) == 2, lines
        assert "answer=False -> confirmed=False" in lines[-1]
        assert _state(n)["confirmed"] is False
        assert n._current_speaker.get("name") is None

    def test_synthetic_retry_after_question_is_not_the_answer(self, clock):
        """Синтетический ретрай гарда после вопроса (без новой реплики)."""
        n = _node()
        _second_question_asked(n, clock)

        clock.now += 0.024
        _turn(
            n,
            "[CRITICAL] ты не вызвал инструмент, повтори",
            is_synthetic=True,
        )
        assert len(_answer_lines(n)) == 1
        assert _state(n)["confirmed"] is None

        clock.now += 62.5
        _turn(n, N706_TEXT, utterance_id="utt-706")
        lines = _answer_lines(n)
        assert len(lines) == 2, lines
        assert "answer=False -> confirmed=False" in lines[-1]

    def test_yes_after_question_still_confirms(self, clock):
        """#2809 не сломан: «да» следующей репликой подтверждает имя."""
        n = _node()
        _second_question_asked(n, clock)

        clock.now += 40.0
        _turn(n, "да это я", utterance_id="utt-706")
        assert _state(n)["confirmed"] is True
        assert n._current_speaker["name"] == "Саша"


# ─────────────────────────────────────────────────────────────────────────────
#  Вторая половина #2914: ретрай гарда не говорит поверх вопроса
# ─────────────────────────────────────────────────────────────────────────────

# Дословно из docker logs voice-assistant, run 35923951507 (комментарий в
# issue #2914): ретрай #1777/#1762 по memory_search озвучил имя как факт
# через 4 с после «Саша, это ты?».
RETRY_LLM = (
    "Здравствуй! Я здесь, всё в порядке. Помню: ты Саша, "
    "чинишь технику по вечерам."
)
N705_RAW = "ты еще здесь помнишь меня"


def _node_with_real_tool_guard():
    """Как ``_node``, но гард #1777/#1762 настоящий: он сам решает, что
    «ты еще здесь помнишь меня» требует memory_search, и диспатчит
    синтетический ретрай (его ловим в ``n.dispatched``)."""
    n = _node()
    del n._apply_tool_skipped_guard
    n._tool_retry_used = False
    n._synthetic_retries_left = 3
    n._reopen_dialogue_for_retry = MagicMock()
    return n


def _run_dispatched(n):
    """Выполнить задиспатченные ходы так, как их выполнил бы loop."""
    while n.dispatched:
        text, kw = n.dispatched.pop(0)
        kw = {k: v for k, v in kw.items() if k != "was_idle"}
        _turn(n, text, **kw)


class TestNoGuardRetryOverIdentityQuestion:
    def test_memory_search_retry_does_not_speak_over_question(self, clock):
        """Лог n705: вопрос → ретрай memory_search → «Помню: ты Саша».
        В TTS должен остаться только вопрос."""
        n = _node_with_real_tool_guard()
        n.llm_reply = N705_LLM
        _turn(n, N705_TEXT, utterance_id="utt-705", raw_user_command=N705_RAW)
        assert n.tts == [QUESTION], n.tts

        n.llm_reply = RETRY_LLM
        _run_dispatched(n)

        assert n.tts == [QUESTION], n.tts
        # Вопрос по-прежнему ждёт ответа человека.
        assert _state(n)["confirmed"] is None

    def test_tool_retry_still_works_without_identity_question(self, clock):
        """#1777/#1762 не сломан: без вопроса о личности ретрай идёт."""
        n = _node_with_real_tool_guard()

        class _Known:
            async def resolve(self, utterance_id, timeout):
                return {"is_known": True, "speaker_id": SASHA_ID,
                        "name": "Саша", "confidence": 0.95}

        n._utterance_speaker = _Known()
        n.llm_reply = "Да, я здесь."
        _turn(n, N705_TEXT, utterance_id="utt-705", raw_user_command=N705_RAW)
        assert n.tts == ["Да, я здесь."], n.tts
        assert len(n.dispatched) == 1, n.dispatched
        assert n.dispatched[0][1].get("is_synthetic") is True
