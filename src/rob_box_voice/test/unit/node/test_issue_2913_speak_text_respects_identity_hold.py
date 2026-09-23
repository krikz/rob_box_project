"""test_issue_2913_speak_text_respects_identity_hold.py

Issue #2913 — реплика ``speak_text`` обходила механизм «придержать и
заменить ответ хода» (#2828/#2888/#2908): тот заменял только итоговый
текст хода, а ``speak_text`` уходила в TTS сразу.

Живой лог (E2E акт 2c, run 35923157899, n722)::

    .642 [mcp_server]      📥 register_speaker {'name': 'Борис', 'utterance_id': '0e3b0bbc7d52'}
    .659 [speaker_id_node] ⚠️ Speaker 'Борис' — голос похож на уже известного 'Саша' (score=0.700)
    .679 [mcp_server]      📥 speak_text {'text': 'Здравствуй, Борис! Приятно познакомиться.'}
    .686 [tts_node]        🔊 TTS: text='Здравствуй, Борис! Приятно познакомиться.'
    .696 [dialogue_node]   👥 переспрашиваю про личность: conflict=True held_until_turn_end=True
    0.093 [dialogue_node]  👥 [issue #2828] ответ хода заменён переспросом про личность
    0.097 [tts_node]       🔊 TTS: text='Твой голос очень похож на голос, ... Вы разные люди ...'

Стенд: ход идёт через настоящий ``_run_turn``; тулы — через настоящий
``SchedulerToolExecutor`` (TaskScheduler, VOICE-канал), собранный самой
нодой (``_make_scheduler_executor``); ack регистрации — через настоящий
``_on_speaker_result``. Замоканы LLM (сценарий тул-вызовов), MCP-провайдер
(``speak_text`` пишет в список ``tts``), гарды и DSM.
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
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

from rob_box_llm.provider import ToolCall, ToolResult  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from rob_box_voice.scheduler.task_scheduler import ChannelKind  # noqa: E402

BORIS_ID = "e6daabb1-0000-0000-0000-000000000000"
SASHA_ID = "ca4f6b3a-0000-0000-0000-000000000000"
UTTERANCE = "0e3b0bbc7d52"
GREETING = "Здравствуй, Борис! Приятно познакомиться."
CONFLICT_ACK = {
    "event": "registered",
    "name": "Борис",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
    "utterance_id": UTTERANCE,
    "voice_conflict": {"name": "Саша", "speaker_id": SASHA_ID, "score": 0.7},
}
PLAIN_ACK = {
    "event": "registered",
    "name": "Борис",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
    "utterance_id": UTTERANCE,
}
REFUSAL_ACK = {
    "event": "register_error",
    "error": "too_short",
    "name": "Борис",
    "duration_s": 2.97,
    "min_required_s": 3.0,
    "utterance_id": UTTERANCE,
}
INTRO = "здравствуй, меня зовут Борис"


def _msg(payload: dict):
    return type("Msg", (), {"data": json.dumps(payload, ensure_ascii=False)})()


class _Result:
    """Ход сказал всё через speak_text — свободного текста нет."""

    spoken_text = ""
    tools_called = ("register_speaker", "speak_text")
    error = None


class _FakeMcp:
    """MCP-провайдер: speak_text → TTS, register_speaker → «pending»."""

    def __init__(self, node, register_reply=None):
        self._node = node
        self._register_reply = register_reply or {
            "registered_name": "Борис", "speaker_id": "pending",
        }
        self.tts_at: list = []

    async def execute(self, call: ToolCall) -> ToolResult:
        if call.name == "speak_text":
            self._node.tts.append(call.arguments["text"])
            self.tts_at.append(time.monotonic())
        return ToolResult(
            tool_call_id=call.id,
            content=repr(self._register_reply)
            if call.name == "register_speaker" else "{'spoken': True}",
            is_error=False,
        )


def _call(tool: str, **args) -> ToolCall:
    return ToolCall(id=f"call-{tool}", name=tool, arguments=args)


def _node(script):
    """``script(n, executor)`` — корутина «LLM этого хода»: зовёт тулы через
    исполнитель ноды и шлёт ack, как это делает ROS-колбэк."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._speaker_merge_pub = MagicMock()
    n.tts = []
    n._speak_direct = MagicMock(
        side_effect=lambda text, language=None: n.tts.append(text)
    )
    n._handle_result = MagicMock(
        side_effect=lambda result, **_kw: (
            n.tts.append(result.spoken_text) if result.spoken_text else None
        )
    )
    n.mcp = _FakeMcp(n)
    n.executor = n._make_scheduler_executor(n.mcp)

    async def invoke_llm(**_kw):
        await script(n, n.executor)
        # Ход кончается, когда голосовой канал выговорился.
        await n.executor._scheduler.wait_until_idle(ChannelKind.VOICE)
        return _Result()

    async def prepare(**kw):
        return kw["user_input"], "<system_context/>"

    n._invoke_llm_with_telemetry = invoke_llm
    n._prepare_user_input_context = prepare
    n._reset_turn_retry_budgets = MagicMock()
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._drain_pending_user_messages = MagicMock(return_value=False)
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    # Сбой стенда не маскируется под «задумался» — падает тест.
    n._handle_llm_error = _reraise
    return n


def _reraise(exc, *_a, **_kw):
    raise exc


def _register_then_speak(ack, ack_delay_s=0.05):
    """Порядок из лога n722: register_speaker → speak_text (в очереди
    голосового канала) → через ``ack_delay_s`` приходит исход регистрации."""

    async def script(n, ex):
        await ex.execute(_call("register_speaker", name="Борис"))
        await ex.execute(_call("speak_text", text=GREETING))
        # Голосовой канал успевает взять реплику (до фикса — прямо в TTS).
        await asyncio.sleep(ack_delay_s)
        if ack is not None:
            n._on_speaker_result(_msg(ack))

    return script


def _is_greeting(text: str) -> bool:
    return "Приятно познакомиться" in text


def test_conflict_after_speak_text_only_question_is_spoken():
    """Регресс n722: в TTS только вопрос «вы разные люди?», приветствия нет."""
    n = _node(_register_then_speak(CONFLICT_ACK))

    asyncio.run(n._run_turn(INTRO))

    assert not any(_is_greeting(t) for t in n.tts), n.tts
    assert len(n.tts) == 1, n.tts
    assert "Саша" in n.tts[0] and "разные" in n.tts[0], n.tts


def test_conflict_from_ros_thread_only_question_is_spoken():
    """То же, но ack приходит из другого потока (ROS-колбэк), как на роботе."""

    async def script(n, ex):
        await ex.execute(_call("register_speaker", name="Борис"))
        await ex.execute(_call("speak_text", text=GREETING))
        threading.Timer(
            0.03, n._on_speaker_result, args=(_msg(CONFLICT_ACK),)
        ).start()
        await asyncio.sleep(0.1)

    n = _node(script)

    asyncio.run(n._run_turn(INTRO))

    assert not any(_is_greeting(t) for t in n.tts), n.tts
    assert len(n.tts) == 1 and "разные" in n.tts[0], n.tts


def test_refusal_2908_replaces_speak_text_too():
    """#2908: отказ регистрации — в TTS только «Не расслышал…»."""
    n = _node(_register_then_speak(REFUSAL_ACK))

    asyncio.run(n._run_turn(INTRO))

    assert not any(_is_greeting(t) for t in n.tts), n.tts
    assert len(n.tts) == 1 and "Не расслышал" in n.tts[0], n.tts


def test_plain_registration_speaks_greeting_once():
    """Регистрация без конфликта — приветствие звучит (ровно один раз)."""
    n = _node(_register_then_speak(PLAIN_ACK))

    asyncio.run(n._run_turn(INTRO))

    assert n.tts == [GREETING], n.tts


def test_speak_text_before_register_in_batch_still_waits():
    """LLM назвала speak_text раньше register_speaker в одной пачке —
    реплика всё равно не уходит раньше исхода."""

    async def script(n, ex):
        await ex.execute(_call("speak_text", text=GREETING))
        await ex.execute(_call("register_speaker", name="Борис"))
        await asyncio.sleep(0.05)
        n._on_speaker_result(_msg(CONFLICT_ACK))

    n = _node(script)

    asyncio.run(n._run_turn(INTRO))

    assert not any(_is_greeting(t) for t in n.tts), n.tts


def test_tentative_2888_question_replaces_speak_text():
    """#2888: вопрос «Саша, это ты?» придержан до LLM — реплика
    speak_text с именем-гипотезой как фактом не звучит."""
    name_as_fact = "С возвращением, Саша. Узнал, приятель."

    async def script(n, ex):
        await ex.execute(_call("speak_text", text=name_as_fact))
        await asyncio.sleep(0.05)

    n = _node(script)

    async def prepare(**kw):
        n._queue_identity_question(
            {"kind": "tentative", "question": "Саша, это ты?", "name": "Саша"}
        )
        return kw["user_input"], "<system_context/>"

    n._prepare_user_input_context = prepare

    asyncio.run(n._run_turn("робот, ты меня узнал?"))

    assert n.tts == ["Саша, это ты?"], n.tts


def test_plain_turn_speak_text_is_not_delayed():
    """Обычный ход без register_speaker: реплика уходит в TTS сразу, гейт
    не ждёт (таймаут гейта задран, чтобы любое ожидание было видно)."""
    started: list = []

    async def script(n, ex):
        started.append(time.monotonic())
        await ex.execute(_call("speak_text", text="Сейчас без пяти восемь."))
        await asyncio.sleep(0.05)

    n = _node(script)
    n._turn_speech_gate()._timeout_s = 30.0

    asyncio.run(n._run_turn("который час?"))

    assert n.tts == ["Сейчас без пяти восемь."], n.tts
    assert n.mcp.tts_at[0] - started[0] < 0.03, n.mcp.tts_at[0] - started[0]


def test_register_without_request_does_not_delay_speech():
    """register_speaker(name=None) — «спроси имя»: запроса в speaker_id_node
    нет, ack не будет — «Как тебя зовут?» не ждёт таймаута."""
    started: list = []

    async def script(n, ex):
        started.append(time.monotonic())
        await ex.execute(_call("register_speaker", name=None))
        await ex.execute(_call("speak_text", text="Как тебя зовут?"))
        await asyncio.sleep(0.05)

    n = _node(script)
    n.mcp._register_reply = {"ask_required": True, "name": None}
    n._turn_speech_gate()._timeout_s = 30.0

    asyncio.run(n._run_turn("привет"))

    assert n.tts == ["Как тебя зовут?"], n.tts
    assert n.mcp.tts_at[0] - started[0] < 0.03, n.mcp.tts_at[0] - started[0]


def test_lost_ack_releases_speech_after_timeout():
    """Ack так и не пришёл — робот не немеет: реплика звучит по таймауту."""
    n = _node(_register_then_speak(None))
    n._turn_speech_gate()._timeout_s = 0.2

    asyncio.run(n._run_turn(INTRO))

    assert n.tts == [GREETING], n.tts
