"""ADR-0150 PR-6 (#3696) — Tier-1 в ``DialogueNode`` v1: реальные ``_on_stt`` → ``Tier1Step`` → ``Tier1Runner``.

Харнесс без ROS2 (``object.__new__``, как ``test_issue_3134_media_router_node``): мок — исполнитель тулов, ход LLM
(``_dispatch_turn``), речь (``_speak_direct``), тишина (``_handle_silence``) и Chooser спорного стопа.
Проверяется: стоп — без LLM и с остановкой музыки; в SILENCED — работает, но молчит; громкость — канал по снимку;
голос — по реестру; спорный стоп (В19) — речь не рвётся, при ``part_of_command`` реплика идёт дальше.
"""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_llm.provider import ToolResult

from rob_box_voice.core.dialogue_text import DEFAULT_WAKE_WORDS
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module
from ros_stubs import make_utterance  # noqa: E402


class _FakeExecutor:
    def __init__(self) -> None:
        self.calls = []

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return ToolResult(tool_call_id=call.id, content=json.dumps({"success": True}), is_error=False)


class _Chooser:
    def __init__(self, verdict: str) -> None:
        self.verdict, self.asked = verdict, []

    def decide(self, text):
        self.asked.append(text)
        return self.verdict, "model"


def _make_node(*, playing=False, silenced=False, speaking=False, verdict="part_of_command"):
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._wake_words = list(DEFAULT_WAKE_WORDS)
    n._command_intent_gate_enabled = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = new_llm_skip_counter()
    n._maybe_log_skip_summary = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.SILENCED if silenced else DialogueStateKind.IDLE
    n._cancel_run = MagicMock()
    n._handle_silence = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._speak_direct = MagicMock()
    n._verbose_llm = False
    n._active_tg_chat_id = None
    n._intent_pub = MagicMock()
    n._music_player_state = MusicPlayerState(state="playing" if playing else "idle", dj=False,
                                             dj_info={"enabled": False, "title": "Still Dre"})
    n._speaking_watch = SimpleNamespace(speaking=speaking)
    n._stop_chooser = _Chooser(verdict)
    n._tool_executor = _FakeExecutor()
    n._loop = MagicMock()
    return n


@pytest.fixture
def run_plans(monkeypatch):
    pending = []
    monkeypatch.setattr(dialogue_node_module.asyncio, "run_coroutine_threadsafe",
                        lambda coro, loop: pending.append(coro) or MagicMock())

    def _drain():
        loop = asyncio.new_event_loop()
        try:
            while pending:
                loop.run_until_complete(pending.pop(0))
        finally:
            loop.close()

    return _drain


def _stt(node, text):
    node._on_stt(make_utterance(text))


def _speech_stopped(node) -> bool:
    return any(c.kwargs.get("stop_tts", True) for c in node._cancel_run.call_args_list)


def test_stop_stops_speech_motion_and_music_without_llm(run_plans):
    n = _make_node(playing=True)
    _stt(n, "Робот, стоп")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert _speech_stopped(n)
    assert n._intent_pub.publish.call_args[0][0].kind == "stop_motion"
    assert n._tool_executor.calls == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    n._speak_direct.assert_not_called()
    assert n._llm_skipped_counter["tier1"] == 1


def test_stop_in_silence_works(run_plans):
    n = _make_node(silenced=True)
    _stt(n, "Робот, хватит")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert _speech_stopped(n)
    n._handle_silence.assert_not_called()  # уже тишина: без «Хорошо, молчу.»
    n._speak_direct.assert_not_called()


def test_silence_outside_silence_is_the_old_path(run_plans):
    n = _make_node()
    _stt(n, "Робот, хватит")
    n._handle_silence.assert_called_once()
    n._dispatch_turn.assert_not_called()
    assert n._llm_skipped_counter["silence_command"] == 1


@pytest.mark.parametrize("silenced, spoken", [(False, True), (True, False)])
def test_volume_while_speaking_is_tts_and_silent_in_silence(run_plans, silenced, spoken):
    n = _make_node(speaking=True, silenced=silenced)
    _stt(n, "Робот, громче")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._tool_executor.calls == [("set_volume", {"action": "louder"})]
    assert n._speak_direct.called is spoken


def test_volume_while_music_is_music(run_plans):
    n = _make_node(playing=True, speaking=True)
    _stt(n, "Робот, громче")
    run_plans()
    assert n._tool_executor.calls == [("set_music_volume", {"action": "louder"})]
    n._speak_direct.assert_called_once_with("Сделал музыку громче.")


def test_voice_by_registry_without_llm(run_plans):
    n = _make_node()
    _stt(n, "Робот, включи голос надёжного мужика")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._tool_executor.calls == [("set_voice", {"voice": "Russian_ReliableMan"})]


def test_disputed_stop_part_of_command_goes_on_and_speech_is_not_cut(run_plans):
    n = _make_node(verdict="part_of_command")
    _stt(n, "Робот, расскажи сказку про то как ёж сказал хватит")
    assert not _speech_stopped(n)  # до решения Chooser речь не рвётся
    run_plans()
    assert n._stop_chooser.asked == ["расскажи сказку про то как ёж сказал хватит"]
    n._dispatch_turn.assert_called_once()
    assert not _speech_stopped(n)
    n._handle_silence.assert_not_called()


@pytest.mark.parametrize("text, silence", [("Робот, спой другую и хватит", True), ("Робот, покажи анимацию стоп", False)])
def test_disputed_stop_judged_stop_stops(run_plans, text, silence):
    # Вид — от грамматики: «хватит» — тишина (как ясное «хватит»), «стоп» — стоп всего.
    n = _make_node(verdict="stop")
    _stt(n, text)
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._handle_silence.called is silence
    assert _speech_stopped(n) is not silence


def test_stop_then_new_command_runs_both(run_plans):
    n = _make_node()
    _stt(n, "Робот, всё, хватит, расскажи анекдот")
    run_plans()
    assert _speech_stopped(n)
    n._handle_silence.assert_not_called()  # стоп с продолжением — не тишина
    n._dispatch_turn.assert_called_once()
    assert "расскажи анекдот" in str(n._dispatch_turn.call_args)
