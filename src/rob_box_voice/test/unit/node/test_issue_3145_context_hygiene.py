"""Issue #3145 — гигиена контекста: история после ретраев гуардов.

Живой лог 28.09 (``voice-assistant_1824-1852.log``): ретраи Bug D/E и
TurnGuards не отзывали отвергнутый ответ — ``discard_last_reply`` звали
только music-гуарды. После ответа ретрая в окне стояли два assistant подряд
(28 мест), «Клубняк в клубе, погнали дальше!» ~10 ходов висел ответом на
«ты диджей Снупдог». Плюс префикс «[🎧 Музыкальный режим активен …]» жил в
user-ходах истории (82 штуки) — DJ-преамбула удалена в ADR-0149 PR-13a.

Здесь нода работает с НАСТОЯЩИМ ``AgentCore`` (фейковая LLM) на отдельном
asyncio-лупе — ровно как в проде: гуард зовёт ``_dispatch_turn``, отзыв и
ретрай планируются на луп через ``run_coroutine_threadsafe``.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_llm.provider import LLMResponse
from rob_box_voice.dialogue_node import DialogueNode


class _FakeLLM:
    name = "fake_llm"

    def __init__(self, replies: list[str]) -> None:
        self.replies = list(replies)
        self.calls: list[list[Any]] = []

    async def complete(self, messages: Any = None, **_kw: Any) -> Any:
        self.calls.append(list(messages or []))
        return LLMResponse(content=self.replies.pop(0), tool_calls=())

    async def aclose(self) -> None:
        return None


class _NoTools:
    name = "no_tools"

    async def discover(self) -> tuple:
        return ()

    async def execute(self, call: Any) -> Any:  # pragma: no cover
        raise AssertionError("тулов в этом тесте нет")

    async def aclose(self) -> None:
        return None


class _NoMemory:
    async def save_fact(self, *a: Any, **k: Any) -> None:
        return None

    async def search_facts(self, *a: Any, **k: Any) -> list:
        return []

    async def aclose(self) -> None:
        return None


@pytest.fixture
def loop():
    lp = asyncio.new_event_loop()
    thread = threading.Thread(target=lp.run_forever, daemon=True)
    thread.start()
    yield lp
    lp.call_soon_threadsafe(lp.stop)
    thread.join(timeout=5)
    lp.close()


def _setup(loop, replies: list[str]):
    llm = _FakeLLM(replies)
    core = AgentCore(
        llm=llm, tools=_NoTools(), memory=_NoMemory(),
        dsm=DialogueStateMachine(), system_prompt="ПРОМПТ",
        history_trim_limit=20,
    )
    node = object.__new__(DialogueNode)
    node.get_logger = lambda: MagicMock()
    node._core = core
    node._loop = loop
    node._pending_music_cleanup = False
    node._session_started_at = None
    node._dsm = MagicMock()
    node._publish_state = lambda: None
    node._reopen_dialogue_for_retry = lambda: None
    node._retry_dispatched_in_turn = False
    node._synthetic_retries_left = DialogueNode.DEFAULT_SYNTHETIC_RETRIES
    node._action_claim_retry_used = False
    node._turn_state = None
    retry_done = threading.Event()

    async def fake_run_turn(user_input, **kw):
        # Прод: _run_turn → _invoke_llm_with_telemetry → process_input.
        core._dsm.on_event(DialogueEvent.WAKE_WORD)
        await core.process_input(
            user_input, is_synthetic=kw.get("is_synthetic", False)
        )
        retry_done.set()

    node._run_turn = fake_run_turn
    return node, core, llm, retry_done


def _user_turn(loop, core, text: str) -> None:
    async def _go():
        core._dsm.on_event(DialogueEvent.WAKE_WORD)
        await core.process_input(text)

    asyncio.run_coroutine_threadsafe(_go(), loop).result(timeout=5)


def _assert_clean_window(core, *, rejected: str, accepted: str, request: str):
    window = [(t.role, t.content) for t in core._turn_window]
    assert window == [("user", request), ("assistant", accepted)], window
    roles = [role for role, _ in window]
    assert not any(
        a == b == "assistant" for a, b in zip(roles, roles[1:])
    ), "два assistant подряд"
    assert all(rejected != content for _, content in window)


def _retry_prompt_messages(llm) -> list[Any]:
    return llm.calls[-1]


class TestRejectedReplyLeavesHistory:
    def test_bug_e_retry(self, loop):
        node, core, llm, done = _setup(
            loop, ["Точка удалена.", "Удаляю точку «кухня»."]
        )
        _user_turn(loop, core, "удали точку кухня")
        fired = node._check_unbacked_action_claim_and_retry(
            spoken="Точка удалена.",
            user_input="удали точку кухня",
            tools_called=(),
        )
        assert fired is True, "Bug E должен поймать «Точка удалена.» без тула"
        assert done.wait(5)
        _assert_clean_window(
            core, rejected="Точка удалена.",
            accepted="Удаляю точку «кухня».", request="удали точку кухня",
        )
        sent = [m.content for m in _retry_prompt_messages(llm)]
        assert "Точка удалена." not in sent[:-1], sent
        assert "удали точку кухня" in sent, sent

    def test_bug_d_babble_retry(self, loop):
        node, core, llm, done = _setup(
            loop, ["Погнали, сейчас зачитаю!", "Йо, вот мой куплет."]
        )
        _user_turn(loop, core, "зачитай рэп про кота")
        fired = node._check_babble_and_retry(
            spoken="Погнали, сейчас зачитаю!",
            user_input="зачитай рэп про кота",
            tools_called=(),
        )
        assert fired is True, "Bug D должен поймать «Погнали…» без тула"
        assert done.wait(5)
        _assert_clean_window(
            core, rejected="Погнали, сейчас зачитаю!",
            accepted="Йо, вот мой куплет.", request="зачитай рэп про кота",
        )

    def test_retry_on_unpersisted_turn_keeps_older_reply(self, loop):
        """Ход, который ничего не записал (DJ-переход, тихий ответ), —
        отзывать нечего: прошлый ответ юзеру остаётся на месте."""
        node, core, llm, done = _setup(loop, ["Привет!", "Ок."])
        _user_turn(loop, core, "привет")

        async def _dj_turn():
            core._dsm.on_event(DialogueEvent.WAKE_WORD)
            await core.process_input("[DJ_AUTO] переход", is_dj_auto=True)

        llm.replies.insert(1, "Клубняк!")
        asyncio.run_coroutine_threadsafe(_dj_turn(), loop).result(timeout=5)
        node._dispatch_turn("[CRITICAL] retry", is_synthetic=True)
        assert done.wait(5)
        contents = [t.content for t in core._turn_window]
        assert "Привет!" in contents, contents


class TestUserWordsVerbatimInHistory:
    """Реплика уходит в ход дословно — без префиксов состояния.

    ADR-0149 PR-13a: DJ-преамбула («[🎧 Музыкальный режим активен …]») удалена
    вместе с DJ-контроллером; состояние музыки — в ``<music_state>``.
    """

    def test_dispatch_keeps_user_words_verbatim(self):
        node = object.__new__(DialogueNode)
        node.get_logger = lambda: MagicMock()
        node._verbose_llm = False
        node._dispatch_turn = MagicMock()
        node._dispatch_cleaned(
            clean="горный король погромче", was_idle=False,
            speaker_tag=None, speaker_duration_s=0.0, from_tg=False,
            backlog_pending=False,
        )
        args, _kwargs = node._dispatch_turn.call_args
        assert args[0] == "горный король погромче"
