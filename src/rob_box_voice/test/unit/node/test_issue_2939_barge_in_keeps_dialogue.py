"""Issue #2939 — ход после barge-in-отмены возвращался пустым за 1 мс.

Живой лог (E2E акт 2b, run 35948844238)::

    15.85 [dialogue_node] 🛑 Cancel: new STT input
    15.86 [dialogue_node] 🛑 Turn cancelled (barge-in)
    15.87 [dialogue_node] 🚀 [turn] calling process_input: '... привет я борис ...'
    15.87 [dialogue_node] ✅ [turn] process_input returned: spoken='' finish_reason=None
    15.87 [dialogue_node] ⚠️ Empty assistant response → «Принял.»

Механизм (доказан этим тестом): приём новой фразы (``_on_stt``, поток ROS)
отменяет ход и переводит DSM в DIALOGUE для НОВОГО хода. Отменённый ход
в своём ``finally`` (``_finalize_turn_dsm``) видит DIALOGUE и закрывает
сессию ``DIALOGUE_END`` → IDLE. Новый ход зовёт ``AgentCore.process_input``,
тот видит IDLE и LLM не вызывает вовсе — пустой ответ за миллисекунду.

Стенд: настоящие ``_run_turn``, ``_cancel_run``, ``_DialogueSttHost``
(cancel + DSM-переходы ``DispatchTriggerStep``), настоящая
``DialogueStateMachine`` и настоящий ``AgentCore`` (его LLM-гейт по
состоянию DSM). LLM — заглушка: первый ход висит (как minimax в #2939),
второй отвечает.
"""

from __future__ import annotations

import asyncio
import threading
from collections import deque
from typing import Any, AsyncIterator
from unittest.mock import AsyncMock, MagicMock

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_llm.provider import LLMChunk, LLMProvider, LLMResponse, ProviderCapabilities
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode, _DialogueSttHost


class _HangThenAnswer(LLMProvider):  # type: ignore[misc]
    """Первый запрос висит (minimax #2939), следующие отвечают."""

    name = "minimax"

    def __init__(self) -> None:
        self.calls = 0
        self.started = asyncio.Event()

    @property
    def capabilities(self) -> ProviderCapabilities:
        return ProviderCapabilities(text=True, streaming_text=True, tools=True, streaming_tools=True)

    async def complete(self, messages: Any, *, tools: Any = (), settings: Any = None) -> LLMResponse:
        return await self._answer()

    async def stream(self, messages: Any, *, tools: Any = (), settings: Any = None) -> AsyncIterator[LLMChunk]:
        resp = await self._answer()
        yield LLMChunk(content_delta=resp.content, finish_reason="stop")

    async def _answer(self) -> LLMResponse:
        self.calls += 1
        if self.calls == 1:
            self.started.set()
            await asyncio.sleep(3600)
        return LLMResponse(content="Привет, Борис!", finish_reason="stop")

    async def aclose(self) -> None:
        return None


def _make_node(llm: _HangThenAnswer) -> DialogueNode:
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_task = None
    n._run_cancelled = False
    n._music_guard = MusicGuard()
    n._pending_music_cleanup = False
    n._track_mode_music_active = False
    n._speaker_id_enabled = False
    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(return_value="<ctx/>")
    n._llm = llm
    n._active_batches = {}
    n._dsm = DialogueStateMachine()
    n._publish_state = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._publish_music_cleanup = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n.get_logger = lambda: MagicMock()
    n._core = AgentCore(
        llm=llm,
        tools=MagicMock(discover=AsyncMock(return_value=())),
        memory=InMemoryStore(),
        dsm=n._dsm,
    )
    n._handle_result = MagicMock()
    n._dispatch_turn = MagicMock()
    n._pending_user_messages = deque()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._tts_control_pub = MagicMock()
    n._effects = MagicMock()
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    return n


def test_turn_after_barge_in_reaches_llm() -> None:
    async def scenario() -> tuple[Any, DialogueStateKind]:
        llm = _HangThenAnswer()
        node = _make_node(llm)
        node._loop = asyncio.get_running_loop()
        # Ход 1 (Саша): wake + фраза → DIALOGUE, LLM висит.
        node._dsm.on_event(DialogueEvent.WAKE_WORD)
        node._dsm.on_event(DialogueEvent.STT_RESULT)
        first = asyncio.ensure_future(
            node._run_turn("а сейчас как меня по твоему зовут", session_epoch=0)
        )
        await asyncio.wait_for(llm.started.wait(), 5)

        # Реплика Бориса: то же, что делает приём STT перед диспатчем.
        host = _DialogueSttHost(node)
        host.cancel_inflight(stop_tts=True)       # BargeInClassifyStep
        host.transition_idle_to_wake()            # DispatchTriggerStep
        host.transition_stt_result()
        await asyncio.gather(first, return_exceptions=True)
        state_before_second = node._dsm.current_state

        # Ход 2 — как в живом логе, сразу после «Turn cancelled».
        await node._run_turn("привет я борис заглянул проверить проводку", session_epoch=0)
        return node._handle_result.call_args, state_before_second

    call, state_before_second = asyncio.run(scenario())
    assert state_before_second == DialogueStateKind.DIALOGUE, (
        f"после barge-in DSM={state_before_second}: отменённый ход закрыл "
        "сессию (DIALOGUE_END), хотя её уже принял следующий ход"
    )
    assert call is not None, "ход Бориса не дошёл до выдачи ответа"
    result = call.args[0]
    assert result.spoken_text == "Привет, Борис!", (
        f"spoken={result.spoken_text!r}: LLM не вызвана, робот скажет «Принял.» (#2939)"
    )
