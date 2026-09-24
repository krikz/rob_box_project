"""test_issue_2967_dj_silence_and_repeat_guard.py — issue #2967.

Живая сессия 24.09.2026 (DJ «Сапог», minimax): 2-3 DJ auto-transition
подряд отвечали ТОЛЬКО текстом без музыкального тула. Ретрай по
``[issue 992 Bug B]`` уже существовал (``MusicGuard``), но каждый ПУСТОЙ
ход всё равно озвучивался — слушатель слышал несколько анонсов подряд
без единого реально запущенного трека.

Two systemic fixes (не под конкретные фразы из лога — товарищ Шифу,
24.09.2026):

1. ``DialogueNode._dj_giveup_silent_in_turn`` — когда Bug-B retry-budget
   на ДАННОМ DJ-переходе исчерпан (``MusicGuardVerdictKind.SKIP_NOT_APPLICABLE``,
   ``reason="bug_b_budget_exhausted"``), ход всё равно не озвучивается:
   он не завёл музыку, значит не финальный ход сета. Опирается на ФАКТ
   «музыкальный тул не вызван в этом ходе», а не на конкретный текст.
2. ``DialogueNode._repeated_music_call_args`` — сравнение аргументов
   ПОСЛЕДОВАТЕЛЬНЫХ вызовов ``compose_music`` (кросс-ходовое состояние),
   используется guard'ом #2549 (см. ``test_issue_2549_universal_action_claim_guard.py``
   в ``test/unit/core`` для чистой логики; здесь — только интеграция с
   реальным ``DialogueNode``, которому нужны rclpy-заглушки).

Этот файл прогоняет НАСТОЯЩИЙ ``_run_turn`` + ``_apply_music_guard`` +
``MusicGuard`` (как ``test_issue_2874_silent_retries.py``), считая, что
реально ушло в TTS.
"""

from __future__ import annotations

import asyncio
import threading
from collections import deque
from unittest.mock import AsyncMock, MagicMock, patch

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode


class _Result:
    def __init__(self, spoken, tools=(), music_call_args=None):
        self.spoken_text = spoken
        self.tools_called = tuple(tools)
        self.error = None
        self.music_call_args = music_call_args


def _make_node(replies, *, dj_enabled=True, transition_count=1):
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_cancelled = False
    n._music_guard = MusicGuard(max_user_retries=3, max_dj_retries=2)
    n._pending_music_cleanup = False
    n._track_mode_music_active = False
    n._speaker_id_enabled = False
    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(return_value="<ctx/>")
    n._llm = MagicMock()
    n._active_batches = {}
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._publish_state = MagicMock()
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._maybe_record_session_end = MagicMock()
    n.get_logger = lambda: MagicMock()
    n._core = MagicMock()
    n._core.process_input = AsyncMock(side_effect=list(replies))
    n._pending_user_messages = deque()
    n._loop = MagicMock()
    n._last_music_call_args = None

    hook = DJHook(
        dispatch=lambda *a, **k: None,
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    n._dj = DJModeController(hook=hook, logger=MagicMock())
    n._dj.state.enabled = dj_enabled
    n._dj.state.transition_count = transition_count

    n._tts = MagicMock()
    n._publish_response = n._tts.response
    n._publish_response_batch = n._tts.batch
    n._publish_response_batch.side_effect = lambda chunks, *a, **k: len(chunks)
    n._speak_direct = n._tts.direct
    n._publish_music_cleanup = n._tts.cleanup
    n._reopen_dialogue_for_retry = MagicMock()

    dispatched: list = []

    def _dispatch_turn_mock(prompt, **kw):
        dispatched.append((prompt, kw))

    n._dispatch_turn = MagicMock(side_effect=_dispatch_turn_mock)

    def _tail(result, **kw):
        n._voice_turn_text(
            result.spoken_text,
            user_input=kw.get("raw_user_command") or kw.get("user_input"),
        )

    n._handle_result = MagicMock(side_effect=_tail)
    return n, dispatched


def _voiced(n):
    return [c for c in n._tts.mock_calls if c[0] in ("response", "batch", "direct")]


def _run(n, text, **kw):
    with patch(
        "rob_box_voice.dialogue_node.asyncio.run_coroutine_threadsafe",
        return_value=MagicMock(),
    ):
        asyncio.run(n._run_turn(text, session_epoch=0, **kw))


def _run_dj_chain(n, dispatched):
    """DJ tick + все Bug-B ретраи, которые он задиспатчил."""
    _run(n, "[DJ_AUTO] start", is_dj_auto=True)
    while dispatched:
        prompt, _kw = dispatched.pop(0)
        _run(n, prompt, is_dj_auto=True, is_synthetic=True)


# ── Part 1: DJ-переход без музыки не звучит, даже когда budget исчерпан ──


class TestDjRetryBudgetExhaustedStaysSilent:
    def test_three_empty_turns_all_silent(self):
        """Живой лог 24.09: 3 пустых хода подряд (max_dj_retries=2) — ни
        один не должен уйти в TTS (только следующий реальный тик сделает
        это, а тут его нет)."""
        n, dispatched = _make_node(
            [
                _Result("Йоу, народ! ДиДжей Сапог на связи!"),
                _Result("Качаем Still Dre — расслабься"),
                _Result("Качаем The Next Episode"),
            ]
        )

        _run_dj_chain(n, dispatched)

        assert n._core.process_input.await_count == 3
        assert _voiced(n) == []

    def test_success_on_last_attempt_is_voiced(self):
        """Контраст: если ПОСЛЕДНИЙ ход (в бюджете) реально запускает
        музыку — он звучит как обычно."""
        n, dispatched = _make_node(
            [
                _Result("Йоу, народ!"),
                _Result("Погнали!", tools=["compose_music"]),
            ]
        )

        _run_dj_chain(n, dispatched)

        voiced = _voiced(n)
        assert len(voiced) == 1
        assert voiced[0] == ("response", ("Погнали!",), {})

    def test_dj_giveup_flag_does_not_defer_dialogue_end(self):
        """``_dj_giveup_silent_in_turn`` НЕ должен путаться с
        ``music_retry_dispatched`` — DSM обязан штатно закрыть диалог
        на исчерпанном ходе (ретрая ведь не будет, ждать нечего)."""
        n, dispatched = _make_node(
            [
                _Result("Йоу, народ!"),
                _Result("Ещё раз мимо."),
            ]
        )

        _run_dj_chain(n, dispatched)

        # Оба хода не вызвали музыку → DSM должен был закрыться штатно
        # на КАЖДОМ (DIALOGUE_END вызывается через MagicMock DSM — здесь
        # проверяем косвенно: on_event(DIALOGUE_END) был вызван хотя бы
        # раз и без зависаний/исключений выше).
        assert n._dsm.on_event.called


# ── Part 3 (integration slice) — repeated compose_music args ────────────


class TestRepeatedMusicCallArgsHelperIntegration:
    """``DialogueNode._repeated_music_call_args`` needs a real (if bare)
    ``DialogueNode`` instance — pure-logic coverage for the detector
    itself lives in ``test/unit/core/test_issue_2549_universal_action_claim_guard.py``."""

    @staticmethod
    def _make_bare_node():
        node = object.__new__(DialogueNode)
        node._last_music_call_args = None
        return node

    def test_first_call_is_not_a_repeat(self):
        node = self._make_bare_node()
        args = {"name": "imperialbrass", "style": "march"}
        assert node._repeated_music_call_args(_Result("", music_call_args=args)) is False
        assert node._last_music_call_args == args

    def test_identical_second_call_is_a_repeat(self):
        node = self._make_bare_node()
        args = {"name": "imperialbrass", "style": "march"}
        node._repeated_music_call_args(_Result("", music_call_args=dict(args)))
        assert node._repeated_music_call_args(_Result("", music_call_args=dict(args))) is True

    def test_different_second_call_is_not_a_repeat(self):
        node = self._make_bare_node()
        node._repeated_music_call_args(
            _Result("", music_call_args={"name": "imperialbrass", "style": "march"})
        )
        assert (
            node._repeated_music_call_args(
                _Result("", music_call_args={"name": "imperialbrass", "style": "waltz"})
            )
            is False
        )

    def test_no_call_this_turn_leaves_baseline_untouched(self):
        node = self._make_bare_node()
        node._last_music_call_args = {"name": "a"}
        assert node._repeated_music_call_args(_Result("", music_call_args=None)) is False
        assert node._last_music_call_args == {"name": "a"}

    def test_baseline_advances_after_a_genuine_change(self):
        node = self._make_bare_node()
        node._repeated_music_call_args(_Result("", music_call_args={"name": "a"}))
        node._repeated_music_call_args(_Result("", music_call_args={"name": "b"}))
        assert node._repeated_music_call_args(_Result("", music_call_args={"name": "b"})) is True

    def test_session_reset_clears_baseline(self):
        """``_reset_session_music_and_dj`` (issue #2835) тоже сбрасывает
        ``_last_music_call_args`` — прошлая сессия не должна «забраковывать»
        первый ``compose_music`` новой сессии."""
        node = self._make_bare_node()
        node._dj = MagicMock()
        node._publish_dj_off = MagicMock()
        node._publish_music_cleanup = MagicMock()
        node._music_guard = MagicMock()
        node._last_music_call_args = {"name": "imperialbrass", "style": "march"}

        node._reset_session_music_and_dj()

        assert node._last_music_call_args is None
