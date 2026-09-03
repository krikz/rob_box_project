"""
test_new_session_reset.py — Unit-тесты «новая сессия» / сброс диалога.

Проверяет, что dialogue_node при фразе «новая сессия» / «сбрось всё» /
Telegram-команде ``/clear`` детерминированно сбрасывает контекст текущего
диалога (историю, DSM, speaker-состояние, бэклог) и НЕ гоняет фразу в LLM.

Не требует ROS2 — rclpy замокан в conftest.py. DialogueNode создаётся через
object.__new__ + ручные атрибуты (как в test_command_intent_gate.py).
"""

from __future__ import annotations

import asyncio
import threading
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.dialogue_node import DialogueNode


def _make_node(new_session_enabled: bool = True) -> DialogueNode:
    """Минимальная DialogueNode без __init__ — атрибуты для _on_stt."""
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()

    n._wake_words = ["робок", "робот", "роббокс"]
    n._command_intent_gate_enabled = False
    n._new_session_enabled = new_session_enabled
    n._new_session_phrases = DialogueNode._DEFAULT_NEW_SESSION_PHRASES
    n._speaker_by_text = {}
    n._llm_skipped_counter = new_llm_skip_counter()
    n._maybe_log_skip_summary = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._publish_response = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._active_tg_chat_id = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._speaker_tracker = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n._session_started_at = None
    n._pending_backlog_flush = False
    # Issue #1563 — _tts_control_pub публикует STOP / IGNORE_STOP_MS.
    # По умолчанию MagicMock (тесты _on_stt его не трогают).
    n._tts_control_pub = MagicMock()
    # _loop не задаём — асинхронная очистка истории в тесте не дёргается.
    return n


def _stt(node: DialogueNode, text: str) -> None:
    msg = MagicMock()
    msg.data = text
    node._on_stt(msg)


class TestIsNewSessionCommand:
    """Чистая логика распознавания команды «новая сессия»."""

    @pytest.mark.parametrize(
        "clean",
        [
            "начни новую сессию",
            "новая сессия",
            "начать заново",
            "сбрось всё",
            "сбросить всё",
            "забудь всё что было",
            "очисти историю",
            "стереть историю",
            "новый диалог",
        ],
    )
    def test_voice_phrases_match(self, clean):
        n = _make_node()
        assert n._is_new_session_command(clean, clean.lower(), None) is True

    @pytest.mark.parametrize(
        "clean",
        [
            "расскажи анекдот",
            "как дела",
            "спой песню",
            "что нового",
        ],
    )
    def test_non_reset_phrases_do_not_match(self, clean):
        n = _make_node()
        assert n._is_new_session_command(clean, clean.lower(), None) is False

    def test_telegram_clear_matches(self):
        n = _make_node()
        assert n._is_new_session_command("/clear", "/clear", 42) is True

    def test_telegram_clear_requires_tg_source(self):
        """«/clear» без TG-маркера — это не команда сброса (обычная фраза)."""
        n = _make_node()
        assert n._is_new_session_command("/clear", "/clear", None) is False

    def test_disabled_flag_blocks_reset(self):
        n = _make_node(new_session_enabled=False)
        assert n._is_new_session_command("новая сессия", "новая сессия", None) is False
        assert n._is_new_session_command("/clear", "/clear", 42) is False


class TestOnSttNewSession:
    """Проверка _on_stt: фраза сброса → сброс, НЕ LLM."""

    @pytest.mark.parametrize(
        "phrase",
        [
            "робот начни новую сессию",
            "робот сбрось всё",
            "робот забудь всё",
            "робот очисти историю",
        ],
    )
    def test_voice_phrase_resets_without_llm(self, phrase):
        n = _make_node()
        _stt(n, phrase)
        n._dispatch_turn.assert_not_called()
        assert n._llm_skipped_counter["new_session"] == 1
        n._cancel_run.assert_called()
        n._dsm.reset.assert_called()
        n._speaker_tracker.reset.assert_called()
        n._maybe_record_session_end.assert_called_once()
        n._publish_response.assert_called_once()

    def test_telegram_clear_resets_without_llm(self):
        n = _make_node()
        _stt(n, "[TG:42] /clear")
        n._dispatch_turn.assert_not_called()
        assert n._llm_skipped_counter["new_session"] == 1
        n._publish_response.assert_called_once()

    def test_ordinary_phrase_still_dispatches(self):
        n = _make_node()
        _stt(n, "робот расскажи анекдот")
        n._dispatch_turn.assert_called_once()
        assert n._llm_skipped_counter["new_session"] == 0

    def test_disabled_reset_still_dispatches_to_llm(self):
        n = _make_node(new_session_enabled=False)
        _stt(n, "робот начни новую сессию")
        n._dispatch_turn.assert_called_once()
        assert n._llm_skipped_counter["new_session"] == 0


class TestClearSessionTurns:
    """Очистка in-memory окна ходов через DialogCore.clear_history."""

    def test_clear_session_turns_calls_core_clear_history(self):
        n = _make_node()
        n._core = MagicMock()

        asyncio.run(n._clear_session_turns())

        n._core.clear_history.assert_called_once()


class TestResetPublishesTtsImmune:
    """Issue #1563 — _reset_dialogue_session публикует IGNORE_STOP_MS в tts_control.

    Без этого barge-in STOP от wake-word в той же STT-фразе (например,
    «робот новая сессия…») успевает отменить синтез «Начинаю новую
    сессию…» ДО воспроизведения.
    """

    def _captured_publishes(self, n) -> list:
        """Список ``.data`` аргументов всех вызовов _tts_control_pub.publish."""
        return [
            call.args[0].data
            for call in n._tts_control_pub.publish.call_args_list
            if call.args and hasattr(call.args[0], "data")
        ]

    def test_reset_publishes_ignore_stop_ms_before_response(self):
        """После «робот новая сессия» dialogue_node публикует
        ``IGNORE_STOP_MS:700`` В ``/voice/tts/control`` ДО ``_publish_response``."""
        n = _make_node()
        _stt(n, "робот начни новую сессию")

        publishes = self._captured_publishes(n)
        # Должна быть хотя бы одна публикация с payload IGNORE_STOP_MS:700.
        assert any("IGNORE_STOP_MS" in p for p in publishes), (
            f"Expected IGNORE_STOP_MS publish, got: {publishes}"
        )
        assert any(p.startswith("IGNORE_STOP_MS:") and p.endswith("700") for p in publishes), (
            f"Expected 700 ms window, got: {publishes}"
        )
        # _publish_response должен быть вызван ровно один раз.
        n._publish_response.assert_called_once()
        # Порядок вызовов: IGNORE_STOP_MS публикуется ДО _publish_response.
        # Сравниваем по call_count: первый вызов должен быть IGNORE_STOP_MS,
        # потому что мы публикуем его в шаге 1a (ДО шага 7 _publish_response).
        first_publish_data = (
            n._tts_control_pub.publish.call_args_list[0]
            .args[0].data
        )
        assert "IGNORE_STOP_MS" in first_publish_data, (
            f"First tts_control publish must be IGNORE_STOP_MS, got: "
            f"{first_publish_data!r}"
        )

    def test_reset_publishes_for_telegram_clear(self):
        """Telegram «/clear» тоже должен открыть IMMUNE-окно TTS."""
        n = _make_node()
        _stt(n, "[TG:42] /clear")
        publishes = self._captured_publishes(n)
        assert any("IGNORE_STOP_MS" in p for p in publishes), (
            f"Expected IGNORE_STOP_MS publish for /clear, got: {publishes}"
        )

    def test_publish_failure_does_not_break_reset(self):
        """Если _tts_control_pub.publish падает — reset сессии всё равно
        должен отработать (best-effort, не роняем новый сеанс)."""
        n = _make_node()
        n._tts_control_pub.publish.side_effect = RuntimeError("publisher dead")
        # Не должно быть необработанного исключения.
        _stt(n, "робот сбрось всё")
        # _publish_response всё равно вызван.
        n._publish_response.assert_called_once()
        # _cancel_run отработал.
        n._cancel_run.assert_called()
        # DSM сброшен в IDLE.
        n._dsm.reset.assert_called()
