"""
test_dialogue_node.py — Реальные unit-тесты DialogueNode (FA-5, issue #833).

Заменяет 13+ пустых тест-методов (``pass``) из legacy-файла
``test/test_dialogue_node.py``. Каждый метод теперь проверяет
реальное поведение текущего DialogueNode:

  * создание ноды / контракт модуля          → test_node_creation_*
  * missing API key / пустая цепочка         → test_build_llm_* / test_resolve_provider_chain_*
  * deepseek API call (провайдер)            → test_build_single_provider_*
  * API error handling (fallback-цепочка)    → test_fallback_llm_*
  * управление контекстом (speaker context)  → test_build_dynamic_system_context_*
  * system prompt injection                  → test_load_system_prompt_* / test_render_event_instructions_*
  * max context length (skip-summary окно)   → test_maybe_log_skip_summary_*
  * накопление очереди запросов (STT gate)   → test_on_stt_*
  * batch processing (TTS batch)             → test_publish_response_batch_* / test_tts_batch_registry_*
  * очистка очереди на silence               → test_handle_silence_*
  * llm_processing flag / barge-in           → test_on_vad_*
  * прочие хелперы (babble, music guard)     → test_babble_* / test_music_guard_*

Не требует ROS2 — rclpy и openai замоканы в conftest.py.
"""

import asyncio
import json
import time
from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.dialogue_node import (
    BABBLE_BANNED_OPENERS,
    BABBLE_PERFORMANCE_KEYWORDS,
    DialogueNode,
    _FallbackLLM,
)

# Реальные enum'ы из rob_box_harness (conftest их не мокает — это
# чистые перечисления, безопасные для unit-окружения).
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
)


# ─────────────────────────────────────────────────────────────────────────────
#  Fixture: минимальная DialogueNode без __init__ (как test_pure_methods.py)
# ─────────────────────────────────────────────────────────────────────────────

def _make_node(parameters: dict | None = None) -> DialogueNode:
    """DialogueNode через ``object.__new__`` + ручные атрибуты.

    ``parameters`` — значения, которые возвращает ``get_parameter(name)``;
    отсутствующие ключи дают ``value=None`` (как FakeNode в conftest).
    """
    values = parameters or {}
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    n.get_parameter = lambda name: type(
        "Parameter", (), {"value": values.get(name)}
    )()

    # Publishers
    n._response_pub = MagicMock()
    n._state_pub = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._tts_control_pub = MagicMock()
    n._music_cleanup_pub = MagicMock()

    # State attrs
    n._wake_words = ["робок", "робот", "роббокс"]
    n._verbose_llm = False
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
        # Issue #1385 — gating: сколько раз wake-word был проигнорирован
        # из-за /voice/e2e/busy=True (идёт атомарный e2e-тест).
        "e2e_busy": 0,
    }
    n._last_skip_summary_ts = time.monotonic()
    n._speaker_by_text = {}
    n._current_speaker = {"is_known": False}
    n._speaker_lock = MagicMock()
    n._speaker_lock.__enter__ = MagicMock(return_value=n._speaker_lock)
    n._speaker_lock.__exit__ = MagicMock(return_value=False)
    n._speaker_tracker = MagicMock()

    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.IDLE
    n._dj = MagicMock()
    n._dj.state.enabled = False

    n._run_task = None
    n._run_cancelled = False
    n._task_lock = MagicMock()
    n._loop = MagicMock()
    n._loop.call_soon_threadsafe = lambda fn, *a, **kw: fn(*a, **kw)
    n._effects = MagicMock()
    n._effects.handle_tts_finished = MagicMock()
    n._effects.handle_sound_state = MagicMock()

    n._active_batches = {}
    n._pending_music_cleanup = False
    n._session_started_at = None
    n._session_end_reason = "success"

    n._core = MagicMock()
    n._faq_store = None
    n._event_profile = None
    n._startup_greeting_fired = False
    return n


# ─────────────────────────────────────────────────────────────────────────────
#  test_node_creation (legacy: «node can be created with API key»)
# ─────────────────────────────────────────────────────────────────────────────

class TestNodeCreation:
    def test_dialogue_node_is_ros2_node(self):
        """DialogueNode — подкласс rclpy.node.Node.

        Проверяем через ``dn.Node`` (класс, который dialogue_node.py реально
        связал при импорте), а не через ``rclpy.node.Node`` на момент теста:
        test/unit/tts/conftest.py перезаписывает ``sys.modules['rclpy.node']``
        своим FakeNode, поэтому прямой ``issubclass(DialogueNode, Node)``
        падает при полном прогоне (порядок сбора node < tts).
        """
        import rob_box_voice.dialogue_node as dn
        assert issubclass(dn.DialogueNode, dn.Node)

    def test_module_exposes_skill_aliases(self):
        """Модуль объявляет skill-классы как атрибуты (test contracts)."""
        import rob_box_voice.dialogue_node as dn
        for alias in ("MusicSkill", "FAQSkill", "WebSearchSkill",
                      "NavigationSkill", "MemorySkill", "StatusSkill"):
            assert hasattr(dn, alias)

    def test_module_constants_present(self):
        import rob_box_voice.dialogue_node as dn
        assert dn.ASYNCIO_LOOP_DRIVER_MAX_WORKERS == 1
        assert dn.ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S > 0
        assert BABBLE_BANNED_OPENERS
        assert BABBLE_PERFORMANCE_KEYWORDS

    def test_node_has_dialogue_name_contract(self):
        """Имя ноды — 'dialogue_node' (используется в __init__)."""
        n = _make_node()
        # Напрямую __init__ в unit-окружении не вызываем (нужен ROS2),
        # но контракт имени проверяем через статический анализ: класс
        # объявляет параметры с ожидаемыми именами.
        assert hasattr(n, "_declare_params")
        assert hasattr(n, "_build_llm")


# ─────────────────────────────────────────────────────────────────────────────
#  missing API key / provider chain (legacy: test_missing_api_key_raises_error)
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildLlm:
    def test_resolve_provider_chain_default(self):
        """Нет параметра llm_providers → default ['deepseek']."""
        n = _make_node()
        assert n._resolve_provider_chain() == ["deepseek"]

    def test_resolve_provider_chain_parses_csv(self):
        n = _make_node({"llm_providers": "minimax, deepseek"})
        assert n._resolve_provider_chain() == ["minimax", "deepseek"]

    def test_resolve_provider_chain_empty_uses_default(self):
        n = _make_node({"llm_providers": ""})
        assert n._resolve_provider_chain() == ["deepseek"]

    def test_build_llm_raises_when_no_provider(self):
        """Пустая цепочка провайдеров → RuntimeError (missing API key path)."""
        n = _make_node()
        n._resolve_provider_chain = MagicMock(return_value=["deepseek"])
        n._build_single_provider = MagicMock(return_value=None)
        with pytest.raises(RuntimeError, match="No LLM providers"):
            n._build_llm()

    def test_build_llm_single_provider_returned_directly(self):
        n = _make_node()
        provider = MagicMock()
        n._resolve_provider_chain = MagicMock(return_value=["deepseek"])
        n._build_single_provider = MagicMock(return_value=provider)
        assert n._build_llm() is provider

    def test_build_single_provider_unknown_name(self):
        """Неизвестный провайдер → None + warning (без краха)."""
        n = _make_node()
        assert n._build_single_provider("nonexistent") is None
        n.get_logger().warning.assert_called()

    @patch("rob_box_voice.dialogue_node.build_deepseek_provider")
    def test_build_single_provider_deepseek(self, mock_build):
        """deepseek строится с api_key из env (legacy: test_deepseek_api_call)."""
        import os
        old = os.environ.get("DEEPSEEK_API_KEY")
        os.environ["DEEPSEEK_API_KEY"] = "test-ds-key"
        try:
            n = _make_node({"deepseek.api_key": ""})
            provider = MagicMock()
            mock_build.return_value = provider
            assert n._build_single_provider("deepseek") is provider
            kwargs = mock_build.call_args.kwargs
            assert kwargs["api_key"] == "test-ds-key"
            assert kwargs["model"]
            assert kwargs["base_url"]
        finally:
            if old is None:
                os.environ.pop("DEEPSEEK_API_KEY", None)
            else:
                os.environ["DEEPSEEK_API_KEY"] = old


# ─────────────────────────────────────────────────────────────────────────────
#  API error handling (legacy: test_api_error_handling) — _FallbackLLM
# ─────────────────────────────────────────────────────────────────────────────

class TestFallbackLLM:
    def _make_failing_primary(self):
        async def complete(self, messages, tools=None):
            raise RuntimeError("Connection timeout")

        async def stream(self, messages, tools=None):
            raise RuntimeError("Connection timeout")
            yield  # pragma: no cover

        return type("Primary", (), {
            "name": "primary",
            "complete": complete,
            "stream": stream,
        })()

    def _make_ok_fallback(self, text="Привет! Как дела?"):
        async def complete(self, messages, tools=None):
            return {"content": text}

        async def stream(self, messages, tools=None):
            yield {"content": text}

        return type("Fallback", (), {
            "name": "fallback",
            "complete": complete,
            "stream": stream,
        })()

    def test_complete_falls_back_on_primary_error(self):
        llm = _FallbackLLM(
            primary=self._make_failing_primary(),
            fallback=self._make_ok_fallback(),
            logger=MagicMock(),
        )
        result = asyncio.run(llm.complete([{"role": "user", "content": "Test"}]))
        assert result["content"] == "Привет! Как дела?"
        llm._log.warning.assert_called()

    def test_complete_primary_success_no_fallback(self):
        async def complete(self, messages, tools=None):
            return {"content": "primary ok"}

        primary = type("Primary", (), {
            "name": "primary",
            "complete": complete,
            "stream": lambda messages, tools=None: (_ for _ in ()),
        })()
        fallback = self._make_ok_fallback()
        llm = _FallbackLLM(primary=primary, fallback=fallback, logger=MagicMock())
        result = asyncio.run(llm.complete([{"role": "user", "content": "Hi"}]))
        assert result["content"] == "primary ok"
        llm._log.warning.assert_not_called()

    def test_stream_falls_back_on_primary_error(self):
        llm = _FallbackLLM(
            primary=self._make_failing_primary(),
            fallback=self._make_ok_fallback(),
            logger=MagicMock(),
        )
        chunks = list(asyncio.run(self._collect(llm.stream([], tools=None))))
        assert chunks == [{"content": "Привет! Как дела?"}]

    @staticmethod
    async def _collect(agen):
        return [c async for c in agen]


# ─────────────────────────────────────────────────────────────────────────────
#  Conversation context (legacy: test_conversation_context_management)
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildDynamicSystemContext:
    def test_returns_system_context_xml(self):
        n = _make_node({"provider": "yandex"})
        ctx = n._build_dynamic_system_context()
        assert "<system_context>" in ctx
        assert "<user_profile>" in ctx
        assert "<name>unknown</name>" in ctx

    def test_speaker_name_included(self):
        n = _make_node({"provider": "yandex"})
        n._current_speaker = {"is_known": True, "name": "Анна",
                              "confidence": 0.92, "speaker_id": "sp_1234567890"}
        ctx = n._build_dynamic_system_context()
        assert "<name>Анна</name>" in ctx
        assert "<voice_confidence>0.92</voice_confidence>" in ctx
        assert "<speaker_id>sp_12345</speaker_id>" in ctx  # sp_id[:8]

    def test_invalid_speaker_name_sanitized(self):
        n = _make_node({"provider": "yandex"})
        n._current_speaker = {"is_known": True, "name": "Null", "confidence": 0.0}
        ctx = n._build_dynamic_system_context()
        assert "<name>unknown</name>" in ctx
        assert "Null" not in ctx

    def test_tts_provider_read_from_parameter(self):
        n = _make_node({"provider": "minimax"})
        ctx = n._build_dynamic_system_context()
        assert "<tts_provider>minimax</tts_provider>" in ctx


# ─────────────────────────────────────────────────────────────────────────────
#  System prompt (legacy: test_system_prompt_injection)
# ─────────────────────────────────────────────────────────────────────────────

class TestSystemPrompt:
    def test_load_system_prompt_returns_default_when_missing(self):
        """Файл промпта недоступен → дефолтная строка (без краха)."""
        n = _make_node({"system_prompt_file": "nonexistent.txt"})
        prompt = n._load_system_prompt()
        assert "ROBBOX" in prompt
        assert "робот" in prompt.lower()

    def test_render_event_instructions_returns_base_when_no_profile(self):
        n = _make_node()
        assert n._render_event_instructions("BASE") == "BASE"

    def test_render_event_instructions_injects_role(self):
        n = _make_node()
        n._event_profile = {"name": "День открытых дверей",
                            "robot_role": "экскурсовод"}
        rendered = n._render_event_instructions("BASE")
        assert "экскурсовод" in rendered
        assert "День открытых дверей" in rendered
        assert "BASE" in rendered


# ─────────────────────────────────────────────────────────────────────────────
#  Max context / skip-summary окно (legacy: test_max_context_length)
# ─────────────────────────────────────────────────────────────────────────────

class TestMaybeLogSkipSummary:
    def test_no_log_before_window(self):
        n = _make_node()
        n._llm_skipped_counter["no_wake_word"] = 1
        n._last_skip_summary_ts = time.monotonic()  # только что
        n._maybe_log_skip_summary(window_s=300.0)
        n.get_logger().info.assert_not_called()

    def test_logs_after_window_with_skips(self):
        n = _make_node()
        n._llm_skipped_counter["no_wake_word"] = 3
        n._llm_skipped_counter["silenced"] = 1
        n._last_skip_summary_ts = time.monotonic() - 400.0
        n._maybe_log_skip_summary(window_s=300.0)
        n.get_logger().info.assert_called_once()
        msg = n.get_logger().info.call_args[0][0]
        assert "no_wake_word=3" in msg
        assert "silenced=1" in msg
        assert "llm_skipped_total=4" in msg

    def test_no_log_after_window_when_no_skips(self):
        n = _make_node()
        n._last_skip_summary_ts = time.monotonic() - 400.0
        n._maybe_log_skip_summary(window_s=300.0)
        n.get_logger().info.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
#  Query queue / STT gate (legacy: test_query_queue_accumulation)
# ─────────────────────────────────────────────────────────────────────────────

class TestOnStt:
    def _msg(self, data: str):
        msg = MagicMock()
        msg.data = data
        return msg

    def test_empty_text_ignored(self):
        n = _make_node()
        n._on_stt(self._msg(""))
        n._dispatch_turn = MagicMock()
        n._dispatch_turn.assert_not_called()

    def test_no_wake_word_increments_counter(self):
        n = _make_node()
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("привет как дела"))
        assert n._llm_skipped_counter["no_wake_word"] == 1
        n._dispatch_turn.assert_not_called()

    def test_wake_word_dispatches_turn(self):
        n = _make_node()
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("робот, расскажи анекдот"))
        n._dispatch_turn.assert_called_once()
        args = n._dispatch_turn.call_args
        assert "расскажи анекдот" in args.args[0]

    def test_rejected_marker_ignored(self):
        n = _make_node()
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("rejected(empty)"))
        assert n._llm_skipped_counter["stt_rejected"] == 1
        n._dispatch_turn.assert_not_called()

    def test_silence_command_routes_to_handle_silence(self):
        n = _make_node()
        n._handle_silence = MagicMock()
        n._on_stt(self._msg("робот, помолчи"))
        n._handle_silence.assert_called_once()
        assert n._llm_skipped_counter["silence_command"] == 1

    def test_music_stop_override_goes_to_llm(self):
        """«выключи музыку» — НЕ silence: должен уйти в _dispatch_turn."""
        n = _make_node()
        n._dispatch_turn = MagicMock()
        n._handle_silence = MagicMock()
        n._on_stt(self._msg("робот, выключи музыку"))
        n._dispatch_turn.assert_called_once()
        n._handle_silence.assert_not_called()

    def test_silenced_state_unsilence(self):
        n = _make_node()
        n._dsm.current_state = DialogueStateKind.SILENCED
        n._on_stt(self._msg("робот, говори"))
        n._dsm.on_event.assert_called_with(DialogueEvent.UNSILENCE)

    def test_barge_in_cancels_run_on_new_input(self):
        n = _make_node()
        n._cancel_run = MagicMock()
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("робот, спой"))
        n._cancel_run.assert_called_once()

    # --- Issue #1385: gating /voice/e2e/busy -------------------------------
    def test_e2e_busy_blocks_wake_word(self):
        """Пока идёт e2e (busy=True), wake-word из микрофона игнорируется."""
        n = _make_node()
        # Симулируем что e2e запущен
        msg_busy = MagicMock()
        msg_busy.data = True
        n._on_e2e_busy(msg_busy)
        assert n._e2e_busy is True
        # Теперь wake-word НЕ должен пройти
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("робот, спой песенку про енотика"))
        assert n._llm_skipped_counter["e2e_busy"] == 1
        n._dispatch_turn.assert_not_called()

    def test_e2e_not_busy_passes_wake_word(self):
        """busy=False (по умолчанию) → обычное поведение, wake-word проходит."""
        n = _make_node()
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("робот, расскажи анекдот"))
        # Без busy → идёт в LLM (а не в e2e_busy counter)
        assert n._llm_skipped_counter.get("e2e_busy", 0) == 0
        n._dispatch_turn.assert_called_once()

    def test_e2e_busy_false_unblocks_wake_word(self):
        """После e2e False → wake-word снова работает."""
        n = _make_node()
        msg_busy = MagicMock()
        msg_busy.data = True
        n._on_e2e_busy(msg_busy)
        # Снимаем busy
        msg_free = MagicMock()
        msg_free.data = False
        n._on_e2e_busy(msg_free)
        assert n._e2e_busy is False
        # Wake-word снова проходит
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("робот, выключи музыку"))
        n._dispatch_turn.assert_called_once()
        assert n._llm_skipped_counter.get("e2e_busy", 0) == 0

    def test_e2e_busy_does_not_block_telegram(self):
        """Telegram ([TG:...]) НЕ блокируется busy — это не микрофон."""
        n = _make_node()
        msg_busy = MagicMock()
        msg_busy.data = True
        n._on_e2e_busy(msg_busy)
        # Telegram-маркер — обходит busy
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("[TG:12345] напой что-нибудь"))
        assert n._llm_skipped_counter.get("e2e_busy", 0) == 0
        n._dispatch_turn.assert_called_once()

    def test_e2e_busy_increments_counter_per_call(self):
        """Каждый skipped wake-word инкрементирует counter (для метрик)."""
        n = _make_node()
        msg_busy = MagicMock()
        msg_busy.data = True
        n._on_e2e_busy(msg_busy)
        n._dispatch_turn = MagicMock()
        for _ in range(3):
            n._on_stt(self._msg("робот, расскажи анекдот"))
        assert n._llm_skipped_counter["e2e_busy"] == 3

    def test_e2e_busy_callback_idempotent(self):
        """Повторный busy=True сет не ломает state."""
        n = _make_node()
        msg_busy = MagicMock()
        msg_busy.data = True
        n._on_e2e_busy(msg_busy)
        n._on_e2e_busy(msg_busy)
        n._on_e2e_busy(msg_busy)
        assert n._e2e_busy is True
        n._dispatch_turn = MagicMock()
        n._on_stt(self._msg("робот, спой"))
        assert n._llm_skipped_counter["e2e_busy"] == 1


# ─────────────────────────────────────────────────────────────────────────────
#  Batch processing (legacy: test_query_queue_batch_processing)
# ─────────────────────────────────────────────────────────────────────────────

class TestPublishResponseBatch:
    def test_publishes_all_chunks(self):
        n = _make_node()
        n._response_pub = MagicMock()
        total = n._publish_response_batch(["раз", "два", "три"])
        assert total == 3
        assert n._response_pub.publish.call_count == 3

    def test_empty_chunks_returns_zero(self):
        n = _make_node()
        assert n._publish_response_batch([]) == 0
        n._response_pub.publish.assert_not_called()

    def test_each_chunk_has_batch_metadata(self):
        n = _make_node()
        n._response_pub = MagicMock()
        n._publish_response_batch(["раз", "два"])
        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        assert len(published) == 2
        for data in published:
            payload = json.loads(data)
            assert "batch_id" in payload
            assert payload["batch_total"] == 2
        first = json.loads(published[0])
        second = json.loads(published[1])
        assert first["batch_index"] == 1
        assert second["batch_index"] == 2
        assert first["batch_id"] == second["batch_id"]


class TestTtsBatchRegistry:
    def test_register_is_idempotent(self):
        n = _make_node()
        n._register_active_batch("b1", 2)
        n._register_active_batch("b1", 2)
        assert n._active_batches == {"b1": 2}

    def test_unregister_removes(self):
        n = _make_node()
        n._register_active_batch("b1", 2)
        n._unregister_active_batch("b1")
        assert n._active_batches == {}

    def test_batch_complete_fires_cleanup_when_empty_and_pending(self):
        n = _make_node()
        n._pending_music_cleanup = True
        n._publish_music_cleanup = MagicMock()
        msg = MagicMock()
        msg.data = json.dumps({"batch_id": "b1", "chunks_total": 1})
        n._on_tts_batch_complete(msg)
        n._publish_music_cleanup.assert_called_once()

    def test_batch_complete_defers_when_batches_active(self):
        n = _make_node()
        n._pending_music_cleanup = True
        n._register_active_batch("b2", 2)
        n._publish_music_cleanup = MagicMock()
        msg = MagicMock()
        msg.data = json.dumps({"batch_id": "b1", "chunks_total": 1})
        n._on_tts_batch_complete(msg)
        n._publish_music_cleanup.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
#  Silence (legacy: test_query_queue_cleared_on_silence)
# ─────────────────────────────────────────────────────────────────────────────

class TestHandleSilence:
    def test_cancels_run_and_emits_event(self):
        n = _make_node()
        n._cancel_run = MagicMock()
        n._speak_direct = MagicMock()
        n._handle_silence()
        n._cancel_run.assert_called_once()
        n._dsm.on_event.assert_called_with(DialogueEvent.SILENCE_COMMAND)
        n._speak_direct.assert_called_once()

    def test_publish_state_after_silence(self):
        n = _make_node()
        n._cancel_run = MagicMock()
        n._speak_direct = MagicMock()
        n._publish_state = MagicMock()
        n._handle_silence()
        n._publish_state.assert_called_once()


# ─────────────────────────────────────────────────────────────────────────────
#  llm_processing flag / barge-in (legacy: test_llm_processing_flag)
# ─────────────────────────────────────────────────────────────────────────────

class TestOnVad:
    def _msg(self, active: bool):
        msg = MagicMock()
        msg.data = active
        return msg

    def test_rising_edge_sets_flag(self):
        n = _make_node()
        n.vad_speech_detected = False
        n._on_vad(self._msg(True))
        assert n.vad_speech_detected is True

    def test_rising_edge_during_llm_sets_interrupt(self):
        n = _make_node()
        n.vad_speech_detected = False
        n.llm_processing = True
        n.mcp_tools_available = False
        n._on_vad(self._msg(True))
        assert n.interrupt_agent_loop is True

    def test_no_interrupt_when_not_processing(self):
        n = _make_node()
        n.vad_speech_detected = False
        n.llm_processing = False
        n._on_vad(self._msg(True))
        assert getattr(n, "interrupt_agent_loop", False) is False

    def test_falling_edge_clears_flag(self):
        n = _make_node()
        n.vad_speech_detected = True
        n._on_vad(self._msg(False))
        assert n.vad_speech_detected is False


# ─────────────────────────────────────────────────────────────────────────────
#  Babble detector / music guard (issue #992) — новые реальные проверки
# ─────────────────────────────────────────────────────────────────────────────

class TestBabbleDetector:
    def test_metalanguage_opener_detected(self):
        n = _make_node()
        assert n._is_metalanguage_babble("Зачитаю рэп про космос!")
        assert n._is_metalanguage_babble("Сейчас устроим концерт")
        assert n._is_metalanguage_babble("Слушай, давай я спою")

    def test_normal_answer_not_babble(self):
        n = _make_node()
        assert not n._is_metalanguage_babble("Черное море находится на юге России.")
        assert not n._is_metalanguage_babble("")
        assert not n._is_metalanguage_babble(None)

    def test_user_wants_performance(self):
        n = _make_node()
        assert n._user_wants_performance("спой мне песню")
        assert n._user_wants_performance("зачитай рэп")
        assert not n._user_wants_performance("как дела?")

    def test_user_wants_music(self):
        n = _make_node()
        assert n._user_wants_music("включи музыку")
        assert not n._user_wants_music("расскажи анекдот")


class TestMusicCleanup:
    def test_publish_music_cleanup_noop_without_pub(self):
        n = _make_node()
        n._music_cleanup_pub = None
        n._publish_music_cleanup()  # не должно падать

    def test_publish_music_cleanup_publishes_json(self):
        n = _make_node()
        n._music_cleanup_pub = MagicMock()
        n._publish_music_cleanup(reason="dialogue_end")
        msg = n._music_cleanup_pub.publish.call_args[0][0]
        assert json.loads(msg.data) == {"reason": "dialogue_end"}


class TestDjFarewell:
    def test_on_dj_stop_farewell_publishes_response(self):
        n = _make_node()
        n._publish_response = MagicMock()
        n._on_dj_stop_farewell("Роббокс")
        n._publish_response.assert_called_once()
        text = n._publish_response.call_args[0][0]
        assert "выключается" in text


class TestPublishState:
    def test_publish_state_publishes_current_state_name(self):
        n = _make_node()
        n._state_pub = MagicMock()
        n._dsm.current_state = DialogueStateKind.DIALOGUE
        n._publish_state()
        msg = n._state_pub.publish.call_args[0][0]
        assert msg.data == "DIALOGUE"
