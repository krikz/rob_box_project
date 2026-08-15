"""test_issue_1195_tg_source.py — Unit-тесты source-маркера [TG:chat_id] (issue #1195).

Проверяет в dialogue_node:
  1. Текст из Telegram-чата ([TG:chat_id] ...) БЕЗ wake word проходит
     wake-gate (обращение в чате очевидно) и chat_id запоминается для
     маршрутизации ответа.
  2. Обычный текст без wake word по-прежнему игнорируется (no_wake_word).
  3. Битый [TG:...] маркер не отключает wake-gate.
  4. _run_turn(from_tg=True) НЕ вешает голосовую биометрию [Spkr:...] и
     помечает источник префиксом [TG] для LLM.
  5. build_ssml_payload / _publish_response кладут tg_chat_id в payload
     /voice/dialogue/response для маршрутизации эхо-ответа.
"""

import asyncio
import json
import threading
from unittest.mock import AsyncMock, MagicMock

import pytest

from rob_box_voice.core.speak_helpers import build_ssml_payload
from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  Fixture: минимальная DialogueNode через __new__ (без __init__)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None
    return n


def _make_stt_msg(data: str):
    msg = MagicMock()
    msg.data = data
    return msg


# ─────────────────────────────────────────────────────────────────────────────
#  _on_stt: wake-gate + source marker
# ─────────────────────────────────────────────────────────────────────────────

class TestTgSourceMarkerGate:
    def test_tg_text_without_wake_word_bypasses_gate(self, node):
        """[TG:42] продолжай — без wake word, но из чата: должен дойти до
        _dispatch_turn (gate пропущен), chat_id запомнен, маркер снят."""
        node._on_stt(_make_stt_msg("[TG:42] продолжай"))

        assert node._active_tg_chat_id == 42
        node._dispatch_turn.assert_called_once()
        kwargs = node._dispatch_turn.call_args.kwargs
        # маркер снят, текст без wake word дошёл
        assert node._dispatch_turn.call_args.args[0] == "продолжай"
        assert kwargs.get("from_tg") is True

    def test_tg_text_with_wake_word_bypasses_gate(self, node):
        """[TG:42] робот привет — штатный путь с wake word тоже работает
        (wake word снимается strip_wake_word, маркер — до него)."""
        node._on_stt(_make_stt_msg("[TG:42] робот привет"))

        assert node._active_tg_chat_id == 42
        node._dispatch_turn.assert_called_once()
        # wake word «робот» снят, «привет» ушло в диспатч
        assert node._dispatch_turn.call_args.args[0] == "привет"

    def test_plain_text_without_wake_word_still_ignored(self, node):
        """Обычный (голосовой) текст без wake word НЕ должен проходить
        wake-gate — защита от фоновой речи."""
        node._on_stt(_make_stt_msg("продолжай"))

        node._dispatch_turn.assert_not_called()
        assert node._llm_skipped_counter["no_wake_word"] == 1
        assert node._active_tg_chat_id is None

    def test_malformed_tg_marker_does_not_bypass_gate(self, node):
        """Битый маркер ([TG:abc], без ']') не должен отключать wake-gate."""
        node._on_stt(_make_stt_msg("[TG:abc] продолжай"))
        node._dispatch_turn.assert_not_called()
        assert node._llm_skipped_counter["no_wake_word"] == 1
        assert node._active_tg_chat_id is None

    def test_tg_marker_updates_active_chat_for_echo(self, node):
        """Последовательные сообщения обновляют _active_tg_chat_id."""
        node._on_stt(_make_stt_msg("[TG:111] привет"))
        assert node._active_tg_chat_id == 111
        node._on_stt(_make_stt_msg("[TG:222] как дела"))
        assert node._active_tg_chat_id == 222


# ─────────────────────────────────────────────────────────────────────────────
#  _run_turn(from_tg=True): без голосовой биометрии, с префиксом [TG]
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.asyncio
async def test_run_turn_from_tg_skips_speaker_identity_and_prefixes():
    """TG-текст не получает [Spkr:...] от _current_speaker, а помечается
    префиксом [TG] для LLM (источник виден модели)."""
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_cancelled = False
    n._babble_retry_used = False
    n._music_guard_retry_count = 0
    n._pending_music_cleanup = False
    n._speaker_id_enabled = True  # даже при включённой биометрии — TG без неё

    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(return_value="<system_context/>")
    n._llm = MagicMock()
    n._speak_direct = MagicMock()
    n._active_batches = set()
    n._dsm = MagicMock()
    n._dsm.current_state = "idle"  # не DIALOGUE — финальный DIALOGUE_END не нужен
    n._publish_state = MagicMock()
    n._apply_music_guard = MagicMock()
    n._publish_music_cleanup = MagicMock()
    n._maybe_record_session_end = MagicMock()

    logger = MagicMock()
    n.get_logger = lambda: logger

    class _Result:
        spoken_text = "привет"
        tools_called = ()
        error = None

    n._core = MagicMock()
    n._core.process_input = AsyncMock(return_value=_Result())
    n._handle_result = MagicMock()

    await n._run_turn("продолжай", from_tg=True)

    # Биометрия НЕ вызывалась
    n._apply_speaker_identity.assert_not_called()
    # process_input получил user_input с префиксом [TG]
    called_user_input = n._core.process_input.call_args.args[0]
    assert called_user_input == "[TG] продолжай"


# ─────────────────────────────────────────────────────────────────────────────
#  build_ssml_payload / _publish_response: tg_chat_id в payload
# ─────────────────────────────────────────────────────────────────────────────

class TestTgChatIdInPayload:
    def test_build_ssml_payload_includes_tg_chat_id(self):
        payload = json.loads(build_ssml_payload("привет", tg_chat_id=42))
        assert payload["tg_chat_id"] == 42
        assert "<speak>привет</speak>" in payload["ssml"]

    def test_build_ssml_payload_omits_tg_chat_id_by_default(self):
        payload = json.loads(build_ssml_payload("привет"))
        assert "tg_chat_id" not in payload

    def test_publish_response_carries_active_tg_chat_id(self):
        n = object.__new__(DialogueNode)
        n._active_tg_chat_id = 42
        n._response_pub = MagicMock()

        n._publish_response("привет")

        published = n._response_pub.publish.call_args.args[0]
        payload = json.loads(published.data)
        assert payload["tg_chat_id"] == 42

    def test_publish_response_without_tg_chat_omits_field(self):
        n = object.__new__(DialogueNode)
        n._active_tg_chat_id = None
        n._response_pub = MagicMock()

        n._publish_response("привет")

        published = n._response_pub.publish.call_args.args[0]
        payload = json.loads(published.data)
        assert "tg_chat_id" not in payload
