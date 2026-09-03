"""test_quest_command_mode.py — Unit-тесты для AV-22 (Issue #1914).

Проверяет ``dialogue_node._on_quest_stt`` при ``voice_input_mode="quest_command"``:

1. Фраза публикуется в ``/avatar/command`` через ``_avatar_command_pub``
   с правильным JSON-payload (заморожен в worker-brief §3.3 +
   ``rob_box_core.avatar_command``).
2. ``_dispatch_turn`` (LLM личности) **не** вызывается — гейт
   «личность молчит» (worker-brief §6.2, ADR-0018).
3. ``_speak_direct`` тоже **не** вызывается — это не озвучка и не диалог.
4. ``client_id`` имеет форму ``quest:<session_id>``, где ``session_id``
   берётся из ``self._quest_session_id`` (default ``"unknown"`` до
   того, как quest-сервер начнёт выставлять через /avatar/set_voice_mode).
5. Параметр ``voice_input_mode`` задекларирован в ``_declare_params()``
   и в YAML (worker-brief §6.8 — док и код не разъезжаются).
6. ``ADR-0027 §3.4`` упоминает ``quest_command`` в списке режимов
   (ADR-0043/0044 — список режимов в доке = список в коде).
7. При пустой фразе — нет публикации и нет LLM (защита от мусора).

Не требует ROS2 — rclpy/rcl_interfaces замоканы в ``conftest.py``.
DialogueNode создаётся через ``object.__new__`` + ручные атрибуты
(как в ``test_quest_stt_source.py``).
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode


# ─── Пути к источникам истины ─────────────────────────────────────────────


REPO_ROOT = Path(__file__).resolve().parents[5]
DIALOGUE_NODE_PY = (
    REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
)
DIALOGUE_NODE_YAML = (
    REPO_ROOT / "src" / "rob_box_voice" / "config" / "dialogue_node.yaml"
)
ADR_0027 = (
    REPO_ROOT / "docs" / "adr" / "0027-meta-quest-ar-control.md"
)


# ─── Helpers / fixtures ──────────────────────────────────────────────────


class _Param:
    def __init__(self, value):
        self.value = value


@pytest.fixture
def node():
    """DialogueNode через ``object.__new__`` + ручные атрибуты.

    Намеренно НЕ вызываем ``_declare_params`` — оно требует живой rclpy.
    Режим переключаем через ``node._voice_mode`` (наш cache-атрибут),
    который читает ``_on_quest_stt``.
    """
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    # Базовый набор атрибутов, как в test_quest_stt_source.py.
    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._speak_direct = MagicMock()
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

    # AV-22: фиксируем session_id, чтобы проверить client_id.
    n._quest_session_id = "test-sess-123"

    # AV-22: publisher /avatar/command (мокаем).
    n._avatar_command_pub = MagicMock()

    # get_parameter("voice_input_mode") → режим, которым управляет супервизор.
    n._voice_mode = "respeaker"
    n.get_parameter = lambda name: _Param(n._voice_mode)
    return n


def _make_stt_msg(data: str):
    msg = MagicMock()
    msg.data = data
    return msg


def _captured_payload(node):
    """Достаём JSON-payload, опубликованный в ``/avatar/command``."""
    assert node._avatar_command_pub.publish.called, (
        "publisher не вызван — _on_quest_stt не публикует в /avatar/command"
    )
    call = node._avatar_command_pub.publish.call_args
    msg = call.args[0]
    return json.loads(msg.data)


# ─── Сами тесты ──────────────────────────────────────────────────────────


class TestQuestCommandModePublishesToAvatarCommand:
    """AC: ``voice_input_mode='quest_command'`` публикует в ``/avatar/command``."""

    def test_quest_command_publishes_json_payload(self, node):
        node._voice_mode = "quest_command"
        node._on_quest_stt(_make_stt_msg("мотивируй народ"))
        payload = _captured_payload(node)
        # Замороженный контракт (worker-brief §3.3):
        assert set(payload.keys()) >= {
            "request_id", "source", "client_id", "text", "ts_ms",
        }
        assert payload["source"] == "quest"
        assert payload["text"] == "мотивируй народ"
        assert payload["client_id"] == "quest:test-sess-123"
        assert isinstance(payload["request_id"], str)
        assert len(payload["request_id"]) == 36  # UUIDv4
        assert isinstance(payload["ts_ms"], int)
        assert payload["ts_ms"] > 0

    def test_quest_command_default_session_id_is_unknown(self):
        """Без явного ``_quest_session_id`` — ``client_id='quest:unknown'``.

        Quest-сервер выставит реальный session_id через
        ``/avatar/set_voice_mode`` в одной из follow-up карточек
        (worker-brief §1.3: client_id формирует СЕРВЕРНАЯ сторона).
        """
        n = object.__new__(DialogueNode)
        logger = MagicMock()
        n.get_logger = lambda: logger
        n._wake_words = []
        n._dsm = MagicMock()
        n._dsm.current_state = MagicMock()
        n._dj = MagicMock()
        n._dj.state.enabled = False
        n._cancel_run = MagicMock()
        n._sound_trigger_pub = MagicMock()
        n._publish_state = MagicMock()
        n._dispatch_turn = MagicMock()
        n._speak_direct = MagicMock()
        n._verbose_llm = False
        n._speaker_by_text = {}
        n._llm_skipped_counter = {
            "no_wake_word": 0, "silenced": 0, "silence_command": 0,
            "empty_after_strip": 0, "stt_rejected": 0, "music_stop": 0,
        }
        n._maybe_log_skip_summary = MagicMock()
        n._active_tg_chat_id = None
        n._quest_session_id = None  # ← вот этот кейс
        n._avatar_command_pub = MagicMock()
        n._voice_mode = "quest_command"
        n.get_parameter = lambda name: _Param(n._voice_mode)

        n._on_quest_stt(_make_stt_msg("поехали"))
        payload = _captured_payload(n)
        assert payload["client_id"] == "quest:unknown"

    def test_quest_command_routes_only_to_publisher(self, node):
        """AC: ``voice_input_mode='quest_command'`` — НЕ LLM, НЕ TTS."""
        node._voice_mode = "quest_command"
        node._on_quest_stt(_make_stt_msg("привет"))
        node._dispatch_turn.assert_not_called()  # LLM личности не зовём
        node._speak_direct.assert_not_called()    # и TTS не озвучиваем
        # Publisher вызван ровно один раз.
        assert node._avatar_command_pub.publish.call_count == 1

    def test_quest_command_empty_text_no_publish(self, node):
        """Пустая/пробельная фраза — нет публикации (защита от мусора)."""
        node._voice_mode = "quest_command"
        node._on_quest_stt(_make_stt_msg("   \n\t  "))
        node._avatar_command_pub.publish.assert_not_called()
        node._dispatch_turn.assert_not_called()
        node._speak_direct.assert_not_called()


class TestOtherModesUnaffected:
    """AC: новый режим не сломал старые ветки ``_on_quest_stt``."""

    def test_quest_ttts_still_repeats_via_tts(self, node):
        node._voice_mode = "quest_ttts"
        node._on_quest_stt(_make_stt_msg("привет"))
        node._speak_direct.assert_called_once_with("привет")
        node._dispatch_turn.assert_not_called()
        # AV-22: publisher НЕ должен зваться в quest_ttts.
        node._avatar_command_pub.publish.assert_not_called()

    def test_quest_stt_still_dispatches_llm(self, node):
        node._voice_mode = "quest_stt"
        node._on_quest_stt(_make_stt_msg("расскажи что видишь"))
        node._dispatch_turn.assert_called_once()
        # AV-22: publisher НЕ должен зваться в quest_stt.
        node._avatar_command_pub.publish.assert_not_called()

    def test_respeaker_still_ignores_quest_stt(self, node):
        node._voice_mode = "respeaker"
        node._on_quest_stt(_make_stt_msg("привет"))
        node._speak_direct.assert_not_called()
        node._dispatch_turn.assert_not_called()
        node._avatar_command_pub.publish.assert_not_called()

    def test_quest_passthrough_still_ignored_here(self, node):
        """quest_passthrough не идёт через STT (звук играет sound_node)."""
        node._voice_mode = "quest_passthrough"
        node._on_quest_stt(_make_stt_msg("привет"))
        node._speak_direct.assert_not_called()
        node._dispatch_turn.assert_not_called()
        node._avatar_command_pub.publish.assert_not_called()


# ─── Source-of-truth: код ↔ YAML ↔ ADR ────────────────────────────────────


class TestSourcesOfTruthConsistency:
    """AC: список режимов в YAML = список в коде = список в ADR (ADR-0043/0044)."""

    def test_quest_command_in_yaml(self):
        """``src/rob_box_voice/config/dialogue_node.yaml`` упоминает ``quest_command``."""
        text = DIALOGUE_NODE_YAML.read_text(encoding="utf-8")
        assert "quest_command" in text, (
            "voice_input_mode=quest_command отсутствует в dialogue_node.yaml — "
            "список режимов в доке ≠ список в коде (ADR-0043)"
        )

    def test_quest_command_in_dialogue_node_code(self):
        """dialogue_node.py содержит ветку ``quest_command`` в ``_on_quest_stt``."""
        text = DIALOGUE_NODE_PY.read_text(encoding="utf-8")
        assert 'mode == "quest_command"' in text, (
            "_on_quest_stt не обрабатывает voice_input_mode=quest_command"
        )

    def test_quest_command_in_adr_0027_section_3_4(self):
        """ADR-0027 §3.4 перечисляет ``quest_command`` в таблице режимов.

        Без этого — классическая «разъехавшаяся документация» (ADR-0043).
        """
        text = ADR_0027.read_text(encoding="utf-8")
        # §3.4 начинается с заголовка "### 3.4. Voice modes" и заканчивается
        # перед следующим "###". Берём его и ищем упоминание режима.
        m = re.search(
            r"### 3\.4\. Voice modes.*?(?=^### )",
            text,
            re.DOTALL | re.MULTILINE,
        )
        assert m is not None, "ADR-0027 §3.4 не найден"
        section = m.group(0)
        assert "quest_command" in section, (
            "ADR-0027 §3.4 не упоминает quest_command — список в доке ≠ код "
            "(ADR-0043, worker-brief §6.8)"
        )

    def test_publisher_created_for_avatar_command_topic(self):
        """В ``__init__`` DialogueNode создаётся publisher ``/avatar/command``.

        Проверяем через regex по тексту, чтобы не падать на rclpy.
        """
        text = DIALOGUE_NODE_PY.read_text(encoding="utf-8")
        assert '"/avatar/command"' in text or "AVATAR_COMMAND_TOPIC" in text, (
            "publisher для /avatar/command не создан в dialogue_node.__init__"
        )


# ─── Гейт «личность молчит» (acceptance, явно запрошенный в Issue #1914) ─


class TestPersonalityGateWhenCommandMode:
    """AC: «личность не отвечает параллельно ни для одного из источников»."""

    def test_quest_command_skips_personality_llm(self, node):
        """В режиме команды ``_dispatch_turn`` (LLM личности) НЕ вызывается,
        даже если STT вернул непустую фразу.
        """
        node._voice_mode = "quest_command"
        node._on_quest_stt(_make_stt_msg("привет как дела"))
        node._dispatch_turn.assert_not_called()

    def test_quest_command_skips_speak_direct(self, node):
        """В режиме команды ``_speak_direct`` НЕ вызывается."""
        node._voice_mode = "quest_command"
        node._on_quest_stt(_make_stt_msg("привет как дела"))
        node._speak_direct.assert_not_called()
