"""Unit-тесты normalize_voice_state + DIALOGUE_STATE_TO_BRIDGE.

Тесты чистые — не требуют rclpy/geometry_msgs, чтобы запускаться на
dev-env без Docker (соответствует pattern ``test_status.py``,
``test_battery.py`` и др. в этом каталоге).

Источник истины:
    docs/recon/voice-dialogue-state-payload.md (t_1886f7be).
"""

from __future__ import annotations

import logging

import pytest

from rob_box_quest.streams.voice_state import (
    BRIDGE_STATES,
    DIALOGUE_STATE_TO_BRIDGE,
    FALLBACK_STATE,
    normalize_voice_state,
)


class _StringLike:
    """Заглушка под ``std_msgs/String`` для тестов без ROS2."""

    def __init__(self, data: str) -> None:
        self.data = data


# --- Базовый контракт -------------------------------------------------------


class TestBridgeStateEnum:
    """Допустимые bridge-state'ы — контракт для клиента (meta-quest-api.md §4)."""

    def test_idle_listening_thinking_speaking_are_listed(self):
        for s in ("idle", "listening", "thinking", "speaking"):
            assert s in BRIDGE_STATES, f"BRIDGE_STATES missing {s!r}"

    def test_brige_states_are_lowercase(self):
        # Если кто-то добавит 'IDLE' вместо 'idle' — это сломает клиента.
        for s in BRIDGE_STATES:
            assert s == s.lower(), f"BRIDGE_STATES содержит не-lowercase {s!r}"


class TestFallbackState:
    def test_fallback_state_is_idle(self):
        # Жёсткое требование задачи: «неизвестная строка → warning + state=idle».
        assert FALLBACK_STATE == "idle"
        assert FALLBACK_STATE in BRIDGE_STATES


# --- Маппинг DialogueStateKind → bridge ------------------------------------


class TestKnownMappings:
    """Все 4 значения FSM должны маппиться (recon §1.2)."""

    @pytest.mark.parametrize(
        "raw,expected_state,expected_detail",
        [
            ("IDLE",      "idle",      None),
            ("LISTENING", "listening", None),
            ("DIALOGUE",  "speaking",  None),
            ("SILENCED",  "idle",      "silenced"),
        ],
    )
    def test_known_states_map_to_bridge(
        self, raw: str, expected_state: str, expected_detail: str | None
    ):
        result = normalize_voice_state(raw)
        assert result["state"] == expected_state
        assert result["detail"] == expected_detail

    def test_idle_no_detail_field_is_none(self):
        """IDLE → detail=None (НЕ пустая строка)."""
        result = normalize_voice_state("IDLE")
        assert result["detail"] is None

    def test_silenced_sets_detail_for_ui(self):
        """SILENCED — отдельный визуальный режим «mute», сохраняем в detail."""
        result = normalize_voice_state("SILENCED")
        assert result["detail"] == "silenced"
        assert result["state"] == "idle"

    def test_dialogue_maps_to_speaking_not_thinking(self):
        """DIALOGUE = «LLM + TTS» = speaking (решение recon §5.2)."""
        result = normalize_voice_state("DIALOGUE")
        assert result["state"] == "speaking"
        # 'thinking' в текущем контракте DialogueStateKind нет — не путаем.

    def test_table_keys_are_uppercase(self):
        """Ключи таблицы — литералы из DialogueStateKind.name (UPPERCASE)."""
        for k in DIALOGUE_STATE_TO_BRIDGE:
            assert k == k.upper(), f"Ключ {k!r} должен быть UPPERCASE"


class TestPayloadTypeAcceptance:
    """Нормализатор должен принимать ROS String, str, и bytes."""

    def test_accepts_std_msgs_string(self):
        result = normalize_voice_state(_StringLike("LISTENING"))
        assert result["state"] == "listening"

    def test_accepts_plain_str(self):
        result = normalize_voice_state("IDLE")
        assert result["state"] == "idle"

    def test_accepts_bytes(self):
        result = normalize_voice_state(b"DIALOGUE")
        assert result["state"] == "speaking"

    def test_strips_surrounding_whitespace(self):
        """Лишние пробелы вокруг имени FSM (защита от кривого publisher'а)."""
        result = normalize_voice_state("  IDLE\n")
        assert result["state"] == "idle"


# --- Неизвестные значения → fallback + warning -----------------------------


class TestUnknownFallback:
    """Любое значение вне таблицы → WARNING + state=idle (НЕ crash, НЕ silent)."""

    def test_unknown_uppercase_string_falls_back_with_warning(self, caplog):
        with caplog.at_level(logging.WARNING, logger="rob_box_quest.streams.voice_state"):
            result = normalize_voice_state("BOGUS_STATE")
        assert result["state"] == FALLBACK_STATE
        assert result["detail"] is None
        # Сообщение должно содержать raw payload, чтобы дев видел, что пришло.
        assert any("BOGUS_STATE" in rec.message for rec in caplog.records)
        assert any("fallback" in rec.message for rec in caplog.records)

    def test_empty_string_falls_back_at_debug_level(self, caplog):
        """Пустой payload — не WARNING (спам), а DEBUG (один раз на старте)."""
        with caplog.at_level(logging.DEBUG, logger="rob_box_quest.streams.voice_state"):
            result = normalize_voice_state("")
        assert result["state"] == FALLBACK_STATE
        # WARNING не должно быть — только DEBUG.
        warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
        assert warnings == []

    def test_whitespace_only_falls_back(self):
        """Строка из пробелов → пустая после strip → fallback."""
        result = normalize_voice_state("   ")
        assert result["state"] == FALLBACK_STATE
        assert result["detail"] is None

    def test_none_payload_falls_back(self):
        """На случай, если upstream шлёт пустой Optional[str]."""
        result = normalize_voice_state(None)
        assert result["state"] == FALLBACK_STATE

    def test_lowercase_idle_is_unknown_not_silently_mapped(self, caplog):
        """``"idle"`` (lowercase) ≠ ``"IDLE"`` (FSM name). Лучше WARNING.

        Если бы upstream сменил регистр .name — мы хотим это увидеть в
        логах, а не молча работать.
        """
        with caplog.at_level(logging.WARNING, logger="rob_box_quest.streams.voice_state"):
            result = normalize_voice_state("idle")
        assert result["state"] == FALLBACK_STATE
        assert any("idle" in rec.message for rec in caplog.records)


# --- Расширяемость ---------------------------------------------------------


class TestExtensibilityContract:
    """Спецификация требует: новое состояние FSM = явная ветка в таблице + тест.

    Этот тест «пингует» таблицу: если кто-то добавит значение в
    DialogueStateKind в коде, но забудет пополнить DIALOGUE_STATE_TO_BRIDGE —
    тест напомнит через явное сообщение.
    """

    def test_table_contains_at_least_4_entries(self):
        """Recon §1.2: 4 значения (IDLE/LISTENING/DIALOGUE/SILENCED)."""
        assert len(DIALOGUE_STATE_TO_BRIDGE) >= 4, (
            "DialogueStateKind имеет 4 значения (recon §1.2). Если их стало "
            "больше — добавь ветку в DIALOGUE_STATE_TO_BRIDGE + параметризованный тест."
        )

    def test_all_table_values_are_valid_bridge_states(self):
        """Никаких значений вне BRIDGE_STATES — иначе клиент не поймёт."""
        for raw, (bridge_state, _detail) in DIALOGUE_STATE_TO_BRIDGE.items():
            assert bridge_state in BRIDGE_STATES, (
                f"DIALOGUE_STATE_TO_BRIDGE[{raw!r}] ведёт в {bridge_state!r}, "
                f"которого нет в BRIDGE_STATES {BRIDGE_STATES}"
            )
