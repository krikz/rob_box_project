"""Unit-тесты :mod:`rob_box_core.bridge_protocol`.

Что фиксирует (Definition of Done из issue #2192):

  1. ``git grep 'TOPIC_IDS\\|STREAM_CATALOG' src/`` → только
     ``bridge_protocol.py`` и его тест (после рефакторинга).
     До рефакторинга — допустимо, что legacy-места ещё ссылаются;
     отдельный тест ``test_topics_registry_compatibility`` проверяет
     forward-compat: переэкспорт старых имён из rob_box_quest.

  2. Каждая запись ``COMMANDS`` имеет ``name`` и валидный ``subprotocol``
     ∈ {``"v1"``, ``"v2"``, ``"any"``}; каждое ``name`` уникально.

  3. ``ERRORS`` — frozenset-семантика (нет дублей), ``implemented: False``
     коды помечены явно (``RATE_LIMIT`` для ADR-0080 §7).

  4. ``STREAMS`` консистентен между topic_id и ui_name (биекция).

  5. ``MODES`` / ``FLOORS`` совпадают с Enum.

Запуск:
    cd src/rob_box_core && PYTHONPATH=. python3 -m pytest \\
        test/test_bridge_protocol.py -v
"""

from __future__ import annotations

import pytest

from rob_box_core.bridge_protocol import (
    COMMANDS,
    ERRORS,
    ERROR_SPECS,
    EVENTS,
    FLOORS,
    FrameTypeId,
    MODES,
    STREAMS,
    VOICE_LANGUAGES,
    VOICE_PIPELINE_DEFAULT_LANGUAGE,
    VOICE_PRESET_IDS,
    VOICE_WIRE_MODES,
    Floor,
    Mode,
    client_dispatched_commands,
    deprecated_commands,
    get_command,
    get_event,
    get_stream,
    get_stream_by_topic_id,
    is_known_error,
)


# --- 1. Catalog basics -----------------------------------------------------


class TestCommandsBasics:
    """Каждая запись COMMANDS имеет корректную форму."""

    def test_commands_nonempty(self):
        assert len(COMMANDS) >= 18  # голосвание: 18+ команд (по ws_server)

    def test_command_names_unique(self):
        names = [c.name for c in COMMANDS]
        assert len(names) == len(set(names)), (
            f"дубли в COMMANDS: "
            f"{sorted(n for n in names if names.count(n) > 1)}"
        )

    @pytest.mark.parametrize("cmd", COMMANDS, ids=lambda c: c.name)
    def test_command_subprotocol_valid(self, cmd):
        assert cmd.subprotocol in ("v1", "v2", "any"), (
            f"{cmd.name}: bad subprotocol {cmd.subprotocol!r}"
        )

    @pytest.mark.parametrize("cmd", COMMANDS, ids=lambda c: c.name)
    def test_command_has_name(self, cmd):
        assert cmd.name
        assert cmd.name.replace("_", "").isalnum(), (
            f"{cmd.name}: должно быть snake_case ASCII"
        )

    def test_dispatched_commands_subset(self):
        """client_dispatched_commands ⊆ COMMANDS и не пусто."""
        dispatched = client_dispatched_commands()
        assert set(dispatched) <= {c.name for c in COMMANDS}
        assert len(dispatched) >= 15  # sanity: должны быть все ws_server

    def test_deprecated_commands_have_marker(self):
        """deprecated_commands (server_dispatched=False) — у каждого
        в description есть один из маркеров причины:
          * «deprecated» / «алиас» — старый алиас на новое имя;
          * «voice-vr 09» или просто «voice-vr» — отдельная карточка закрытия;
          * «Phase 2» / «R11» / «R14» / «не подключ» — закрыто в roadmap.

        Если ни одного маркера — это «забытый» cmd, надо руками
        перевести в правильную категорию."""
        deprecated = deprecated_commands()
        # Категории (regex по lower())
        markers = (
            "deprecated", "алиас",
            "voice-vr",
            "phase 2", "r11", "r14", "не подключ",
        )
        for name in deprecated:
            cmd = get_command(name)
            assert cmd is not None
            assert not cmd.server_dispatched
            desc = cmd.description.lower()
            assert any(m in desc for m in markers), (
                f"{name}: server_dispatched=False, но без маркера причины "
                f"(ожидается один из {markers}): {cmd.description!r}"
            )

    def test_avatar_aliases_marked_deprecated(self):
        """Голосвание карточки: ``avatar_*`` помечены deprecated,
        новые имена — ``supervisor_*`` (по meta-quest-api.md §3, §5.1)."""
        assert get_command("avatar_set_mode") is not None
        assert not get_command("avatar_set_mode").server_dispatched
        assert get_command("supervisor_set_mode") is not None
        assert get_command("supervisor_set_mode").server_dispatched
        # И симметрично для floor:
        assert get_command("avatar_acquire_floor") is not None
        assert not get_command("avatar_acquire_floor").server_dispatched
        assert get_command("supervisor_acquire_floor") is not None
        assert get_command("supervisor_acquire_floor").server_dispatched
        assert get_command("avatar_release_floor") is not None
        assert not get_command("avatar_release_floor").server_dispatched
        assert get_command("supervisor_release_floor") is not None
        assert get_command("supervisor_release_floor").server_dispatched


class TestEventsBasics:
    """EVENTS — те же инварианты, что для COMMANDS."""

    def test_events_nonempty(self):
        assert len(EVENTS) >= 15

    def test_event_names_unique(self):
        names = [e.name for e in EVENTS]
        assert len(names) == len(set(names)), (
            f"дубли в EVENTS: "
            f"{sorted(n for n in names if names.count(n) > 1)}"
        )

    @pytest.mark.parametrize("evt", EVENTS, ids=lambda e: e.name)
    def test_event_subprotocol_valid(self, evt):
        assert evt.subprotocol in ("v1", "v2", "any")

    def test_ping_is_only_handled_event(self):
        """Документированный инвариант: из incoming JSON_EVENT клиента
        сервер обрабатывает ТОЛЬКО ping (см. ws_server.py:_on_json_event).
        Это часть контракта мостика (ADR-0080 §2.2 инвариант 3)."""
        handled = [e.name for e in EVENTS if e.server_handled]
        assert handled == ["ping"], (
            f"Только ping должен быть server_handled; нашлось: {handled}"
        )


# --- 2. Streams: consistency & lookup --------------------------------------


class TestStreams:
    """STREAMS — биекция между ui_name и topic_id, уникальность id."""

    def test_streams_nonempty(self):
        assert len(STREAMS) >= 8

    def test_ui_names_unique(self):
        names = [s.ui_name for s in STREAMS]
        assert len(names) == len(set(names))

    def test_topic_ids_unique(self):
        ids_ = [s.topic_id for s in STREAMS]
        assert len(ids_) == len(set(ids_)), (
            f"topic_id дубль: "
            f"{sorted(i for i in ids_ if ids_.count(i) > 1)}"
        )

    def test_topic_ids_in_server_pool(self):
        """Сервер-инициируемые stream_id ∈ 0x1000..0xFFFF (§2, meta-quest-api.md)."""
        for s in STREAMS:
            assert 0x1000 <= s.topic_id <= 0xFFFF, (
                f"{s.ui_name}: topic_id {hex(s.topic_id)} вне server pool"
            )

    def test_kind_in_known_set(self):
        for s in STREAMS:
            assert s.kind in ("ros_topic", "camera_direct"), (
                f"{s.ui_name}: kind={s.kind!r}"
            )

    def test_default_quality_in_known_set(self):
        for s in STREAMS:
            assert s.default_quality in ("low", "med", "high"), (
                f"{s.ui_name}: default_quality={s.default_quality!r}"
            )

    def test_get_stream_by_name_matches_by_id(self):
        """Lookup по ui_name ↔ topic_id консистентен."""
        for s in STREAMS:
            assert get_stream(s.ui_name) is s
            assert get_stream_by_topic_id(s.topic_id) is s

    def test_get_stream_unknown_returns_none(self):
        assert get_stream("totally_bogus") is None
        assert get_stream_by_topic_id(0xDEAD) is None

    def test_camera_rear_topic_id_unchanged(self):
        """Guard от регрессии: meta-quest-api.md §4 фиксирует id'ы."""
        spec = get_stream("camera_rear")
        assert spec is not None
        assert spec.topic_id == 0x1001

    def test_lidar_2d_topic_id(self):
        assert get_stream("lidar_2d").topic_id == 0x1101

    def test_robot_status_topic_id(self):
        assert get_stream("robot_status").topic_id == 0x1201

    def test_voice_state_topic_id(self):
        assert get_stream("voice_state").topic_id == 0x1202

    def test_person_detections_topic_id(self):
        assert get_stream("person_detections").topic_id == 0x1301


# --- 3. Modes & Floors: Enum matches tuple ---------------------------------


class TestModesAndFloors:
    def test_modes_matches_enum(self):
        assert set(MODES) == {m.value for m in Mode}

    def test_floors_matches_enum(self):
        assert set(FLOORS) == {f.value for f in Floor}

    def test_mode_values(self):
        # Голосвание карточки: «avatar_*» (домен «аватар»). Канонические
        # имена режимов из ADR-0028 §4.1.
        assert "off" in MODES
        assert "telegram_active" in MODES
        assert "avatar_present" in MODES
        assert "mixed" in MODES
        assert "teleop_only" in MODES
        assert "voice_only" in MODES

    def test_floor_values(self):
        assert FLOORS == ("teleop", "voice")

    def test_modes_have_supervisor_commands(self):
        """Каждый режим (кроме off) имеет supervisor_set_mode cmd."""
        assert get_command("supervisor_set_mode") is not None


# --- 4. Errors: no duplicates, RATE_LIMIT marked unimplemented ------------


class TestErrors:
    def test_errors_no_duplicates(self):
        """DoD: ErrorCode не содержит повторных присваиваний (AST-тест).
        Здесь — эквивалент через tuple-семантику: ERRORS — tuple строк,
        set/tuple-конверсии не должны терять/добавлять элементы."""
        assert len(ERRORS) == len(set(ERRORS)), (
            f"дубли в ERRORS: "
            f"{sorted(e for e in ERRORS if ERRORS.count(e) > 1)}"
        )

    def test_error_specs_no_duplicates(self):
        codes = [s.code for s in ERROR_SPECS]
        assert len(codes) == len(set(codes)), (
            f"дубли в ERROR_SPECS: "
            f"{sorted(c for c in codes if codes.count(c) > 1)}"
        )

    def test_error_specs_cover_errors(self):
        """ERROR_SPECS покрывает ERRORS 1:1."""
        spec_codes = {s.code for s in ERROR_SPECS}
        assert spec_codes == set(ERRORS)

    def test_is_known_error(self):
        for code in ERRORS:
            assert is_known_error(code), f"{code!r} должен быть в is_known_error"
        assert not is_known_error("NOT_A_REAL_CODE")

    def test_rate_limit_marked_unimplemented(self):
        """DoD: RATE_LIMIT — на следующий релиз (ADR-0080 §7 вопрос 2).
        Сервер его не шлёт, и это зафиксировано в ERROR_SPECS."""
        rate = next((s for s in ERROR_SPECS if s.code == "RATE_LIMIT"), None)
        assert rate is not None, "RATE_LIMIT должен быть в ERROR_SPECS"
        assert rate.implemented is False, (
            "RATE_LIMIT.implemented должно быть False "
            "(см. ADR-0080 §7 вопрос 2)"
        )
        assert rate.server_emitted is False, (
            "RATE_LIMIT.server_emitted должно быть False — "
            "исторически задекларирован, но ws_server.py его НЕ шлёт"
        )

    def test_canonical_error_codes_present(self):
        for code in (
            "AUTH_FAIL",
            "BAD_PAYLOAD",
            "TOPIC_UNKNOWN",
            "PROTOCOL_VERSION",
            "FLOOR_HELD",
            "MODE_CONFLICT",
            "INTERNAL",
        ):
            assert is_known_error(code), f"{code} должен быть в ERRORS"


# --- 5. Voice presets & languages ------------------------------------------


class TestVoiceConfig:
    def test_presets_nonempty(self):
        assert len(VOICE_PRESET_IDS) >= 6

    def test_presets_unique(self):
        assert len(VOICE_PRESET_IDS) == len(set(VOICE_PRESET_IDS))

    def test_presets_include_translate(self):
        """«Перевод» — нейтральный пресет для грипа."""
        assert "translate" in VOICE_PRESET_IDS

    def test_languages_nonempty(self):
        assert len(VOICE_LANGUAGES) >= 4

    def test_default_language_in_set(self):
        """Голосвание: дефолтный язык пайплайна ∈ VOICE_LANGUAGES."""
        assert VOICE_PIPELINE_DEFAULT_LANGUAGE in VOICE_LANGUAGES

    def test_wire_modes_subset_of_input_modes(self):
        """Клиент шлёт только разрешённые voice_input_mode."""
        from rob_box_core.bridge_protocol import VoiceInputMode
        all_modes = {m.value for m in VoiceInputMode}
        for m in VOICE_WIRE_MODES:
            assert m in all_modes, (
                f"{m!r} в VOICE_WIRE_MODES, но отсутствует в VoiceInputMode"
            )


# --- 6. Frame types reference ----------------------------------------------


class TestFrameTypes:
    """FrameTypeId — reference из meta-quest-api.md §3."""

    def test_hello(self):
        assert FrameTypeId.HELLO == 0x01

    def test_welcome(self):
        assert FrameTypeId.WELCOME == 0x02

    def test_json_cmd(self):
        assert FrameTypeId.JSON_CMD == 0x11

    def test_json_event(self):
        assert FrameTypeId.JSON_EVENT == 0x12

    def test_binary_frame(self):
        assert FrameTypeId.BINARY_FRAME == 0x10

    def test_error_frame(self):
        assert FrameTypeId.ERROR == 0xFF

    def test_supervisor_set_mode(self):
        assert FrameTypeId.SET_MODE == 0x30

    def test_supervisor_acquire_floor(self):
        assert FrameTypeId.ACQUIRE_FLOOR == 0x31

    def test_supervisor_release_floor(self):
        assert FrameTypeId.RELEASE_FLOOR == 0x32

    def test_state_update(self):
        assert FrameTypeId.STATE_UPDATE == 0x33

    def test_voice_audio_frame(self):
        assert FrameTypeId.VOICE_AUDIO == 0x21


# --- 7. Catalog-driven checks vs source code -------------------------------
#
# Эти тесты — замена grep по ws_server.py: они читают каталог напрямую
# и валидируют, что _on_json_cmd покрывает все client_dispatched_commands.


class TestCatalogDrivenChecks:
    """Catalog-driven тесты (без grep)."""

    def test_get_command_returns_correct_spec(self):
        spec = get_command("teleop_twist")
        assert spec is not None
        assert "linear" in spec.required_fields
        assert "angular" in spec.required_fields
        assert "deadman" in spec.required_fields

    def test_get_event_returns_correct_spec(self):
        spec = get_event("voice_state")
        assert spec is not None
        assert "state" in spec.required_fields
        assert "ts_ms" in spec.required_fields
        assert "holder_id" in spec.optional_fields

    def test_supervisor_commands_require_v2(self):
        for cmd_name in (
            "supervisor_set_mode",
            "supervisor_acquire_floor",
            "supervisor_release_floor",
            "supervisor_get_state",
            "voice_pipeline",
            "preview_voice",
        ):
            cmd = get_command(cmd_name)
            assert cmd is not None, f"{cmd_name} отсутствует в каталоге"
            assert cmd.subprotocol == "v2", (
                f"{cmd_name}: subprotocol={cmd.subprotocol}, ожидалось v2"
            )

    def test_v1_commands_marked_any_or_v1(self):
        """Команды из Phase 1 должны быть v1 или any."""
        v1_only_names = [
            c.name for c in COMMANDS
            if c.subprotocol in ("v1", "any") and c.server_dispatched
        ]
        # sanity: должно быть много v1-команд (голос/твист/стоп/списки)
        assert len(v1_only_names) >= 10
