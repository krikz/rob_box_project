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

    def test_avatar_canonical_and_supervisor_alias_both_dispatched(self):
        """[voice-vr 09 / issue #2194] закрыт: ``avatar_*`` — канонические
        имена (ADR-0080 §1.2), ``supervisor_*`` — legacy-алиасы. Сервер
        диспатчит ОБА набора имён (см. ws_server.py, блок «канонические
        имена» рядом с legacy supervisor_* алиасом).

        До закрытия voice-vr 09 этот тест (тогда назывался
        ``test_avatar_aliases_marked_deprecated``) проверял обратное —
        что именно ``avatar_*`` не диспатчится, а ``supervisor_*`` — новое
        имя. Это было названо неверно ещё на этапе постановки задачи:
        канон — ``avatar_*`` (ADR-0080 §1.2), а ``supervisor_*`` остаётся
        рабочим legacy-алиасом. Инвертировано вместе с фиксом бага."""
        assert get_command("avatar_set_mode") is not None
        assert get_command("avatar_set_mode").server_dispatched
        assert get_command("supervisor_set_mode") is not None
        assert get_command("supervisor_set_mode").server_dispatched
        # И симметрично для floor:
        assert get_command("avatar_acquire_floor") is not None
        assert get_command("avatar_acquire_floor").server_dispatched
        assert get_command("supervisor_acquire_floor") is not None
        assert get_command("supervisor_acquire_floor").server_dispatched
        assert get_command("avatar_release_floor") is not None
        assert get_command("avatar_release_floor").server_dispatched
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

    def test_default_language_is_single_source_of_truth(self):
        """issue #2265 — больше НЕТ трёх копий «ru».

        ``supervisor_node.GRIP_DEFAULT_LANGUAGE`` и
        ``ws_server.VOICE_PIPELINE_DEFAULT_LANGUAGE`` ОБЯЗАНЫ быть
        re-export'ами канонической константы из ``bridge_protocol``
        (тот же объект, что и здесь). Если кто-то снова заведёт
        локальную копию — этот тест поймает на первом же запуске CI,
        без ручного вычитывания номеров строк в комментариях.

        Конструкция «try/except ImportError» нужна потому, что
        ws_server тащит за собой tornado; supervisor_node — rclpy.
        Оба пакета устанавливаются в G-Run Tests.yml, но здесь мы
        хотим лёгкий guard, который едет даже на голой машине без
        ROS2 — поэтому проверяем только то, что доступно.
        """
        import importlib

        canonical = VOICE_PIPELINE_DEFAULT_LANGUAGE
        assert canonical == "ru", (
            "Дефолтный язык пайплайна сменился — обнови комментарий и "
            "убедись, что voice_presets.yaml / ws_server всё ещё "
            "принимают это значение как валидный language_id."
        )

        # ws_server — pure Python (tornado присутствует в CI), проверяем
        # равенство re-export'а канону. Но workflow G-Bridge-Protocol-Drift
        # ставит только ``rob_box_core`` (без ``rob_box_quest``), чтобы
        # держать CI дёшёво; без try/except тест валится на голом CI,
        # который НЕ должен ловить эту регрессию — для этого есть
        # ``test_supervisor_node.test_grip_default_language_is_re_export_of_catalog``
        # в G-Run Tests.yml. ModuleNotFoundError (нет пакета) → skip,
        # ImportError (сам модуль сломан) → fail loud.
        try:
            ws_server = importlib.import_module("rob_box_quest.server.ws_server")
        except ModuleNotFoundError as exc:
            if "rob_box_quest" not in str(exc):
                raise
            return  # rob_box_quest не установлен — пропускаем локально.
        assert ws_server.VOICE_PIPELINE_DEFAULT_LANGUAGE is canonical, (
            "ws_server.VOICE_PIPELINE_DEFAULT_LANGUAGE оторвался от "
            "канона — верни прямой импорт из bridge_protocol."
        )

        # supervisor_node — ROS-узел (rclpy в CI); импорт может
        # провалиться на голой машине. В этом случае доверяемся
        # смоук-тесту в test_supervisor_node.py, который запускается
        # в G-Run Tests.yml.
        try:
            supervisor = importlib.import_module(
                "rob_box_supervisor.supervisor_node"
            )
        except ImportError as exc:
            if "rclpy" not in str(exc):
                raise
            return  # ROS не установлен — пропускаем локально.

        assert supervisor.GRIP_DEFAULT_LANGUAGE is canonical, (
            "supervisor_node.GRIP_DEFAULT_LANGUAGE оторвался от "
            "канона — верни прямой импорт из bridge_protocol."
        )

    def test_wire_modes_subset_of_input_modes(self):
        """Клиент шлёт только разрешённые voice_input_mode."""
        from rob_box_core.bridge_protocol import VoiceInputMode
        all_modes = {m.value for m in VoiceInputMode}
        for m in VOICE_WIRE_MODES:
            assert m in all_modes, (
                f"{m!r} в VOICE_WIRE_MODES, но отсутствует в VoiceInputMode"
            )


class TestVoiceConfigYamlDrivesWhitelist:
    """voice-vr 21 / ADR-0080 §2.7 — DoD-3:

    Добавление пресета в ``voice_presets.yaml`` доезжает до
    ``VOICE_PRESET_IDS`` / ``VOICE_LANGUAGES`` БЕЗ правки Python.
    Тест подменяет путь к yaml через ``monkeypatch`` на временный
    файл с тем же shape и проверяет, что константы его отражают.

    Дополнительно фиксирует, что реэкспорт в ws_server
    (через повторный импорт) тоже подхватывает обновлённый
    список — это и есть «один белый список» (ADR-0080 §2.7).
    """

    def _write_yaml(self, tmp_path, presets, languages):
        """Записать минимальный ``voice_presets.yaml`` и вернуть путь."""
        # Минимальный, но валидный формат — секции presets/languages;
        # остальные поля yaml не влияют на VOICE_*_IDS.
        lines = ["presets:"]
        for key, name in presets.items():
            lines.append(f"  {key}:")
            lines.append(f'    name: "{name}"')
        lines.append("languages:")
        for code, meta in languages.items():
            lines.append(f"  {code}:")
            lines.append(f'    name: "{meta["name"]}"')
        path = tmp_path / "voice_presets.yaml"
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def test_added_preset_in_yaml_reaches_voice_preset_ids(
        self, monkeypatch, tmp_path
    ):
        """Новый ключ в yaml → присутствует в VOICE_PRESET_IDS.

        Подменяем ``_resolve_voice_presets_yaml_path`` через
        monkeypatch.setattr — так тест НЕ правит реальный yaml и
        НЕ зависит от colcon/ament-share (source-tree lookup).
        """
        from rob_box_core import bridge_protocol

        yaml_path = self._write_yaml(
            tmp_path,
            presets={
                "technical": "Технический",
                "new_one": "Новый пресет",
                "another": "Другой",
            },
            languages={"ru": {"name": "Русский"}, "en": {"name": "English"}},
        )
        monkeypatch.setattr(
            bridge_protocol, "_resolve_voice_presets_yaml_path", lambda: str(yaml_path)
        )

        preset_ids, language_ids = bridge_protocol._load_voice_lists_from_yaml()
        assert "new_one" in preset_ids
        assert "another" in preset_ids
        # Старые ключи из yaml не должны быть потеряны.
        assert "technical" in preset_ids
        # Languages — тоже из yaml.
        assert set(language_ids) == {"ru", "en"}

    def test_yaml_change_picked_up_on_subprocess_restart(
        self, tmp_path
    ):
        """После смены yaml + новый процесс — константы свежие.

        Это вторая половина DoD-3: между запусками робота yaml
        может поменяться (например, через ``cp`` новой версии
        в ``/ws/install/.../share/rob_box_voice/config/``).
        Контракт «yaml → whitelist» не должен требовать правки
        Python; достаточно перезапустить процесс (новый import
        модуля = новое чтение yaml).

        Тест запускает ``importlib.reload(bridge_protocol)`` в
        дочернем Python-процессе с подменённым путём к yaml —
        изоляция от родителя + реальный «новый import» с нуля,
        без monkeypatch-артефактов.
        """
        import os
        import subprocess
        import sys
        import textwrap
        from pathlib import Path

        # Репозиторий, не ``tmp_path`` (тот — системный temp-каталог,
        # ``tmp_path.parent.parent`` НЕ репо-рут ни на Windows, ни на
        # Linux CI — с этим cwd child всегда падал в
        # ``ModuleNotFoundError: No module named 'rob_box_core'``,
        # независимо от PYTHONPATH). Этот файл лежит в
        # ``<repo>/src/rob_box_core/test/test_bridge_protocol.py``,
        # поэтому repo root = parents[3].
        repo_root = Path(__file__).resolve().parents[3]

        yaml_text = textwrap.dedent(
            """\
            presets:
              pirate:
                name: "Пират"
              wizard:
                name: "Маг"
            languages:
              ru:
                name: "Русский"
              jp:
                name: "日本語"
            """
        )
        yaml_path = tmp_path / "voice_presets.yaml"
        yaml_path.write_text(yaml_text, encoding="utf-8")

        # Подменяем резолвер через env var ROB_BOX_VOICE_PRESETS_YAML —
        # чистый путь «как на роботе после деплоя»: тот же бинарь,
        # другой yaml на диске (или просто env override). Import + reload
        # идут стандартно, monkeypatch НЕ нужен.
        child_script = textwrap.dedent(
            """
            import importlib
            from rob_box_core import bridge_protocol
            reloaded = importlib.reload(bridge_protocol)
            print("PRESETS", ",".join(reloaded.VOICE_PRESET_IDS))
            print("LANGUAGES", ",".join(reloaded.VOICE_LANGUAGES))
            """
        )
        result = subprocess.run(
            [sys.executable, "-c", child_script],
            capture_output=True,
            text=True,
            check=False,
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join(
                    [
                        str(repo_root / "src" / "rob_box_voice"),
                        str(repo_root / "src" / "rob_box_llm"),
                        str(repo_root / "src" / "rob_box_core"),
                        str(repo_root / "src" / "rob_box_harness"),
                    ]
                ),
                "ROB_BOX_VOICE_PRESETS_YAML": str(yaml_path),
            },
            cwd=str(repo_root),
            timeout=30,
        )
        assert result.returncode == 0, (
            f"child failed: rc={result.returncode} stdout={result.stdout!r} "
            f"stderr={result.stderr!r}"
        )
        presets_line = next(
            (l for l in result.stdout.splitlines() if l.startswith("PRESETS ")),
            "",
        )
        languages_line = next(
            (l for l in result.stdout.splitlines() if l.startswith("LANGUAGES ")),
            "",
        )
        assert presets_line, f"no PRESETS in child stdout: {result.stdout!r}"
        assert languages_line, f"no LANGUAGES in child stdout: {result.stdout!r}"
        child_presets = presets_line.split(" ", 1)[1].split(",")
        child_languages = languages_line.split(" ", 1)[1].split(",")
        assert "pirate" in child_presets, (
            f"DoD-3: добавление пресета в yaml должно доезжать до "
            f"VOICE_PRESET_IDS без правки Python; got {child_presets!r}"
        )
        assert "wizard" in child_presets
        assert "jp" in child_languages
        # И наоборот: НЕ должно быть preset'ов, которых нет в yaml.
        # (Защита от регрессии «FALLBACK приклеился к yaml».)
        for forbidden in ("technical", "street", "caveman", "lenin", "translate"):
            assert forbidden not in child_presets, (
                f"DoD-3: yaml не содержит {forbidden!r}, но child-процесс "
                f"его видит — FALLBACK просочился: {child_presets!r}"
            )

    def test_yaml_missing_falls_back_to_hardcoded(self, monkeypatch):
        """yaml недоступен → откат на FALLBACK-туплу (CI-минимум).

        Это страховка: модуль должен импортироваться БЕЗ yaml
        (conftest, минимальный CI), и тогда константы всё равно
        содержат рабочий минимум (``technical``, ``translate``,
        ``ru``, ``en``).
        """
        from rob_box_core import bridge_protocol

        monkeypatch.setattr(
            bridge_protocol, "_resolve_voice_presets_yaml_path", lambda: None
        )
        preset_ids, language_ids = bridge_protocol._load_voice_lists_from_yaml()
        assert preset_ids == bridge_protocol._VOICE_PRESET_IDS_FALLBACK
        assert language_ids == bridge_protocol._VOICE_LANGUAGES_FALLBACK
        # Минимум для UI/тестов: «translate» (нейтральный пресет) и
        # «ru» (дефолтный язык).
        assert "translate" in preset_ids
        assert "ru" in language_ids

    def test_yaml_list_languages_legacy_format(self, monkeypatch, tmp_path):
        """Поддержка старого списочного формата ``languages: [ru, en]``.

        Исторический комментарий в yaml упоминает обратную совместимость;
        тест фиксирует: даже если в yaml ``languages: [...]`` —
        константы собираются из списка.
        """
        from rob_box_core import bridge_protocol

        yaml_path = tmp_path / "voice_presets.yaml"
        yaml_path.write_text(
            "presets:\n  tech: {name: \"T\"}\nlanguages: [ru, en, de]\n",
            encoding="utf-8",
        )
        monkeypatch.setattr(
            bridge_protocol, "_resolve_voice_presets_yaml_path", lambda: str(yaml_path)
        )
        _, language_ids = bridge_protocol._load_voice_lists_from_yaml()
        assert language_ids == ("ru", "en", "de")


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
