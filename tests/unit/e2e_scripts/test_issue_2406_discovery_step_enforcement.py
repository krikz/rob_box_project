"""test_issue_2406_discovery_step_enforcement.py — issue #2406 regression.

Bug class: LLM отвечает verbal-only на discovery/inquiry-шаги, пропуская
первый обязательный tool. Воспроизводилось 3 раза подряд на develop HEAD
4ab3a0a (runs 363/364/365):

- Run 363 / act2 / n201_sasha_intro_long — missing ``register_speaker``.
- Run 364 / act3 / n313_silence_restored — missing ``get_music_state``.
- Run 365 / act4 / n401_list_voices — missing ``list_tts_voices``.

Fix (PR #2458 #2457 #2387) — RULE #DISCOVERY-TOOLS в master_prompt_compact.txt.
Этот тест ЗАЩИЩАЕТ от регрессии через acceptance.json schema: новое
опциональное поле ``discovery_tools``, которое чекер
(``check_acceptance`` в e2e_voice_test.sh, lines 1028+) валидирует как
«тул ОБЯЗАН быть вызван ДО первого голосового ответа».

Покрытие:
- TestFirstInvocationPosition: unit-тесты хелпера
  ``first_invocation_position`` из ``e2e_tool_match.py``.
- TestFirstVoiceCyclePosition: unit-тесты хелпера
  ``first_voice_cycle_position``.
- TestDiscoveryStepAcceptance: интеграционные тесты Python-валидатора
  из check_acceptance (per-step heredoc) на синтетических логах.
- TestNegativeVerbalOnly: явно-негативные кейсы — тул НЕ вызван / вызван
  ПОСЛЕ verbal ответа → должны падать с явной подсказкой про issue #2406.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_2406_discovery_step_enforcement.py -v --no-cov
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"

sys.path.insert(0, str(SCRIPTS_DIR))
from e2e_tool_match import (  # noqa: E402
    TOOL_NAME_RE,
    first_invocation_position,
    first_voice_cycle_position,
    tool_invoked,
)


# ── синтетические фрагменты лога ──────────────────────────────────────────
#
# Шаблон «нормального» хода с discovery tool ДО verbal ответа:
#   1. dialogue_node: tools(56) catalog
#   2. LLM решает вызвать tool — assistant tool_calls=[...]
#   3. mcp_server выполняет tool
#   4. dialogue_node: tools=['register_speaker', 'speak_text']
#   5. tts finished
#   6. ✅ [turn] process_input returned: spoken='...'

TOOLS_CATALOG = (
    "[dialogue_node-4]   tools(56): clear_waypoints, compose_music, "
    "continue_mapping, delete_track, delete_waypoint, estimate_tts_duration, "
    "execute_music_code, faq_search, finish_mapping, gen_delete_from_library, "
    "get_battery_level, get_current_time, get_music_state, move_direction, "
    "play_animation, play_sound, register_speaker, save_track, set_dj_mode, "
    "set_voice, set_volume, speak_text, start_mapping, stop_music, "
    "stop_navigation, task_delta, load_skill\n"
)

#: «Хороший» ход: tool вызван ДО speak_text (n201 regression-passing).
TURN_TOOL_BEFORE_VOICE = TOOLS_CATALOG + (
    "[dialogue_node-4] 📤 Отправлен запрос b5119339: register_speaker\n"
    "[mcp_server-10] 📥 Запрос выполнения: register_speaker с параметрами "
    "{'name': 'Саша'}\n"
    "[mcp_server-10] ✅ Инструмент register_speaker выполнен успешно\n"
    "[dialogue_node-4]   [3] assistant: '' tool_calls=(ToolCall("
    "id='call_01a0', name='register_speaker', "
    "arguments=mappingproxy({'name': 'Саша'})),)\n"
    "[dialogue_node-4] ✅ [turn] process_input returned: "
    "spoken='Приятно познакомиться, Саша!'[:60] "
    "tools=['register_speaker', 'speak_text'] finish_reason='stop'\n"
)

#: «Регресс #2406»: LLM ответил голосом БЕЗ вызова register_speaker
#: (запустил tts сразу после catalog). Точный паттерн багов из runs 363/364/365.
TURN_VERBAL_ONLY = TOOLS_CATALOG + (
    "[dialogue_node-4]   [3] assistant: '' tool_calls=()\n"
    "[dialogue_node-4] ✅ [turn] process_input returned: "
    "spoken='Приятно познакомиться, Саша!'[:60] "
    "tools=[] finish_reason='stop'\n"
)

#: «Регресс #2406 наоборот»: tool вызван, НО ПОСЛЕ голосового ответа
#: (LLM ответил текстом, потом опомнился и дёрнул tool — GATE-1 PASS,
#: issue #2406 говорит что должно быть FAIL).
TURN_VOICE_BEFORE_TOOL = TOOLS_CATALOG + (
    "[dialogue_node-4] ✅ [turn] process_input returned: "
    "spoken='Приятно познакомиться, Саша!'[:60] "
    "tools=[] finish_reason='stop'\n"
    "[dialogue_node-4] 📤 Отправлен запрос b5119339: register_speaker\n"
    "[mcp_server-10] 📥 Запрос выполнения: register_speaker с параметрами "
    "{'name': 'Саша'}\n"
    "[mcp_server-10] ✅ Инструмент register_speaker выполнен успешно\n"
)


# ── TestFirstInvocationPosition ───────────────────────────────────────────


class TestFirstInvocationPosition:
    """``first_invocation_position(logs, tool) → int | None``."""

    def test_returns_position_when_tool_invoked(self) -> None:
        pos = first_invocation_position(TURN_TOOL_BEFORE_VOICE, "register_speaker")
        assert pos is not None
        assert pos >= 0
        # Позиция должна указывать на ЛЮБОЙ из маркеров execution:
        # 'register_speaker' в кавычках, "запрос выполнения: register_speaker",
        # "инструмент register_speaker выполнен". Мы берём lower-case лог
        # и проверяем, что в позиции pos начинается хотя бы один маркер.
        low = TURN_TOOL_BEFORE_VOICE.lower()
        assert low[pos:].startswith(
            ("'register_speaker'", '"register_speaker"',
             "запрос выполнения: register_speaker",
             "инструмент register_speaker выполнен",
             "публикую результат для register_speaker")
        )

    def test_returns_none_when_tool_not_invoked(self) -> None:
        assert first_invocation_position(TURN_VERBAL_ONLY, "register_speaker") is None

    def test_returns_position_only_for_real_invocation(self) -> None:
        # catalog «tools(56): ...» НЕ должен считаться вызовом —
        # позиция первого маркера execution вне catalog-строки.
        pos = first_invocation_position(TURN_TOOL_BEFORE_VOICE, "stop_music")
        # stop_music НЕ вызывался в этом ходе — None.
        assert pos is None

    def test_catalog_alone_returns_none(self) -> None:
        """Каталог доступных тулов НЕ считается execution-маркером."""
        assert first_invocation_position(TOOLS_CATALOG, "stop_music") is None
        assert first_invocation_position(TOOLS_CATALOG, "register_speaker") is None

    def test_free_text_returns_none(self) -> None:
        """Не-имя тула → None (контракт TOOL_NAME_RE)."""
        assert first_invocation_position(TURN_TOOL_BEFORE_VOICE, "") is None
        assert first_invocation_position(TURN_TOOL_BEFORE_VOICE, None) is None  # type: ignore[arg-type]
        assert first_invocation_position(TURN_TOOL_BEFORE_VOICE, "STOP command received") is None
        assert first_invocation_position(TURN_TOOL_BEFORE_VOICE, "не тул") is None


# ── TestFirstVoiceCyclePosition ───────────────────────────────────────────


class TestFirstVoiceCyclePosition:
    """``first_voice_cycle_position(logs) → int | None``."""

    def test_returns_position_on_spoken_marker(self) -> None:
        pos = first_voice_cycle_position(TURN_TOOL_BEFORE_VOICE)
        assert pos is not None
        low = TURN_TOOL_BEFORE_VOICE.lower()
        # Маркер должен совпадать с префиксом low[pos:].
        assert low[pos:].startswith(
            ("✅ [turn] process_input returned:", "'speak_text'", "инструмент speak_text выполнен", "tts finished")
        )

    def test_returns_none_when_no_voice_cycle(self) -> None:
        assert first_voice_cycle_position(TOOLS_CATALOG) is None

    def test_earliest_marker_wins(self) -> None:
        """Если в логе несколько маркеров — возвращаем самый ранний."""
        log = (
            "tools=['speak_text', 'register_speaker']\n"  # 'speak_text' в tools= (раньше)
            "tts finished\n"                              # tts finished (позже)
            "✅ [turn] process_input returned: spoken='x'\n"
        )
        pos = first_voice_cycle_position(log)
        assert pos is not None
        # Самый ранний execution-маркер в этом логе — "'speak_text'" внутри
        # tools=... (поиск идёт по подстроке, поэтому вернёт позицию
        # открывающей кавычки перед speak_text, а не саму 'tools=').
        assert log[pos:].startswith("'speak_text'")


# ── TestDiscoveryStepAcceptance (per-step validator) ───────────────────────
#
# Воспроизводим логику check_acceptance (per-step heredoc, e2e_voice_test.sh
# lines 1028+). Это позволит нам прогонять юнит-тесты без docker / робота.

def _per_step_validate(acc: dict, logs: str) -> dict:
    """Копия Python-валидатора из check_acceptance (per-step).

    Синхронна с тем, что в e2e_voice_test.sh:1041+. При расхождении —
    это drift, и unit-тест ловит его первым.
    """
    from e2e_tool_match import tool_invoked as _tool_invoked

    def has(s, frag):
        return _tool_invoked(s, frag)

    expected_call = acc.get("expected_tool_calls", []) or []
    must_not = acc.get("must_not_call", []) or []
    discovery_tools_raw = acc.get("discovery_tools", []) or []

    discovery_tools: list[str] = []
    discovery_tool_errors: list[str] = []
    for _dt in discovery_tools_raw:
        if not isinstance(_dt, str) or not TOOL_NAME_RE.match(_dt.lower()):
            discovery_tool_errors.append(
                f"discovery_tools entry {repr(_dt)} is not a tool name "
                f"(must match ^[a-z][a-z0-9_]*$)"
            )
            continue
        discovery_tools.append(_dt)

    actual_calls: list[str] = []
    for c in (expected_call + must_not):
        if has(logs, c) and c not in actual_calls:
            actual_calls.append(c)

    found_expected = [c for c in expected_call if has(logs, c)]
    missing_expected = [c for c in expected_call if not has(logs, c)]
    forbidden_called = [c for c in must_not if has(logs, c)]

    discovery_failures: list[str] = []
    discovery_records: list[dict] = []
    voice_pos = first_voice_cycle_position(logs) if discovery_tools else None
    for _dt in discovery_tools:
        tool_pos = first_invocation_position(logs, _dt)
        rec = {
            "tool": _dt,
            "first_invocation_pos": tool_pos,
            "first_voice_pos": voice_pos,
        }
        if tool_pos is None:
            discovery_failures.append(
                f"discovery tool {_dt!r} was NOT invoked at all "
                f"(issue #2406: LLM bypassed the required tool call)"
            )
        elif voice_pos is not None and tool_pos > voice_pos:
            discovery_failures.append(
                f"discovery tool {_dt!r} invoked at pos {tool_pos}, "
                f"AFTER verbal answer at pos {voice_pos} "
                f"(issue #2406: verbal-only LLM answer before required tool)"
            )
        discovery_records.append(rec)

    failures: list[str] = []
    if missing_expected:
        failures.append(f"expected tool calls not invoked: {missing_expected}")
    if forbidden_called:
        failures.append(f"forbidden tool calls invoked: {forbidden_called}")
    if discovery_tool_errors:
        failures.extend(discovery_tool_errors)
    if discovery_failures:
        failures.extend(discovery_failures)

    return {
        "expected_tool_calls": expected_call,
        "missing_expected_calls": missing_expected,
        "forbidden_calls": forbidden_called,
        "discovery_tools": discovery_tools,
        "discovery_records": discovery_records,
        "discovery_first_voice_pos": voice_pos,
        "failures": failures,
        "pass": not failures,
    }


class TestDiscoveryStepAcceptance:
    """Per-step Python-валидатор: поведение с discovery_tools полем."""

    def test_tool_before_voice_passes(self) -> None:
        """Tool вызван ДО verbal ответа → PASS.

        Реальный n201_sasha_intro_long, run 363 fix verification.
        """
        acc = {
            "expected_tool_calls": ["register_speaker"],
            "must_not_call": [],
            "discovery_tools": ["register_speaker"],
        }
        result = _per_step_validate(acc, TURN_TOOL_BEFORE_VOICE)
        assert result["pass"] is True
        assert result["failures"] == []
        assert result["discovery_records"][0]["first_invocation_pos"] is not None
        assert result["discovery_records"][0]["first_voice_pos"] is not None
        # tool_pos < voice_pos.
        assert (
            result["discovery_records"][0]["first_invocation_pos"]
            < result["discovery_records"][0]["first_voice_pos"]
        )

    def test_verbal_only_fails_with_issue_2406_marker(self) -> None:
        """LLM ответил verbal-only → FAIL с явной подсказкой issue #2406.

        Реальный регресс из run 363/364/365.
        """
        acc = {
            "expected_tool_calls": ["register_speaker"],
            "must_not_call": [],
            "discovery_tools": ["register_speaker"],
        }
        result = _per_step_validate(acc, TURN_VERBAL_ONLY)
        assert result["pass"] is False
        # Должны быть ОБЕ причины: expected tool не вызван И verbal-only.
        failures_str = " ".join(result["failures"])
        assert "#2406" in failures_str, (
            f"reason must reference issue #2406 for traceability, "
            f"got: {failures_str}"
        )
        assert "register_speaker" in failures_str
        assert "NOT invoked" in failures_str or "bypassed" in failures_str

    def test_voice_before_tool_fails_with_issue_2406_marker(self) -> None:
        """Tool вызван ПОСЛЕ verbal ответа → FAIL.

        Это случай из тела карточки: «current check passes if order varies»
        — старый чек пропустит, новый должен ловить.
        """
        acc = {
            "expected_tool_calls": ["register_speaker"],
            "must_not_call": [],
            "discovery_tools": ["register_speaker"],
        }
        result = _per_step_validate(acc, TURN_VOICE_BEFORE_TOOL)
        assert result["pass"] is False, (
            "verdict must FAIL when tool invoked AFTER verbal answer — "
            "this is the exact regression #2406 describes"
        )
        failures_str = " ".join(result["failures"])
        assert "#2406" in failures_str
        assert "AFTER verbal answer" in failures_str

    def test_discovery_field_optional(self) -> None:
        """Без discovery_tools — старый чек (только факт вызова).

        Backwards-compat: существующие acceptance.json работают как раньше.
        """
        acc = {
            "expected_tool_calls": ["register_speaker"],
            "must_not_call": [],
            # нет discovery_tools
        }
        # Старый чек: если register_speaker вызван — PASS, не важно до или после.
        result = _per_step_validate(acc, TURN_VOICE_BEFORE_TOOL)
        assert result["pass"] is True
        assert result["discovery_tools"] == []
        assert result["discovery_records"] == []

    def test_invalid_tool_name_in_discovery_fails(self) -> None:
        """Не snake_case имя в discovery_tools → soft FAIL с подсказкой.

        TOOL_NAME_RE = ^[a-z][a-z0-9_]*$, поэтому «Stop_music» / «не тул»
        / пустая строка должны быть отвергнуты.
        """
        acc = {
            "expected_tool_calls": [],
            "discovery_tools": ["Stop_music", "не тул", ""],
        }
        result = _per_step_validate(acc, TURN_TOOL_BEFORE_VOICE)
        assert result["pass"] is False
        failures_str = " ".join(result["failures"])
        assert "discovery_tools entry" in failures_str
        assert "^[a-z][a-z0-9_]*$" in failures_str

    def test_multiple_discovery_tools_independently_validated(self) -> None:
        """Каждый discovery tool валидируется отдельно.

        Пример из act4 / n401_list_voices: list_tts_voices вызван до voice,
        set_voice вызван ПОСЛЕ voice (это уже discovery fail, не expected).
        """
        log = (
            TOOLS_CATALOG
            + "[dialogue_node-4]   [3] assistant: '' tool_calls=(ToolCall("
              "name='list_tts_voices',),)\n"
            + "[mcp_server-10] ✅ Инструмент list_tts_voices выполнен успешно\n"
            + "[dialogue_node-4] ✅ [turn] process_input returned: "
              "spoken='у меня есть антон и алёна'[:60] "
              "tools=['list_tts_voices', 'speak_text']\n"
            # После voice — отдельный ход с set_voice (НЕ в этом тесте).
        )
        acc = {
            "discovery_tools": ["list_tts_voices"],
        }
        result = _per_step_validate(acc, log)
        assert result["pass"] is True, (
            f"list_tts_voices был вызван ДО speak_text — должно быть PASS, "
            f"got: {result['failures']}"
        )

    def test_discovery_tool_bypassed_among_real_tools(self) -> None:
        """Discovery tool пропущен, но другие тулы вызваны → FAIL.

        Типовая картина из issue #2406: LLM вызывает set_voice/set_volume/
        set_speed (мутаторы), но пропускает register_speaker (discovery).
        Старый чек expected_tool_calls это ловит, новый также ловит через
        discovery_tools (двойная защита).
        """
        log = (
            TOOLS_CATALOG
            + "[dialogue_node-4]   [3] assistant: '' tool_calls=(ToolCall("
              "name='set_voice',), ToolCall(name='set_volume',), "
              "ToolCall(name='set_speed',),)\n"
            + "[mcp_server-10] ✅ Инструмент set_voice выполнен успешно\n"
            + "[mcp_server-10] ✅ Инструмент set_volume выполнен успешно\n"
            + "[mcp_server-10] ✅ Инструмент set_speed выполнен успешно\n"
            + "[dialogue_node-4] ✅ [turn] process_input returned: "
              "spoken='готово'[:60] "
              "tools=['set_voice', 'set_volume', 'set_speed', 'speak_text']\n"
        )
        acc = {
            "expected_tool_calls": ["set_voice"],
            "discovery_tools": ["register_speaker"],
        }
        result = _per_step_validate(acc, log)
        # set_voice вызван → expected_tool_calls OK;
        # register_speaker НЕ вызван → discovery FAIL.
        assert result["pass"] is False
        failures_str = " ".join(result["failures"])
        assert "register_speaker" in failures_str
        assert "NOT invoked" in failures_str


# ── TestNegativeVerbalOnly (явный negative) ────────────────────────────────


class TestNegativeVerbalOnly:
    """Явные негативные кейсы — те самые 3 фейла issue #2406."""

    @pytest.mark.parametrize(
        ("label", "discovery_tool"),
        [
            ("n201_sasha_intro_long", "register_speaker"),
            ("n313_silence_restored", "get_music_state"),
            ("n401_list_voices", "list_tts_voices"),
        ],
    )
    def test_real_verbal_only_patterns_fail(self, label: str, discovery_tool: str) -> None:
        """Реальные паттерны регрессии issue #2406.

        Run 363 / n201 — verbal-only «Приятно познакомиться, Саша!»
        Run 364 / n313 — verbal-only «Сейчас тишина» (без get_music_state)
        Run 365 / n401 — verbal-only «У меня только антон» (без list_tts_voices)
        """
        # Синтетический verbal-only лог для каждого тула.
        verbal_log = (
            TOOLS_CATALOG
            + f"[dialogue_node-4] ✅ [turn] process_input returned: "
              f"spoken='verbal-only-answer-for-{discovery_tool}'[:60] "
              f"tools=[] finish_reason='stop'\n"
        )
        acc = {
            "discovery_tools": [discovery_tool],
        }
        result = _per_step_validate(acc, verbal_log)
        assert result["pass"] is False, (
            f"{label}: discovery tool {discovery_tool!r} not invoked, "
            f"verbal-only answer — должно быть FAIL"
        )
        failures_str = " ".join(result["failures"])
        assert discovery_tool in failures_str
        assert "NOT invoked" in failures_str


# ── TestScenarioAcceptanceFiles (backfill в night_marathon) ────────────────


class TestScenarioAcceptanceFiles:
    """Сами acceptance.json файлы в act2/act3/act4 должны содержать
    discovery_tools для соответствующих шагов."""

    @pytest.fixture
    def scenarios(self) -> dict[str, Path]:
        return {
            "act2": REPO_ROOT / ".github/e2e/scenarios/night/"
                    "night_marathon_act2_acquaintance_v1.json",
            "act3": REPO_ROOT / ".github/e2e/scenarios/night/"
                    "night_marathon_act3_backlog_diarization_v1.json",
            "act4": REPO_ROOT / ".github/e2e/scenarios/night/"
                    "night_marathon_act4_voice_prosody_v1.json",
        }

    @pytest.mark.parametrize(
        ("act", "step_label", "expected_discovery"),
        [
            ("act2", "n201_sasha_intro_long", ["register_speaker"]),
            ("act2", "n204_boris_intro_long", ["register_speaker"]),
            ("act3", "n313_silence_restored", ["get_music_state"]),
            ("act4", "n401_list_voices", ["list_tts_voices"]),
        ],
    )
    def test_scenario_step_has_discovery_tools(
        self, scenarios: dict[str, Path], act: str, step_label: str,
        expected_discovery: list[str],
    ) -> None:
        """Backfill guard: discovery-step regression cases должны явно
        объявлять discovery_tools в своих acceptance.

        Если кто-то «почистит» acceptance.json и уберёт это поле —
        тест упадёт с явным указанием на issue #2406.
        """
        sc_path = scenarios[act]
        assert sc_path.exists(), f"scenario not found: {sc_path}"
        sc = json.loads(sc_path.read_text(encoding="utf-8"))
        steps = sc.get("steps", [])
        match = [s for s in steps if s.get("label") == step_label]
        assert match, f"step {step_label} not found in {sc_path}"
        step = match[0]
        acceptance = step.get("acceptance", {})
        actual = acceptance.get("discovery_tools", [])
        assert actual == expected_discovery, (
            f"step {step_label} in {sc_path.name} must declare "
            f"discovery_tools={expected_discovery} for issue #2406 "
            f"regression coverage, got {actual}"
        )


# ── TestE2EScriptContract (sanity — паттерн в harness) ────────────────────


class TestE2EScriptContract:
    """Sanity-check: e2e_voice_test.sh содержит новые паттерны.

    Если кто-то рефакторит harness и уберёт discovery_tools — тест
    упадёт раньше чем сломается acceptance.json контракт.
    """

    def test_script_imports_first_invocation_position(self) -> None:
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert "first_invocation_position" in text, (
            "first_invocation_position не импортируется — issue #2406 "
            "order check удалён из check_acceptance?"
        )

    def test_script_imports_first_voice_cycle_position(self) -> None:
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert "first_voice_cycle_position" in text, (
            "first_voice_cycle_position не импортируется — voice-cycle "
            "marker helper удалён?"
        )

    def test_script_documents_discovery_tools_field(self) -> None:
        """Docstring check_acceptance должен упоминать discovery_tools."""
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert "discovery_tools" in text, (
            "acceptance.json schema doc отсутствует — добавь поле в "
            "check_acceptance docstring (line ~1018)"
        )

    def test_e2e_tool_match_exports_first_invocation_position(self) -> None:
        """e2e_tool_match.py должен экспортировать новый хелпер."""
        text = (SCRIPTS_DIR / "e2e_tool_match.py").read_text(encoding="utf-8")
        assert "first_invocation_position" in text, (
            "first_invocation_position не найден в e2e_tool_match.py"
        )

    def test_e2e_tool_match_exports_first_voice_cycle_position(self) -> None:
        text = (SCRIPTS_DIR / "e2e_tool_match.py").read_text(encoding="utf-8")
        assert "first_voice_cycle_position" in text, (
            "first_voice_cycle_position не найден в e2e_tool_match.py"
        )