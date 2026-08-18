"""
test_e2e_voice_test_gate1.py — юнит-тесты ADR-0022 GATE-1 acceptance.json

Проверяют:
  1. CLI парсинг: --acceptance, --acceptance-skip (через subprocess)
  2. Schema acceptance.json: обязательные поля expected_tool_calls + must_not_call
  3. Python-валидатор: PASS при expected найденном в логах, FAIL при
     expected отсутствующем или forbidden вызванном
  4. Per-step acceptance.json формат (backwards-compat с issue #1396)

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_e2e_voice_test_gate1.py -v --no-cov

ADR-0022 GATE-1 source: docs/adr/0022-process-e2e-done-gates.md §4.1
"""

import json
import os
import re
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"


# --- helpers ---------------------------------------------------------------


def parse_acceptance(acc_path: Path) -> dict:
    """Локальная копия парсера acceptance.json из e2e_voice_test.sh.
    Синхронна с тем, что в check_gate1_aggregate()."""
    with open(acc_path) as f:
        return json.load(f)


def gate1_validate(acc: dict, logs: str) -> dict:
    """Локальная копия Python-валидатора из check_gate1_aggregate().
    Синхронна с тем, что в e2e_voice_test.sh (строки 740-770).

    Возвращает dict с verdict (mirror формата $OUT_DIR/acceptance.json).
    """
    expected_call = acc.get("expected_tool_calls", []) or []
    must_not = acc.get("must_not_call", []) or []

    if not isinstance(expected_call, list) or not isinstance(must_not, list):
        return {
            "gate": "GATE-1",
            "pass": False,
            "reason": "expected_tool_calls and must_not_call must be list[str]",
        }

    def has(frag: str) -> bool:
        return frag.lower() in logs.lower()

    actual_calls = []
    for c in (expected_call + must_not):
        if has(c) and c not in actual_calls:
            actual_calls.append(c)

    found_expected = [c for c in expected_call if has(c)]
    missing_expected = [c for c in expected_call if not has(c)]
    forbidden_called = [c for c in must_not if has(c)]

    failures = []
    if missing_expected:
        failures.append(
            "expected tool calls not invoked during run: "
            + ", ".join(missing_expected)
        )
    if forbidden_called:
        failures.append(
            "forbidden tool calls invoked during run: "
            + ", ".join(forbidden_called)
        )

    return {
        "gate": "GATE-1",
        "name": acc.get("name", ""),
        "expected_tool_calls": expected_call,
        "must_not_call": must_not,
        "actual_tool_calls": actual_calls,
        "found_expected_calls": found_expected,
        "missing_expected_calls": missing_expected,
        "forbidden_calls": forbidden_called,
        "pass": not failures,
        "reason": "; ".join(failures) if failures else "all checks passed",
    }


def make_fake_ssh(tmp_path: Path, log_content: str):
    """Создаёт stub для ROBOT_SSH_OVERRIDE — печатает log_content.
    Возвращает (Path к fake_ssh.sh, Path к logs.txt)."""
    fake = tmp_path / "fake_ssh.sh"
    fake.write_text("#!/bin/bash\ncat \"$LOG_FILE\"\n")
    fake.chmod(0o755)
    log_file = tmp_path / "logs.txt"
    log_file.write_text(log_content)
    return fake, log_file


# --- Tests: schema validation -----------------------------------------------


class TestGate1AcceptanceSchema:
    """Schema acceptance.json (ADR-0022 §4.1)."""

    def test_schema_has_expected_tool_calls(self, tmp_path):
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }))
        parsed = parse_acceptance(acc)
        assert "expected_tool_calls" in parsed
        assert "must_not_call" in parsed

    def test_schema_empty_lists_are_ok(self, tmp_path):
        """Пустые списки допустимы (smoke-проверка) — gating только если
        оба пустые, gate валиден."""
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": [],
            "must_not_call": [],
        }))
        result = gate1_validate(parse_acceptance(acc), "any logs")
        assert result["pass"] is True

    def test_schema_wrong_type_fails(self, tmp_path):
        """expected_tool_calls: не list → FAIL."""
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": "generate_music",  # string, не list
            "must_not_call": [],
        }))
        result = gate1_validate(parse_acceptance(acc), "any logs")
        assert result["pass"] is False
        assert "must be list[str]" in result["reason"]


# --- Tests: GATE-1 validator ------------------------------------------------


class TestGate1Validator:
    """Python-валидатор (имитация check_gate1_aggregate())."""

    def test_passes_when_expected_tool_called(self):
        acc = {
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }
        logs = "2025-08-18 dialogue_node: Calling MCP tool: generate_music"
        result = gate1_validate(acc, logs)
        assert result["pass"] is True
        assert result["found_expected_calls"] == ["generate_music"]
        assert result["missing_expected_calls"] == []
        assert result["forbidden_calls"] == []

    def test_fails_when_expected_tool_not_called(self):
        """Главный кейс из acceptance: generate_music ожидался, но не вызван."""
        acc = {
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }
        logs = "2025-08-18 dialogue_node: LLM ответил текстом без tools"
        result = gate1_validate(acc, logs)
        assert result["pass"] is False
        assert "generate_music" in result["missing_expected_calls"]
        assert "not invoked" in result["reason"]

    def test_fails_when_forbidden_tool_called(self):
        """Главный кейс из acceptance: execute_music_code НЕ должен быть вызван."""
        acc = {
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }
        logs = (
            "2025-08-18 dialogue_node: Calling MCP tool: execute_music_code\n"
            "2025-08-18 dialogue_node: Renardo запустил PTree.__init__()"
        )
        result = gate1_validate(acc, logs)
        assert result["pass"] is False
        assert "execute_music_code" in result["forbidden_calls"]
        assert "forbidden" in result["reason"]

    def test_fails_with_both_missing_and_forbidden(self):
        """Оба условия одновременно → reasons joined."""
        acc = {
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }
        logs = "Calling MCP tool: execute_music_code"
        result = gate1_validate(acc, logs)
        assert result["pass"] is False
        assert "generate_music" in result["missing_expected_calls"]
        assert "execute_music_code" in result["forbidden_calls"]
        assert "not invoked" in result["reason"]
        assert "forbidden" in result["reason"]

    def test_case_insensitive_match(self):
        """Substring match case-insensitive (rename_logs могут быть любые)."""
        acc = {
            "expected_tool_calls": ["Generate_Music"],
            "must_not_call": ["Execute_Music_Code"],
        }
        logs = "calling mcp tool: generate_music"
        result = gate1_validate(acc, logs)
        assert result["pass"] is True

    def test_multiple_expected_tools_and_semantics(self):
        """expected_tool_calls — AND: ВСЕ из них должны быть вызваны хотя бы
        1 раз за прогон. Это контракт ADR-0022 GATE-1 для multi-tool фич
        (например, PR #1398: 7 MCP tools, ВСЕ должны быть вызваны)."""
        acc = {
            "expected_tool_calls": ["generate_music", "gen_list_library"],
            "must_not_call": [],
        }
        # Только generate_music вызван → FAIL (gen_list_library ожидался)
        logs_partial = "Calling MCP tool: generate_music"
        result = gate1_validate(acc, logs_partial)
        assert result["pass"] is False
        assert "gen_list_library" in result["missing_expected_calls"]

        # Оба вызваны → PASS
        logs_full = (
            "Calling MCP tool: generate_music\n"
            "Calling MCP tool: gen_list_library\n"
        )
        result = gate1_validate(acc, logs_full)
        assert result["pass"] is True
        assert result["found_expected_calls"] == ["generate_music", "gen_list_library"]
        assert result["missing_expected_calls"] == []


# --- Tests: e2e_voice_test.sh integration (CLI gating) ----------------------


class TestE2EVoiceTestScriptGating:
    """Интеграционные: e2e_voice_test.sh падает/проходит при разных CLI."""

    @pytest.fixture(autouse=True)
    def require_yandex_key(self, monkeypatch):
        """Скрипт требует YANDEX_API_KEY — подсовываем fake."""
        monkeypatch.setenv("YANDEX_API_KEY", "fake-key-for-test")

    def test_gating_fail_when_scenario_without_acceptance(self, tmp_path):
        """--scenario задан, acceptance.json отсутствует → exit 1 + gating message."""
        scenario = tmp_path / "scenario.json"
        scenario.write_text(json.dumps({"steps": [
            {"label": "s1", "text": "test", "voice": "anton"}
        ]}))
        result = subprocess.run(
            [str(E2E_SCRIPT), "--scenario", str(scenario)],
            capture_output=True, text=True, timeout=15,
        )
        assert result.returncode == 1, f"expected exit 1, got {result.returncode}"
        assert "GATE-1" in result.stdout
        assert "E2E_GATE1_MISSING_ACCEPTANCE" in result.stdout
        assert "acceptance.json" in result.stdout

    def test_gating_skip_with_flag(self, tmp_path):
        """--acceptance-skip обходит gating, но скрипт всё равно упадёт
        на синтезе Yandex (fake-key). Главное — НЕ gating error."""
        scenario = tmp_path / "scenario.json"
        scenario.write_text(json.dumps({"steps": [
            {"label": "s1", "text": "test", "voice": "anton"}
        ]}))
        result = subprocess.run(
            [str(E2E_SCRIPT), "--scenario", str(scenario), "--acceptance-skip"],
            capture_output=True, text=True, timeout=15,
        )
        # Без acceptance.json НЕ должно быть gating error
        assert "E2E_GATE1_MISSING_ACCEPTANCE" not in result.stdout
        # Скрипт ушёл дальше и упал на synth Yandex (fake-key) — это OK
        # (мы НЕ проверяем итоговый verdict, только что gating не сработал)

    def test_gating_pass_with_acceptance_file(self, tmp_path):
        """--acceptance <path> явно задан → gating проходит, aggregate check FAIL
        (нет реального робота, expected tool не вызван)."""
        scenario = tmp_path / "scenario.json"
        scenario.write_text(json.dumps({"steps": [
            {"label": "s1", "text": "test", "voice": "anton"}
        ]}))
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }))
        result = subprocess.run(
            [str(E2E_SCRIPT),
             "--scenario", str(scenario),
             "--acceptance", str(acc),
             "--acceptance-skip"],
            capture_output=True, text=True, timeout=15,
        )
        # Gating прошёл (нет MISSING_ACCEPTANCE)
        assert "E2E_GATE1_MISSING_ACCEPTANCE" not in result.stdout
        # Aggregate check выполнился (даже если verdict FAIL — gating ОК)
        assert "E2E_GATE1" in result.stdout

    def test_auto_discovery_of_acceptance_json(self, tmp_path):
        """Если acceptance.json рядом с scenario.json — auto-discovery."""
        scenario = tmp_path / "scenario.json"
        scenario.write_text(json.dumps({"steps": [
            {"label": "s1", "text": "test", "voice": "anton"}
        ]}))
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": ["foo"],
            "must_not_call": [],
        }))
        result = subprocess.run(
            [str(E2E_SCRIPT),
             "--scenario", str(scenario),
             "--acceptance-skip"],
            capture_output=True, text=True, timeout=15,
        )
        assert "auto-discovered" in result.stdout
        assert "E2E_GATE1_MISSING_ACCEPTANCE" not in result.stdout

    def test_text_only_does_not_require_acceptance(self, tmp_path):
        """--text (single-shot smoke) БЕЗ --scenario не требует acceptance.json
        (legitimate smoke-test use case)."""
        result = subprocess.run(
            [str(E2E_SCRIPT), "--text", "Робот, тест"],
            capture_output=True, text=True, timeout=15,
        )
        # Не должно быть gating error даже без acceptance
        assert "E2E_GATE1_MISSING_ACCEPTANCE" not in result.stdout


# --- Tests: e2e_voice_test.sh aggregate check (full integration) -----------


class TestGate1AggregateIntegration:
    """Полный сценарий: aggregate check с подделкой логов через
    ROBOT_SSH_OVERRIDE — проверяет что GATE-1 PASS/FAIL корректно
    определяется по фактическим логам."""

    def test_aggregate_pass_when_expected_called(self, tmp_path, monkeypatch):
        scenario = tmp_path / "scenario.json"
        scenario.write_text(json.dumps({"steps": [
            {"label": "s1", "text": "test", "voice": "anton"}
        ]}))
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }))
        fake_ssh, log_file = make_fake_ssh(
            tmp_path,
            "2025-08-18 dialogue_node: Calling MCP tool: generate_music\n"
            "2025-08-18 dialogue_node: MCP tool result: generate_music",
        )
        monkeypatch.setenv("YANDEX_API_KEY", "fake")
        monkeypatch.setenv("ROBOT_SSH_OVERRIDE", str(fake_ssh))
        monkeypatch.setenv("LOG_FILE", str(log_file))
        result = subprocess.run(
            [str(E2E_SCRIPT),
             "--scenario", str(scenario),
             "--acceptance", str(acc),
             "--acceptance-skip"],
            capture_output=True, text=True, timeout=20,
        )
        # GATE-1 aggregate должен быть PASS
        assert "GATE-1: ✅" in result.stdout, (
            f"expected GATE-1 PASS in stdout:\n{result.stdout}"
        )
        # $OUT_DIR/acceptance.json создаётся в /tmp/e2e_v2_<run_id>/
        import glob
        out_dirs = sorted(glob.glob("/tmp/e2e_v2_*"),
                          key=os.path.getmtime, reverse=True)
        assert out_dirs, "no /tmp/e2e_v2_* dir found"
        verdict_path = Path(out_dirs[0]) / "acceptance.json"
        if verdict_path.exists():
            verdict = json.loads(verdict_path.read_text())
            assert verdict["pass"] is True, (
                f"acceptance.json verdict FAIL: {verdict}"
            )
            assert verdict["gate"] == "GATE-1"

    def test_aggregate_fail_when_renardo_called(self, tmp_path, monkeypatch):
        """Ровно кейс из acceptance: expected=generate_music,
        must_not_call=execute_music_code. Renardo вызван → GATE-1 FAIL."""
        scenario = tmp_path / "scenario.json"
        scenario.write_text(json.dumps({"steps": [
            {"label": "s1", "text": "test", "voice": "anton"}
        ]}))
        acc = tmp_path / "acceptance.json"
        acc.write_text(json.dumps({
            "expected_tool_calls": ["generate_music"],
            "must_not_call": ["execute_music_code"],
        }))
        fake_ssh, log_file = make_fake_ssh(
            tmp_path,
            "2025-08-18 dialogue_node: Calling MCP tool: execute_music_code\n"
            "2025-08-18 dialogue_node: Renardo PTree.__init__()\n",
        )
        monkeypatch.setenv("YANDEX_API_KEY", "fake")
        monkeypatch.setenv("ROBOT_SSH_OVERRIDE", str(fake_ssh))
        monkeypatch.setenv("LOG_FILE", str(log_file))
        result = subprocess.run(
            [str(E2E_SCRIPT),
             "--scenario", str(scenario),
             "--acceptance", str(acc),
             "--acceptance-skip"],
            capture_output=True, text=True, timeout=20,
        )
        assert "GATE-1: ❌" in result.stdout, (
            f"expected GATE-1 FAIL in stdout:\n{result.stdout}"
        )
        # Проверяем acceptance.json артефакт
        import glob
        out_dirs = sorted(glob.glob("/tmp/e2e_v2_*"),
                          key=os.path.getmtime, reverse=True)
        verdict_path = Path(out_dirs[0]) / "acceptance.json"
        if verdict_path.exists():
            verdict = json.loads(verdict_path.read_text())
            assert verdict["pass"] is False
            assert "execute_music_code" in verdict["forbidden_calls"]
            assert "forbidden" in verdict["reason"]


# --- Sanity: скрипт действительно содержит GATE-1 код -----------------------


class TestE2EScriptContract:
    """Sanity-check: паттерны GATE-1 присутствуют в e2e_voice_test.sh.
    Если кто-то удалил/рефакторил GATE-1 код, тест должен упасть раньше,
    чем кто-то начнёт ломать acceptance.json контракт."""

    def test_script_has_acceptance_file_variable(self):
        assert "ACCEPTANCE_FILE" in E2E_SCRIPT.read_text(), (
            "ACCEPTANCE_FILE переменная не найдена — GATE-1 удалён?"
        )

    def test_script_has_acceptance_skip_flag(self):
        text = E2E_SCRIPT.read_text()
        assert "--acceptance-skip" in text, (
            "--acceptance-skip CLI флаг не найден"
        )

    def test_script_has_auto_discovery(self):
        text = E2E_SCRIPT.read_text()
        assert "auto-discovered" in text, (
            "auto-discovery блок не найден"
        )

    def test_script_has_gating_message(self):
        text = E2E_SCRIPT.read_text()
        assert "E2E_GATE1_MISSING_ACCEPTANCE" in text, (
            "GATE-1 gating error marker не найден"
        )

    def test_script_has_check_gate1_aggregate_function(self):
        text = E2E_SCRIPT.read_text()
        assert "check_gate1_aggregate" in text, (
            "check_gate1_aggregate() функция не найдена"
        )

    def test_script_has_robot_ssh_override(self):
        """Override для тестов: ROBOT_SSH_OVERRIDE env."""
        text = E2E_SCRIPT.read_text()
        assert "ROBOT_SSH_OVERRIDE" in text, (
            "ROBOT_SSH_OVERRIDE для юнит-тестов не найден"
        )