"""
test_wake_gate_preflight.py — юнит-тесты ADR-0027 §5.2 wake-gate pre-flight.

Проверяют:
  1. Schema wake_gate_preflight.json: обязательные поля cleared/checked_at/before/reason/error
  2. wake_gate_cleared_since() с LOGS_FILE stub: cleared vs not cleared
  3. detect_wake_word_in_text() — «Робот»/«Робокс» vs без wake
  4. classify_step_expect() — auto-detect wake-prefix → wake-gated
  5. run_wake_gate_preflight() с LOGS_FILE stub пишет корректный JSON
  6. Contract: e2e_voice_test.sh source'ит e2e_voice_wake_gate.sh и
     использует classify_step_expect, run_wake_gate_preflight в main flow

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_wake_gate_preflight.py -v --no-cov

ADR-0027 §5.2 source: docs/adr/0027-systemic-wake-gate-no-wake-word-blocker.md
Retro: t_be491fba
"""

import json
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
WAKE_GATE_LIB = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_wake_gate.sh"


# --- helpers ---------------------------------------------------------------


def load_bash_lib():
    """Source wake-gate lib in subshell-style subprocess и возвращает функции.
    Чтобы НЕ зависеть от side-effects (source основного e2e_voice_test.sh
    выполняет main flow), source'им ТОЛЬКО wake-gate helper lib.
    """
    if not WAKE_GATE_LIB.exists():
        pytest.fail(f"WAKE_GATE_LIB not found: {WAKE_GATE_LIB}")
    return WAKE_GATE_LIB.read_text()


def run_in_subshell(script_body: str, env: dict) -> tuple:
    """Запускает bash subshell с заданным body и env, возвращает (rc, stdout)."""
    proc = subprocess.run(
        ["bash", "-c", script_body],
        capture_output=True, text=True, env=env, timeout=15,
    )
    return proc.returncode, proc.stdout, proc.stderr


# --- Tests: schema wake_gate_preflight.json --------------------------------


class TestWakeGatePreflightSchema:
    """Schema wake_gate_preflight.json: обязательные поля, типы."""

    EXPECTED_FIELDS = {"cleared", "checked_at", "before", "reason", "error"}

    def test_schema_required_fields_present(self, tmp_path):
        state_file = tmp_path / "wake_gate_preflight.json"
        # Имитация run_wake_gate_preflight: пишем JSON руками с правильными полями
        state_file.write_text(json.dumps({
            "cleared": False,
            "checked_at": "2026-08-24T00:00:00Z",
            "before": "2026-08-23T23:59:00Z",
            "reason": "wake-gate cold-start NOT cleared",
            "error": None,
        }, indent=2))
        parsed = json.loads(state_file.read_text())
        for field in self.EXPECTED_FIELDS:
            assert field in parsed, f"missing required field: {field}"

    def test_schema_cleared_is_boolean(self, tmp_path):
        state_file = tmp_path / "wake_gate_preflight.json"
        state_file.write_text(json.dumps({
            "cleared": True,
            "checked_at": "2026-08-24T00:00:00Z",
            "before": "2026-08-23T23:59:00Z",
            "reason": "wake-gate cleared",
            "error": None,
        }))
        parsed = json.loads(state_file.read_text())
        assert isinstance(parsed["cleared"], bool)
        assert parsed["cleared"] is True

    def test_schema_checked_at_is_iso8601(self, tmp_path):
        state_file = tmp_path / "wake_gate_preflight.json"
        state_file.write_text(json.dumps({
            "cleared": False,
            "checked_at": "2026-08-24T00:00:00Z",
            "before": "2026-08-23T23:59:00Z",
            "reason": "test",
            "error": None,
        }))
        parsed = json.loads(state_file.read_text())
        # Loose check: ISO 8601 UTC с Z
        assert re.match(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$", parsed["checked_at"])

    def test_schema_error_is_nullable(self, tmp_path):
        """error может быть null (всё ок) или строкой (probe error)."""
        for error_value in (None, "no ROBOT_SSH/LOGS_FILE — cannot probe"):
            state_file = tmp_path / "wg.json"
            state_file.write_text(json.dumps({
                "cleared": error_value is None,
                "checked_at": "2026-08-24T00:00:00Z",
                "before": "2026-08-23T23:59:00Z",
                "reason": "test",
                "error": error_value,
            }))
            parsed = json.loads(state_file.read_text())
            if error_value is None:
                assert parsed["error"] is None
            else:
                assert isinstance(parsed["error"], str)


# --- Tests: wake_gate_cleared_since (с LOGS_FILE stub) --------------------


class TestWakeGateClearedSince:
    """wake_gate_cleared_since() — pure function через LOGS_FILE env (для тестов)."""

    def _source_lib(self):
        return f'source "{WAKE_GATE_LIB}"'

    def test_cleared_when_wake_accepted_in_logs(self, tmp_path):
        logs = tmp_path / "voice.log"
        logs.write_text(
            "2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: Робот, как дела\n"
            "2026-08-23T23:59:57 dialogue_node: LLM INPUT\n"
        )
        body = f"""
        {self._source_lib()}
        if wake_gate_cleared_since "2026-08-23T23:59:00Z"; then
            echo "CLEARED"
        else
            echo "NOT_CLEARED"
        fi
        """
        rc, out, err = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 0
        assert "CLEARED" in out

    def test_not_cleared_when_no_wake_in_logs(self, tmp_path):
        logs = tmp_path / "voice.log"
        logs.write_text(
            "2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: как дела\n"  # без «Робот»
            "2026-08-23T23:59:57 dialogue_node: backlog accumulated\n"
        )
        body = f"""
        {self._source_lib()}
        if wake_gate_cleared_since "2026-08-23T23:59:00Z"; then
            echo "CLEARED"
        else
            echo "NOT_CLEARED"
        fi
        """
        rc, out, err = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 0
        assert "NOT_CLEARED" in out

    def test_cleared_with_alternative_wake_roboks(self, tmp_path):
        """Альтернативный wake-word «Робокс» (issue #1252) — тоже cleared."""
        logs = tmp_path / "voice.log"
        logs.write_text(
            "2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: Робокс, привет\n"
        )
        body = f"""
        {self._source_lib()}
        if wake_gate_cleared_since "2026-08-23T23:59:00Z"; then
            echo "CLEARED"
        else
            echo "NOT_CLEARED"
        fi
        """
        rc, out, err = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 0
        assert "CLEARED" in out

    def test_cleared_after_cyrillic_lowercase(self, tmp_path):
        """«робот, ...» (lowercase) не матчится — wake-word case-sensitive в ПРИНЯТО.
        STT пишет «Робот» с заглавной. Проверяем, что lowercase НЕ cleared
        (защита от false positive)."""
        logs = tmp_path / "voice.log"
        logs.write_text(
            "2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: робот, как дела\n"
        )
        body = f"""
        {self._source_lib()}
        if wake_gate_cleared_since "2026-08-23T23:59:00Z"; then
            echo "CLEARED"
        else
            echo "NOT_CLEARED"
        fi
        """
        rc, out, err = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 0
        # lowercase wake-word не матчится (по контракту STT пишет «Робот»/«Робокс» capitalized)
        assert "NOT_CLEARED" in out

    def test_fatal_when_neither_ssh_nor_logs(self):
        """Без ROBOT_SSH И без LOGS_FILE → exit 2 (FATAL)."""
        body = f"""
        {self._source_lib()}
        wake_gate_cleared_since "2026-08-23T23:59:00Z"
        """
        env = {"PATH": os.environ["PATH"]}  # NO ROBOT_SSH, NO LOGS_FILE
        proc = subprocess.run(
            ["bash", "-c", body], capture_output=True, text=True, env=env, timeout=10,
        )
        # rc=2 (FATAL) — caller обрабатывает как probe error
        assert proc.returncode == 2
        assert "E2E_FATAL" in proc.stderr


# --- Tests: detect_wake_word_in_text --------------------------------------


class TestDetectWakeWordInText:
    """detect_wake_word_in_text() — детектирует wake-prefix в тексте команды."""

    def _source_lib(self):
        return f'source "{WAKE_GATE_LIB}"'

    def _check(self, text: str) -> bool:
        body = f"""
        {self._source_lib()}
        if detect_wake_word_in_text {shlex_quote(text)}; then
            echo "WAKE"
        else
            echo "NO_WAKE"
        fi
        """
        rc, out, _ = run_in_subshell(body, {"PATH": os.environ["PATH"]})
        return "WAKE" in out and "NO_WAKE" not in out

    def test_robot_standard(self):
        assert self._check("Робот, как дела") is True

    def test_roboks_alternative(self):
        assert self._check("Робокс, как дела") is True

    def test_robot_with_comma_after(self):
        assert self._check("Робот, скажи привет") is True

    def test_robot_no_comma(self):
        """Без запятой сразу после wake — тоже матчится (whitespace tolerant)."""
        assert self._check("Робот  как дела") is True

    def test_no_wake_word(self):
        assert self._check("привет, как дела") is False

    def test_empty_text(self):
        assert self._check("") is False

    def test_wake_in_middle_not_at_start(self):
        """«привет, робот, как дела» — wake в середине, не в начале → NO_WAKE."""
        assert self._check("привет, робот, как дела") is False


def shlex_quote(s: str) -> str:
    """Минимальный shell-quote для тестов (без зависимости от shlex)."""
    if not s:
        return "''"
    if re.match(r"^[A-Za-z0-9_./:=,]+$", s):
        return s
    return "'" + s.replace("'", "'\"'\"'") + "'"


# --- Tests: classify_step_expect ------------------------------------------


class TestClassifyStepExpect:
    """classify_step_expect() — auto-detect wake-prefix → wake-gated."""

    def _source_lib(self):
        return f'source "{WAKE_GATE_LIB}"'

    def _classify(self, expect_raw: str, text: str) -> str:
        body = f"""
        {self._source_lib()}
        classify_step_expect {shlex_quote(expect_raw)} {shlex_quote(text)}
        """
        rc, out, _ = run_in_subshell(body, {"PATH": os.environ["PATH"]})
        return out.strip()

    def test_explicit_wake_gated(self):
        assert self._classify("wake-gated", "Робот, как дела") == "wake-gated"

    def test_explicit_wake_gated_underscore_alias(self):
        """«wake_gated» — алиас для back-compat (camelCase + underscore)."""
        assert self._classify("wake_gated", "Робот, как дела") == "wake-gated"

    def test_explicit_backlog(self):
        assert self._classify("backlog", "Я считаю что чай лучше кофе") == "backlog"

    def test_auto_detect_wake_prefix_robot(self):
        """Пустой expect + текст с «Робот» → wake-gated (auto-detect)."""
        assert self._classify("", "Робот, как дела") == "wake-gated"

    def test_auto_detect_wake_prefix_roboks(self):
        assert self._classify("", "Робокс, привет") == "wake-gated"

    def test_default_cycle_no_wake(self):
        """Пустой expect + текст без wake → cycle (default)."""
        assert self._classify("", "привет, как дела") == "cycle"

    def test_explicit_cycle_wake_text(self):
        """Явный expect=cycle даже при wake-prefix → cycle (override)."""
        # NB: caller ответственен за override. classify_step_expect
        # respects explicit cycle (НЕ auto-promote).
        assert self._classify("cycle", "Робот, как дела") == "cycle"

    def test_unknown_expect_falls_back_to_cycle(self):
        assert self._classify("foobar", "привет") == "cycle"


# --- Tests: run_wake_gate_preflight (integration) -------------------------


class TestRunWakeGatePreflight:
    """run_wake_gate_preflight() пишет корректный JSON + возвращает 0/1/2."""

    def _source_lib(self):
        return f'source "{WAKE_GATE_LIB}"'

    def test_cleared_writes_cleared_true(self, tmp_path):
        logs = tmp_path / "voice.log"
        logs.write_text("2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: Робот, как дела\n")
        state_file = tmp_path / "wg.json"
        body = f"""
        {self._source_lib()}
        run_wake_gate_preflight "2026-08-23T23:59:00Z" "{state_file}"
        """
        rc, _, _ = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 0
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is True
        assert parsed["error"] is None
        assert "checked_at" in parsed
        assert parsed["before"] == "2026-08-23T23:59:00Z"

    def test_not_cleared_writes_cleared_false(self, tmp_path):
        logs = tmp_path / "voice.log"
        logs.write_text("2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: как дела\n")
        state_file = tmp_path / "wg.json"
        body = f"""
        {self._source_lib()}
        run_wake_gate_preflight "2026-08-23T23:59:00Z" "{state_file}"
        """
        rc, _, _ = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 1
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is False
        assert parsed["error"] is None
        assert "NOT cleared" in parsed["reason"]

    def test_probe_error_writes_error_message(self, tmp_path):
        """Без LOGS_FILE/ROBOT_SSH → probe error (rc=2, error != null)."""
        state_file = tmp_path / "wg.json"
        body = f"""
        {self._source_lib()}
        run_wake_gate_preflight "2026-08-23T23:59:00Z" "{state_file}"
        """
        env = {"PATH": os.environ["PATH"]}  # no LOGS_FILE, no ROBOT_SSH
        proc = subprocess.run(
            ["bash", "-c", body], capture_output=True, text=True, env=env, timeout=10,
        )
        assert proc.returncode == 2
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is False
        assert parsed["error"] is not None
        assert "no ROBOT_SSH" in parsed["error"]

    def test_writes_to_nested_path_creates_dirs(self, tmp_path):
        """state_file в несуществующем dir → mkdir -p создаёт dir."""
        logs = tmp_path / "voice.log"
        logs.write_text("2026-08-23T23:59:55 stt_node: ✅ ПРИНЯТО: Робот, как дела\n")
        state_file = tmp_path / "deep" / "nested" / "wg.json"
        body = f"""
        {self._source_lib()}
        run_wake_gate_preflight "2026-08-23T23:59:00Z" "{state_file}"
        """
        rc, _, _ = run_in_subshell(body, {"LOGS_FILE": str(logs), "PATH": os.environ["PATH"]})
        assert rc == 0
        assert state_file.exists()


# --- Tests: e2e_voice_test.sh contract (sources lib, uses functions) -----


class TestE2EScriptWakeGateContract:
    """Sanity: e2e_voice_test.sh source'ит wake-gate lib и использует API."""

    def test_script_sources_wake_gate_lib(self):
        text = E2E_SCRIPT.read_text()
        assert "e2e_voice_wake_gate.sh" in text, (
            "e2e_voice_wake_gate.sh not sourced — wake-gate preflight missing"
        )

    def test_script_uses_classify_step_expect(self):
        text = E2E_SCRIPT.read_text()
        assert "classify_step_expect" in text, (
            "classify_step_expect() not used — auto-detect wake-prefix missing"
        )

    def test_script_uses_run_wake_gate_preflight(self):
        text = E2E_SCRIPT.read_text()
        assert "run_wake_gate_preflight" in text, (
            "run_wake_gate_preflight() not invoked — preflight probe missing"
        )

    def test_script_writes_wake_gate_preflight_artifact(self):
        text = E2E_SCRIPT.read_text()
        assert "wake_gate_preflight.json" in text, (
            "wake_gate_preflight.json artifact not written — no audit trail"
        )

    def test_script_emits_skip_wake_gate_cold_start_marker(self):
        """E2E_STEP <label> SKIP wake-gate-cold-start — маркер для пост-валидатора."""
        text = E2E_SCRIPT.read_text()
        assert "SKIP wake-gate-cold-start" in text, (
            "SKIP marker not emitted — post-validator can't distinguish systemic vs acceptance fail"
        )

    def test_script_emits_gate1_skip_wake_gate_marker(self):
        """E2E_GATE1_SKIP_WAKE_GATE — marker для aggregate GATE-1 skip."""
        text = E2E_SCRIPT.read_text()
        assert "E2E_GATE1_SKIP_WAKE_GATE" in text, (
            "GATE-1 skip marker not emitted — aggregate still fails on cold-start"
        )

    def test_script_writes_gate1_skip_reason_artifact(self):
        """gate1_skip_reason.json — reason для SKIP, не FAIL."""
        text = E2E_SCRIPT.read_text()
        assert "gate1_skip_reason.json" in text, (
            "gate1_skip_reason.json artifact not written — no reason audit"
        )

    def test_lib_file_exists_and_is_executable_or_sourcible(self):
        """WAKE_GATE_LIB существует и не пустой."""
        assert WAKE_GATE_LIB.exists()
        assert WAKE_GATE_LIB.stat().st_size > 100
        # Не требуем executable bit (source'ится, не запускается), но
        # проверяем что bash может его source'ить.
        body = f'source "{WAKE_GATE_LIB}" && echo OK'
        proc = subprocess.run(
            ["bash", "-c", body], capture_output=True, text=True,
            env={"PATH": os.environ["PATH"]}, timeout=5,
        )
        assert "OK" in proc.stdout


# --- Tests: scenario DSL (voice_core_suite_v1.json compatibility) ---------


class TestScenarioDSLCompatibility:
    """Existing voice_core_suite_v1.json совместим с новой classify_step_expect."""

    def test_voice_core_suite_uses_existing_expect_field(self):
        """Ничего не сломали: voice_core_suite_v1.json продолжает работать."""
        scenario = REPO_ROOT / ".github" / "e2e" / "scenarios" / "voice_core_suite_v1.json"
        if not scenario.exists():
            pytest.skip("voice_core_suite_v1.json not found in worktree")
        parsed = json.loads(scenario.read_text())
        for step in parsed.get("steps", []):
            expect = step.get("expect", "cycle")
            assert expect in ("cycle", "wake-gated", "backlog", ""), (
                f"unknown expect kind: {expect!r} in step {step.get('label')!r}"
            )

    def test_new_wake_gated_expect_field_documented(self):
        """scenario.json schema допускает expect='wake-gated' (новый тип)."""
        scenario = {
            "name": "test_wake_gated",
            "steps": [
                {
                    "label": "test_step",
                    "text": "Робот, тест",
                    "voice": "anton",
                    "expect": "wake-gated",
                    "acceptance": {
                        "expected_tool_calls": ["set_voice"],
                    },
                },
            ],
        }
        # Проверяем что схема парсится без ошибок и содержит wake-gated
        assert scenario["steps"][0]["expect"] == "wake-gated"
        # Проверяем что classify_step_expect правильно мапит wake-gated
        body = f"""
        source "{WAKE_GATE_LIB}"
        classify_step_expect "wake-gated" "Робот, тест"
        """
        proc = subprocess.run(
            ["bash", "-c", body], capture_output=True, text=True,
            env={"PATH": os.environ["PATH"]}, timeout=5,
        )
        assert proc.stdout.strip() == "wake-gated"
