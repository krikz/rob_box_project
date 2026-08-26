"""
test_noisy_gate_preflight.py — юнит-тесты ADR-0032 noisy-room pre-flight.

Проверяют:
  1. Schema noisy_preflight.json: cleared/checked_at/window_s/rms_avg_dbfs/
     rms_threshold_dbfs/reason/error/issue_ref
  2. compute_avg_rms_dbfs() — пустой stdin → "-inf", среднее значение
  3. busy_recent() — stub logs busy vs quiet → правильный return code
  4. noisy_preflight() — stub logs → cleared/not-cleared/probe-error,
     пишет корректный JSON
  5. mark_fail_kind с noisy_preflight приоритет feature > noisy > llm > synth
  6. Contract: e2e_voice_test.sh source'ит e2e_voice_noisy_gate.sh и
     вызывает noisy_preflight в main flow

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_noisy_gate_preflight.py -v --no-cov

ADR-0032 source: docs/adr/0032-noisy-preflight-gate.md
Issue: #1668
Retro: t_6e587508 (диагностика), t_67394082 (этот фикс)
"""

import json
import os
import re
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
NOISY_GATE_LIB = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_noisy_gate.sh"
WAKE_GATE_LIB = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_wake_gate.sh"
LIB_SH = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_lib.sh"


# --- helpers ---------------------------------------------------------------


def run_in_subshell(script_body: str, env: dict) -> tuple:
    """Запускает bash subshell с body и env. Возвращает (rc, stdout, stderr)."""
    proc = subprocess.run(
        ["bash", "-c", script_body],
        capture_output=True, text=True, env=env, timeout=15,
    )
    return proc.returncode, proc.stdout, proc.stderr


def source_env() -> dict:
    """Базовый env для subshell — QUIET=1 (подавляет progress logs)."""
    env = os.environ.copy()
    env["QUIET"] = "1"
    return env


# --- Tests: schema noisy_preflight.json ------------------------------------


class TestNoisyPreflightSchema:
    """Schema noisy_preflight.json: обязательные поля и типы."""

    EXPECTED_FIELDS = {
        "cleared", "checked_at", "window_s", "rms_avg_dbfs",
        "rms_threshold_dbfs", "reason", "error", "issue_ref",
    }

    def test_schema_required_fields_present(self, tmp_path):
        """Минимальный корректный JSON проходит schema-check."""
        state_file = tmp_path / "noisy_preflight.json"
        state_file.write_text(json.dumps({
            "cleared": False,
            "checked_at": "2026-08-26T10:00:00Z",
            "window_s": 30,
            "rms_avg_dbfs": "-38.50",
            "rms_threshold_dbfs": -45,
            "reason": "robot too noisy (rms_dbfs=-38.50 > -45 threshold for 30s)",
            "error": None,
            "issue_ref": "#1668",
        }, indent=2))
        parsed = json.loads(state_file.read_text())
        for field in self.EXPECTED_FIELDS:
            assert field in parsed, f"missing required field: {field}"

    def test_schema_cleared_is_boolean(self, tmp_path):
        state_file = tmp_path / "noisy_preflight.json"
        state_file.write_text(json.dumps({
            "cleared": True,
            "checked_at": "2026-08-26T10:00:00Z",
            "window_s": 30,
            "rms_avg_dbfs": "-55.00",
            "rms_threshold_dbfs": -45,
            "reason": "robot quiet",
            "error": None,
            "issue_ref": "#1668",
        }, indent=2))
        parsed = json.loads(state_file.read_text())
        assert isinstance(parsed["cleared"], bool)

    def test_schema_window_s_is_int(self, tmp_path):
        state_file = tmp_path / "noisy_preflight.json"
        state_file.write_text(json.dumps({
            "cleared": False,
            "checked_at": "2026-08-26T10:00:00Z",
            "window_s": 30,
            "rms_avg_dbfs": "-38.50",
            "rms_threshold_dbfs": -45,
            "reason": "robot too noisy",
            "error": None,
            "issue_ref": "#1668",
        }, indent=2))
        parsed = json.loads(state_file.read_text())
        assert isinstance(parsed["window_s"], int)

    def test_schema_rms_avg_dbfs_can_be_negative_inf_sentinel(self, tmp_path):
        """Пустой logs → rms_avg_dbfs='-inf' (sentinel)."""
        state_file = tmp_path / "noisy_preflight.json"
        state_file.write_text(json.dumps({
            "cleared": True,  # если busy_recent тоже quiet — cleared даже без RMS
            "checked_at": "2026-08-26T10:00:00Z",
            "window_s": 30,
            "rms_avg_dbfs": "-inf",
            "rms_threshold_dbfs": -45,
            "reason": "robot quiet (rms_avg=-inf dBFS <= -45; busy=no)",
            "error": None,
            "issue_ref": "#1668",
        }, indent=2))
        parsed = json.loads(state_file.read_text())
        assert parsed["rms_avg_dbfs"] == "-inf"


# --- Tests: compute_avg_rms_dbfs() -----------------------------------------


class TestComputeAvgRmsDbfs:
    """Pure compute: stdin → stdout float."""

    def test_empty_stdin_returns_inf(self):
        rc, out, _ = run_in_subshell(
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'echo "" | compute_avg_rms_dbfs',
            source_env(),
        )
        assert rc == 0
        assert out.strip() == "-inf"

    def test_single_value(self):
        rc, out, _ = run_in_subshell(
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'LC_ALL=C printf "%d %.2f\\n" 100 -42.50 | compute_avg_rms_dbfs',
            source_env(),
        )
        assert rc == 0
        assert out.strip() == "-42.50"

    def test_multiple_values_arithmetic_mean(self):
        rc, out, _ = run_in_subshell(
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'LC_ALL=C printf "%d %.2f\\n%d %.2f\\n%d %.2f\\n" 100 -40.00 200 -50.00 300 -45.00 | compute_avg_rms_dbfs',
            source_env(),
        )
        assert rc == 0
        # (-40 + -50 + -45) / 3 = -45.00
        assert out.strip() == "-45.00"

    def test_ignores_malformed_lines(self):
        """Невалидные строки игнорируются (не падаем)."""
        rc, out, _ = run_in_subshell(
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'LC_ALL=C printf "garbage line\\n%d %.2f\\nmore garbage\\n" 100 -42.00 | compute_avg_rms_dbfs',
            source_env(),
        )
        assert rc == 0
        assert out.strip() == "-42.00"


# --- Tests: busy_recent() --------------------------------------------------


class TestBusyRecent:
    """Read-only probe: docker logs → busy/quiet verdict."""

    def test_quiet_logs_return_1(self, tmp_path):
        """Пустой / тихий лог → quiet (rc=1)."""
        log_file = tmp_path / "quiet.log"
        log_file.write_text("")  # совсем пустой
        env = source_env()
        env["LOGS_FILE"] = str(log_file)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'busy_recent 30\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=1" in out  # quiet

    def test_noisy_logs_with_rms_bursts_return_0(self, tmp_path):
        """Лог с >10 audio_rms_dbfs событий за 30s → busy (rc=0)."""
        log_file = tmp_path / "noisy.log"
        # Генерим 20 событий за 30s (>10/мин порог).
        lines = []
        for i in range(20):
            lines.append(
                f"2026-08-26T10:00:{i:02d}Z 📊 [issue 1477] "
                f"audio_rms_dbfs=-38.{i % 10} peak_dbfs=-4.{i % 10} "
                f"duration=1.20s samples=19200"
            )
        log_file.write_text("\n".join(lines))
        env = source_env()
        env["LOGS_FILE"] = str(log_file)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'busy_recent 30\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=0" in out  # busy

    def test_tts_active_returns_busy(self, tmp_path):
        """Активный TTS → busy (rc=0), даже если RMS мало."""
        log_file = tmp_path / "tts_active.log"
        log_file.write_text(
            "2026-08-26T10:00:00Z 🎵 Синтез через Silero v5: «Привет, мир»\n"
            "2026-08-26T10:00:01Z 🎵 TTS finished (2.10s)\n"
        )
        env = source_env()
        env["LOGS_FILE"] = str(log_file)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'busy_recent 30\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=0" in out  # busy (TTS активен)

    def test_no_ssh_no_logs_returns_2(self):
        """Нет ни ROBOT_SSH, ни LOGS_FILE → probe error (rc=2)."""
        env = source_env()
        env.pop("ROBOT_SSH", None)
        env.pop("LOGS_FILE", None)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            'busy_recent 30\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=2" in out  # probe error


# --- Tests: noisy_preflight() ----------------------------------------------


class TestNoisyPreflight:
    """Orchestrator: combined busy + RMS verdict + JSON write."""

    def test_quiet_room_returns_cleared(self, tmp_path):
        """Тихий лог (мало событий, RMS очень низкий) → cleared (rc=0)."""
        log_file = tmp_path / "quiet.log"
        log_file.write_text("")  # пустой
        state_file = tmp_path / "noisy_preflight.json"
        env = source_env()
        env["LOGS_FILE"] = str(log_file)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            f'noisy_preflight 30 "{state_file}" -45\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=0" in out  # cleared
        # JSON должен иметь cleared=true.
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is True
        assert parsed["rms_avg_dbfs"] == "-inf"
        assert "quiet" in parsed["reason"]

    def test_noisy_room_returns_not_cleared(self, tmp_path):
        """Много RMS-событий (>10/мин) → not cleared (rc=1)."""
        log_file = tmp_path / "noisy.log"
        lines = []
        for i in range(20):
            lines.append(
                f"2026-08-26T10:00:{i:02d}Z 📊 [issue 1477] "
                f"audio_rms_dbfs=-38.{i % 10} peak_dbfs=-4.{i % 10} "
                f"duration=1.20s samples=19200"
            )
        log_file.write_text("\n".join(lines))
        state_file = tmp_path / "noisy_preflight.json"
        env = source_env()
        env["LOGS_FILE"] = str(log_file)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            f'noisy_preflight 30 "{state_file}" -45\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=1" in out  # not cleared
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is False
        assert "busy" in parsed["reason"]

    def test_high_rms_returns_not_cleared(self, tmp_path):
        """Несколько событий с RMS>-45 → robot too noisy (rc=1)."""
        log_file = tmp_path / "highrms.log"
        # 3 события с RMS=-38 (выше threshold -45), но мало чтобы busy_recent не сработал (<10/мин).
        # 3 события за 30s = 6/мин < 10 → busy_recent=quiet,
        # но RMS avg = -38 > -45 → robot too noisy.
        log_file.write_text(
            "2026-08-26T10:00:00Z 📊 [issue 1477] audio_rms_dbfs=-38.00 peak_dbfs=-4.00 duration=1.20s samples=19200\n"
            "2026-08-26T10:00:10Z 📊 [issue 1477] audio_rms_dbfs=-38.00 peak_dbfs=-4.00 duration=1.20s samples=19200\n"
            "2026-08-26T10:00:20Z 📊 [issue 1477] audio_rms_dbfs=-38.00 peak_dbfs=-4.00 duration=1.20s samples=19200\n"
        )
        state_file = tmp_path / "noisy_preflight.json"
        env = source_env()
        env["LOGS_FILE"] = str(log_file)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            f'noisy_preflight 30 "{state_file}" -45\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=1" in out  # not cleared
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is False
        assert "too noisy" in parsed["reason"]
        assert float(parsed["rms_avg_dbfs"]) > -45

    def test_no_ssh_returns_probe_error(self, tmp_path):
        """Нет доступа к docker logs → probe error (rc=2)."""
        state_file = tmp_path / "noisy_preflight.json"
        env = source_env()
        env.pop("ROBOT_SSH", None)
        env.pop("LOGS_FILE", None)
        body = (
            'source "' + str(NOISY_GATE_LIB) + '"\n'
            f'noisy_preflight 30 "{state_file}" -45\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "rc=2" in out  # probe error
        parsed = json.loads(state_file.read_text())
        assert parsed["cleared"] is False
        assert parsed["error"] is not None


# --- Tests: e2e_voice_test.sh contract ------------------------------------


class TestE2EContract:
    """e2e_voice_test.sh source'ит noisy lib и использует noisy_preflight."""

    def test_e2e_test_sources_noisy_gate(self):
        if not E2E_SCRIPT.exists():
            pytest.fail(f"E2E_SCRIPT not found: {E2E_SCRIPT}")
        content = E2E_SCRIPT.read_text()
        assert "e2e_voice_noisy_gate.sh" in content, \
            "e2e_voice_test.sh должен source'ить e2e_voice_noisy_gate.sh"

    def test_e2e_test_calls_noisy_preflight(self):
        content = E2E_SCRIPT.read_text()
        assert "noisy_preflight" in content, \
            "e2e_voice_test.sh должен вызывать noisy_preflight"

    def test_e2e_test_has_ignore_noisy_preflight_flag(self):
        content = E2E_SCRIPT.read_text()
        assert "--ignore-noisy-preflight" in content, \
            "должен быть CLI-флаг --ignore-noisy-preflight для bypass"

    def test_e2e_test_has_noisy_threshold_flag(self):
        content = E2E_SCRIPT.read_text()
        assert "--noisy-threshold-dbfs" in content, \
            "должен быть CLI-флаг --noisy-threshold-dbfs для тонкой настройки"

    def test_e2e_test_exit_code_7_for_noisy(self):
        content = E2E_SCRIPT.read_text()
        # Должен быть exit 7 при noisy-preflight fail (отдельный от обычного FAIL=1).
        assert re.search(r"exit\s+7", content), \
            "должен быть exit 7 при noisy-preflight fail"

    def test_e2e_test_emits_e2e_noisy_preflight_marker(self):
        content = E2E_SCRIPT.read_text()
        assert "E2E_NOISY_PREFLIGHT" in content, \
            "должен emit'ить маркер E2E_NOISY_PREFLIGHT для detect_fail_kind"

    def test_mark_fail_kind_has_noisy_preflight(self):
        content = E2E_SCRIPT.read_text()
        # mark_fail_kind должен принимать noisy_preflight как приоритет.
        m = re.search(r"mark_fail_kind\(\)\s*\{[^}]*noisy_preflight[^}]*\}", content, re.DOTALL)
        assert m, "mark_fail_kind должен обрабатывать noisy_preflight kind"


# --- Tests: detect_fail_kind integration ----------------------------------


class TestAgentFlowIntegration:
    """agent-flow-e2e-process.sh::detect_fail_kind ловит E2E_NOISY_PREFLIGHT."""

    AGENT_FLOW_SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"

    def test_detect_fail_kind_recognizes_noisy_preflight_marker(self, tmp_path):
        """Если в артефактах есть E2E_NOISY_PREFLIGHT — classify as infra."""
        if not self.AGENT_FLOW_SCRIPT.exists():
            pytest.fail(f"agent-flow script not found: {self.AGENT_FLOW_SCRIPT}")
        # Создаём stub artifact dir с маркером.
        artifact_dir = tmp_path / "artifacts"
        artifact_dir.mkdir()
        (artifact_dir / "voice_e2e_test.log").write_text(
            "[10:00:30] E2E_NOISY_PREFLIGHT: robot busy (rms_per_min>=10)\n"
            "[10:00:31] E2E_VERDICT FAIL\n"
        )
        # Source'им ТОЛЬКО detect_fail_kind (без main flow). Это требует
        # GH_REPO и gh auth — мокаем через stub env.
        content = self.AGENT_FLOW_SCRIPT.read_text()
        # Извлекаем только функцию detect_fail_kind (через sed-range hack).
        # Простой путь — source'ить весь скрипт под stub env и проверить вывод.
        env = source_env()
        env["GH_REPO"] = "krikz/rob_box_project"
        # Skip early steps via stub: LOCK_FILE writeable, no flock contention.
        env["AGENT_FLOW_DRY_RUN"] = "1"
        # Не запускаем main flow — вытаскиваем detect_fail_kind через
        # grep -A100 detect_fail_kind и eval в subshell.
        body = (
            f'AGENT_FLOW_SCRIPT="{self.AGENT_FLOW_SCRIPT}"\n'
            f'ARTIFACT_DIR="{artifact_dir}"\n'
            # Извлекаем тело detect_fail_kind (sed от строки "^detect_fail_kind" до следующей "^}").
            'eval "$(sed -n "/^detect_fail_kind()/,/^}$/p" "$AGENT_FLOW_SCRIPT")"\n'
            'detect_fail_kind "$ARTIFACT_DIR" ""\n'
            'echo "rc=$?"'
        )
        rc, out, _ = run_in_subshell(body, env)
        # Должно вывести "infra" (НЕ feature).
        assert "infra" in out, f"detect_fail_kind should classify noisy-preflight as infra, got: {out!r}"

    def test_noisy_streak_state_machine(self, tmp_path):
        """detect_noisy_streak корректно отслеживает streak count."""
        if not self.AGENT_FLOW_SCRIPT.exists():
            pytest.fail(f"agent-flow script not found: {self.AGENT_FLOW_SCRIPT}")
        state_file = tmp_path / "noisy-streak.state"
        env = source_env()
        env["NOISY_STREAK_STATE_FILE"] = str(state_file)
        env["NOISY_STREAK_THRESHOLD"] = "3"
        env["NOISY_STREAK_PAUSE_MIN"] = "30"
        # Source'им detect_noisy_streak + record_noisy_fail.
        body = (
            f'AGENT_FLOW_SCRIPT="{self.AGENT_FLOW_SCRIPT}"\n'
            'eval "$(sed -n "/^detect_noisy_streak()/,/^}$/p" "$AGENT_FLOW_SCRIPT")"\n'
            'eval "$(sed -n "/^record_noisy_fail()/,/^}$/p" "$AGENT_FLOW_SCRIPT")"\n'
            # Без state file → active.
            'state1=$(detect_noisy_streak)\n'
            'echo "state1=$state1"\n'
            # 2 noise fails → всё ещё active.
            'record_noisy_fail\n'
            'record_noisy_fail\n'
            'state2=$(detect_noisy_streak)\n'
            'echo "state2=$state2"\n'
            # 3-й fail → threshold достигнут → paused.
            'record_noisy_fail\n'
            'state3=$(detect_noisy_streak)\n'
            'echo "state3=$state3"\n'
        )
        rc, out, _ = run_in_subshell(body, env)
        assert rc == 0
        assert "state1=active" in out, f"expected active at start, got: {out!r}"
        assert "state2=active" in out, f"expected active after 2 fails, got: {out!r}"
        assert "state3=paused" in out, f"expected paused at threshold, got: {out!r}"