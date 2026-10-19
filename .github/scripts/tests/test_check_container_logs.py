"""Тесты .github/scripts/check_container_logs.py (шаг «[<Pi>] Check Container Logs»).

SSH подменяется функцией-раннером: тест видит, какие команды уходят на Pi, и
отдаёт заранее заданные «логи». Фильтры critical/warning — настоящие, из
deployment_issue_dedup.py (тот же модуль, что использует скрипт).
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "check_container_logs.py"
_spec = importlib.util.spec_from_file_location("check_container_logs", SCRIPT)
MODULE = importlib.util.module_from_spec(_spec)
assert _spec is not None and _spec.loader is not None
_spec.loader.exec_module(MODULE)


class FakePi:
    def __init__(self, containers: dict[str, str], ps_rc: int = 0, log_rc: int = 0):
        self.containers = containers
        self.ps_rc = ps_rc
        self.log_rc = log_rc
        self.commands: list[str] = []

    def __call__(self, command: str) -> tuple[int, str]:
        self.commands.append(command)
        if "docker compose ps" in command:
            # compose v2 печатает JSON-lines
            return self.ps_rc, "\n".join(json.dumps({"Name": n, "State": "running"}) for n in self.containers)
        for name, logs in self.containers.items():
            if f" {name} " in command + " ":
                return self.log_rc, logs
        return 1, ""


def run(containers, scope="sound", **kw):
    pi = FakePi(containers, **kw)
    result = MODULE.check_logs(pi, scope=scope, environment="staging", compose_dir="~/rob_box_project/docker/" + scope)
    return pi, result


def test_clean_logs_give_no_findings():
    pi, result = run({"voice-assistant": "[INFO] stt_node ready\n[INFO] tts_node ready"})
    assert result["critical"] == 0 and result["warning"] == 0
    assert result["findings"] == [] and result["report"] == []
    assert any(cmd.startswith("cd ~/rob_box_project/docker/sound && docker compose ps") for cmd in pi.commands)


def test_critical_line_is_reported_with_init_context_and_finding_schema():
    logs = "boot line 1\n[ERROR] scsynth crashed: segfault in audio thread\nlast line"
    _, result = run({"supercollider": logs})
    assert result["critical"] == 1
    finding = result["findings"][0]
    assert finding == {
        "environment": "staging",
        "scope": "sound",
        "container": "supercollider",
        "kind": "critical_log",
        "severity": "critical",
        "summary": "Critical log pattern detected",
        "raw_text": "[ERROR] scsynth crashed: segfault in audio thread",
    }
    report = "\n".join(result["report"])
    assert "=== supercollider CRITICAL ERRORS ===" in report
    assert "=== supercollider INITIALIZATION CONTEXT ===" in report


def test_warning_counted_separately_and_scope_exclusions_apply():
    hp = "[WARN] HPFONOFF: write_parameter вернул False (устройство занято?). Используется дефолт firmware."
    _, sound = run({"voice-assistant": hp}, scope="sound")
    assert sound["warning"] == 0  # правило живёт в scope sound (ADR-0159)

    _, vision = run({"voice-assistant": hp}, scope="vision")
    assert vision["warning"] == 1 and vision["findings"][0]["kind"] == "warning_log"


def test_logs_are_fetched_head_and_tail_per_container():
    pi, _ = run({"a": "ok", "b": "ok"})
    logs_cmds = [c for c in pi.commands if "docker logs" in c]
    assert sum("--tail 10000" in c and "head -50" in c for c in logs_cmds) == 2
    assert sum("--tail 150" in c for c in logs_cmds) == 2


def test_failed_ps_does_not_crash_and_is_not_silent(capsys):
    _, result = run({}, ps_rc=255)
    assert result["critical"] == 0
    assert "::warning" in capsys.readouterr().out


def test_unavailable_logs_warn_but_do_not_abort_other_containers(capsys):
    pi = FakePi({"a": "[ERROR] fatal: boom crashed", "b": "ok"}, log_rc=1)
    result = MODULE.check_logs(pi, scope="main", environment="staging", compose_dir="x")
    out = capsys.readouterr().out
    assert out.count("::warning title=Logs unavailable::") == 2
    assert result["critical"] == 1  # лог прочитан, несмотря на ненулевой код


def test_main_writes_outputs_findings_and_report(tmp_path, monkeypatch):
    out = tmp_path / "gh_output"
    findings = tmp_path / "findings.jsonl"
    logs = tmp_path / "sound_logs.txt"
    fake = FakePi({"voice-assistant": "[ERROR] tts_node crashed: segfault"})
    monkeypatch.setattr(MODULE, "make_ssh_runner", lambda *a, **k: fake)
    for key, value in {
        "SCOPE": "sound", "PI_HOST": "10.1.1.12", "PI_USER": "ros2", "PI_PASSWORD": "open", "PI_SSH_OPTS": "-o X=y",
        "PI_COMPOSE_DIR": "~/rob_box_project/docker/sound", "ENVIRONMENT": "staging",
        "FINDINGS_FILE": str(findings), "LOGS_FILE": str(logs), "GITHUB_OUTPUT": str(out),
    }.items():
        monkeypatch.setenv(key, value)

    assert MODULE.main() == 0

    outputs = dict(line.split("=", 1) for line in out.read_text(encoding="utf-8").splitlines())
    assert outputs == {"critical_count": "1", "warning_count": "0", "has_issues": "true"}
    assert json.loads(findings.read_text(encoding="utf-8").splitlines()[0])["container"] == "voice-assistant"
    assert "voice-assistant CRITICAL ERRORS" in logs.read_text(encoding="utf-8")


def test_main_clean_run_has_empty_report_and_has_issues_false(tmp_path, monkeypatch):
    """Прежний bash писал `echo ""` в файл отчёта, и has_issues был true всегда."""
    out = tmp_path / "gh_output"
    logs = tmp_path / "vision_logs.txt"
    monkeypatch.setattr(MODULE, "make_ssh_runner", lambda *a, **k: FakePi({"oak-d": "all fine"}))
    for key, value in {
        "SCOPE": "vision", "PI_HOST": "h", "PI_USER": "u", "PI_PASSWORD": "p", "PI_SSH_OPTS": "",
        "PI_COMPOSE_DIR": "d", "LOGS_FILE": str(logs), "GITHUB_OUTPUT": str(out),
    }.items():
        monkeypatch.setenv(key, value)
    monkeypatch.delenv("FINDINGS_FILE", raising=False)

    assert MODULE.main() == 0
    assert logs.read_text(encoding="utf-8") == ""
    assert "has_issues=false" in out.read_text(encoding="utf-8")


def test_ssh_runner_builds_sshpass_argv_and_maps_timeout(monkeypatch):
    seen = {}

    class Proc:
        returncode, stdout, stderr = 0, "out", ""

    def fake_run(argv, **kwargs):
        seen["argv"], seen["kwargs"] = argv, kwargs
        return Proc()

    monkeypatch.setattr(MODULE.subprocess, "run", fake_run)
    run_remote = MODULE.make_ssh_runner("10.1.1.12", "ros2", "open", "-o StrictHostKeyChecking=no -o ConnectTimeout=10")
    assert run_remote("uptime") == (0, "out")
    assert seen["argv"] == [
        "sshpass", "-p", "open", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10",
        "ros2@10.1.1.12", "uptime",
    ]
    assert seen["kwargs"]["timeout"] == MODULE.SSH_CALL_TIMEOUT_SEC

    def boom(argv, **kwargs):
        raise MODULE.subprocess.TimeoutExpired(argv, 1)

    monkeypatch.setattr(MODULE.subprocess, "run", boom)
    assert run_remote("uptime") == (124, "")
