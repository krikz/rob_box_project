#!/usr/bin/env python3
"""Проверка логов контейнеров одного Pi (шаг «[<Pi>] Check Container Logs»).

Раньше этот шаг был ~110 строк bash, продублированных в L-Deploy and Verify.yml
для каждого Pi (Vision, Main); с ADR-0159 Pi три, и копировать шаг в третий раз
значило держать три реализации одной логики (с неиспользуемыми EXCLUDE_*-
переменными в придачу). Здесь она одна, параметризована окружением — как
check_container_status.sh.

Что делает (поведение прежнего шага сохранено):
  * `docker compose ps --format json` на Pi → имена контейнеров;
  * для каждого — первые 50 строк из `--tail 10000` (инициализация) и последние
    150 строк; склейка через разделитель;
  * `deployment_issue_dedup.extract_relevant_log_line` (тот же модуль, что
    раньше вызывался как `python3 deployment_issue_dedup.py extract-log`) —
    critical и warning с исключениями по scope;
  * findings JSONL (FINDINGS_FILE), текстовый отчёт (LOGS_FILE),
    outputs critical_count / warning_count / has_issues в GITHUB_OUTPUT.

Отличия от bash-версии (осознанные, ADR-0018):
  * has_issues = «есть critical/warning». Раньше LOGS_FILE создавался через
    `echo ""` (1 байт), и `[ -s ]` было истинно всегда → has_issues=true на
    любом деплое, а диагностика пустого файла уходила в deploy-issue.
  * сбой SSH при чтении логов одного контейнера не валит шаг и не молчит:
    ::warning:: с именем контейнера и кодом (раньше `set -e` обрывал весь шаг
    на первом `No such container`).

Окружение: PI_HOST, PI_USER, PI_PASSWORD, PI_COMPOSE_DIR, PI_SSH_OPTS, SCOPE
(main|vision|sound), ENVIRONMENT, FINDINGS_FILE, LOGS_FILE (по умолчанию
<SCOPE>_logs.txt), GITHUB_OUTPUT (опционально).
"""

from __future__ import annotations

import importlib.util
import json
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Callable

SEPARATOR = "---[...SKIPPED MIDDLE LOGS...]---"
SSH_CALL_TIMEOUT_SEC = 120

_DEDUP_PATH = Path(__file__).resolve().parent / "deployment_issue_dedup.py"
_spec = importlib.util.spec_from_file_location("deployment_issue_dedup", _DEDUP_PATH)
assert _spec is not None and _spec.loader is not None
dedup = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dedup)

# (команда на Pi) -> (код возврата, stdout)
RemoteRunner = Callable[[str], "tuple[int, str]"]


def make_ssh_runner(host: str, user: str, password: str, ssh_opts: str) -> RemoteRunner:
    def run(command: str) -> tuple[int, str]:
        argv = ["sshpass", "-p", password, "ssh", *shlex.split(ssh_opts), f"{user}@{host}", command]
        try:
            proc = subprocess.run(
                argv, capture_output=True, text=True, errors="replace", timeout=SSH_CALL_TIMEOUT_SEC, check=False
            )
        except subprocess.TimeoutExpired:
            return 124, ""
        if proc.stderr:
            sys.stderr.write(proc.stderr)
        return proc.returncode, proc.stdout

    return run


def list_containers(run: RemoteRunner, compose_dir: str) -> list[str]:
    rc, out = run(f"cd {compose_dir} && docker compose ps --format json")
    if rc != 0:
        print(f"::warning::docker compose ps в {compose_dir} завершился с кодом {rc} — логи не проверены")
    names = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        # `compose ps --format json` — либо JSON-lines, либо один массив.
        for item in obj if isinstance(obj, list) else [obj]:
            if isinstance(item, dict) and item.get("Name"):
                names.append(item["Name"])
    return names


def fetch_logs(run: RemoteRunner, container: str) -> tuple[str, str]:
    """(начало: первые 50 строк из tail 10000, хвост: последние 150)."""
    rc_start, start = run(f"docker logs --tail 10000 {container} 2>&1 | head -50")
    rc_end, end = run(f"docker logs --tail 150 {container} 2>&1")
    if rc_start not in (0, 141) or rc_end != 0:  # 141 = SIGPIPE от head
        print(f"::warning title=Logs unavailable::{container}: чтение логов вернуло коды {rc_start}/{rc_end}")
    return start.rstrip("\n"), end.rstrip("\n")


def finding(environment: str, scope: str, container: str, kind: str, severity: str, summary: str, raw: str) -> dict:
    return {
        "environment": environment,
        "scope": scope,
        "container": container,
        "kind": kind,
        "severity": severity,
        "summary": summary,
        "raw_text": raw,
    }


def check_logs(run: RemoteRunner, *, scope: str, environment: str, compose_dir: str) -> dict:
    """Чистая часть: возвращает {critical, warning, report, findings}."""
    critical_count = warning_count = 0
    report: list[str] = []
    findings: list[dict] = []

    for container in list_containers(run, compose_dir):
        print(f"Checking logs for: {container}")
        logs_start, logs_end = fetch_logs(run, container)
        logs = f"{logs_start}\n{SEPARATOR}\n{logs_end}"

        critical = dedup.extract_relevant_log_line(logs, scope=scope, severity="critical")
        if critical:
            critical_count += 1
            print(f"  ❌ CRITICAL errors found in {container}")
            report += [f"=== {container} CRITICAL ERRORS ===", critical, ""]
            findings.append(
                finding(environment, scope, container, "critical_log", "critical", "Critical log pattern detected", critical)
            )

        warning = dedup.extract_relevant_log_line(logs, scope=scope, severity="warning")
        if warning:
            warning_count += 1
            print(f"  ⚠️  Warnings found in {container}")
            report += [f"=== {container} WARNINGS ===", warning, ""]
            findings.append(
                finding(environment, scope, container, "warning_log", "warning", "Warning log pattern detected", warning)
            )

        if critical:
            report += [f"=== {container} INITIALIZATION CONTEXT ===", logs_start, ""]

    return {"critical": critical_count, "warning": warning_count, "report": report, "findings": findings}


def main() -> int:
    env = os.environ
    scope = env["SCOPE"]
    run = make_ssh_runner(env["PI_HOST"], env["PI_USER"], env["PI_PASSWORD"], env["PI_SSH_OPTS"])
    result = check_logs(run, scope=scope, environment=env.get("ENVIRONMENT", ""), compose_dir=env["PI_COMPOSE_DIR"])

    logs_file = Path(env.get("LOGS_FILE") or f"{scope}_logs.txt")
    text = "\n".join(result["report"])
    logs_file.write_text(text + "\n" if text else "", encoding="utf-8")

    findings_file = env.get("FINDINGS_FILE")
    if findings_file and result["findings"]:
        with open(findings_file, "a", encoding="utf-8") as fh:
            for item in result["findings"]:
                fh.write(json.dumps(item, ensure_ascii=False) + "\n")

    has_issues = bool(result["critical"] or result["warning"])
    outputs = {
        "critical_count": result["critical"],
        "warning_count": result["warning"],
        "has_issues": "true" if has_issues else "false",
    }
    if env.get("GITHUB_OUTPUT"):
        with open(env["GITHUB_OUTPUT"], "a", encoding="utf-8") as fh:
            for key, value in outputs.items():
                fh.write(f"{key}={value}\n")

    if has_issues:
        print(text)
    else:
        print("✅ No critical errors or warnings found")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
