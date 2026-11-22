#!/usr/bin/env python3
"""Проверка логов контейнеров одного Pi (шаг «[<Pi>] Check Container Logs»).

Раньше этот шаг был ~110 строк bash, продублированных в L-Deploy and Verify.yml
для каждого Pi (Vision, Main); с ADR-0159 Pi три, и копировать шаг в третий раз
значило держать три реализации одной логики (с неиспользуемыми EXCLUDE_*-
переменными в придачу). Здесь она одна, параметризована окружением — как
check_container_status.sh.

Что делает (поведение прежнего шага сохранено):
  * `docker compose ps --format json` на Pi → имена контейнеров;
  * граница текущего запуска: `docker inspect` → StartedAt и RestartCount
    (issue #5135). Логи читаются окном `--since StartedAt`: ошибка прежнего
    запуска, уже отработанная рестартом, не красит деплой;
  * внутри окна — первые 50 строк из `--tail 10000` (инициализация) и последние
    150 строк; склейка через разделитель;
  * строки старше StartedAt при RestartCount > 0 не прячутся: отдельный блок
    отчёта RECOVERED BEFORE START, без поднятия счётчиков;
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
from datetime import datetime
from pathlib import Path
from typing import Callable

SEPARATOR = "---[...SKIPPED MIDDLE LOGS...]---"
SSH_CALL_TIMEOUT_SEC = 120
# Сколько отработанных рестартом строк показать в сводке. Больше — шум
# карточки; сам факт рестарта в заголовке блока виден и без списка.
RECOVERED_LINES_LIMIT = 5
# Строка «Initial conf: {...» zenohd — весь JSON-конфиг роутера в одну строку
# (десятки КБ). Она попадает в первые 50 строк и целиком уезжает в комментарий
# карточки деплоя как INITIALIZATION CONTEXT (issue #5124). В отчёте режем;
# сам лог и сырой текст finding не трогаем.
REPORT_LINE_LIMIT = 500
REPORT_LINE_MARK = " …[обрезано, {n} символов]"

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


def inspect_start(run: RemoteRunner, container: str) -> tuple[str, int] | None:
    """(StartedAt, RestartCount) текущего запуска. None — inspect не сработал.

    Не молчим: без границы окна честный красный лучше тихого пропуска
    (issue #5135, ADR-0018).
    """
    rc, out = run(f"docker inspect -f '{{{{.State.StartedAt}}}} {{{{.RestartCount}}}}' {container}")
    parts = out.split()
    if rc != 0 or len(parts) != 2 or not parts[1].isdigit():
        detail = out.strip() or f"код {rc}"
        print(f"::warning title=Logs window unavailable::{container}: {detail}")
        return None
    return parts[0], int(parts[1])


def fetch_logs(run: RemoteRunner, container: str, *, since: str | None = None) -> tuple[str, str]:
    """(начало: первые 50 строк из tail 10000, хвост: последние 150).

    ``since`` — граница текущего запуска (StartedAt). Окно режет строки
    прежнего запуска того же контейнера (issue #5135). Без границы читаем
    полный лог — прежнее поведение.
    """
    window = f"--since {shlex.quote(since)} " if since else ""
    rc_start, start = run(f"docker logs {window}--tail 10000 {container} 2>&1 | head -50")
    rc_end, end = run(f"docker logs {window}--tail 150 {container} 2>&1")
    if rc_start not in (0, 141) or rc_end != 0:  # 141 = SIGPIPE от head
        print(f"::warning title=Logs unavailable::{container}: чтение логов вернуло коды {rc_start}/{rc_end}")
    return start.rstrip("\n"), end.rstrip("\n")


def recovered_lines(
    full_text: str, window_text: str, *, scope: str, container: str = "", since: str | None = None
) -> list[str]:
    """Critical-строки полного лога, которых нет в окне текущего запуска.

    Дедуп ищет первую подходящую строку, поэтому обходим по одной: так в
    сводку попадают все отработанные рестартом, а не только первая.
    ``container`` нужен окну старта роутера (issue #5124): без имени та же
    строка транзиента осталась бы critical. Транзиент текущего запуска (его
    старт не раньше ``since``) в RECOVERED не кладём — его уже видит warning.
    """
    seen = {line.strip() for line in window_text.splitlines()}
    since_at = _parse_started_at(since) if since else None
    found: list[str] = []
    for line in full_text.splitlines():
        stripped = line.strip()
        if not stripped or stripped in seen or stripped in found:
            continue
        if not dedup.extract_relevant_log_line(stripped, scope=scope, severity="critical", container=container):
            continue
        if _router_transient_of_this_start(stripped, full_text, container, since_at):
            continue
        found.append(stripped)
    return found


def _parse_started_at(started_at: str):
    """Метка ``docker inspect`` StartedAt. Наносекунды режем: fromisoformat их не ест.

    Не ``_iso_z_timestamp``: тот ищет метку в строке лога и берёт только то, что
    уже прошло TIMESTAMP_RE. Здесь метка голая, с долями до 9 знаков.
    """
    text = started_at.strip().replace("Z", "+00:00").replace("z", "+00:00")
    if "." in text:
        head, rest = text.split(".", 1)
        digits = []
        for ch in rest:
            if not ch.isdigit():
                break
            digits.append(ch)
        text = f"{head}.{''.join(digits[:6])}{rest[len(digits):]}"
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        return None


def _router_transient_of_this_start(line: str, log_text: str, container: str, since_at) -> bool:
    """Транзиент роутера, чей старт не старше границы текущего запуска."""
    if since_at is None or not dedup._zenoh_router_startup_transient(line, log_text, container):
        return False
    start_at, _error_at = dedup._zenoh_router_start_and_error(line, log_text)
    return start_at is not None and start_at >= since_at


def _clip_report_line(line: str) -> str:
    """Обрезать гигантскую строку в тексте отчёта. Сырой лог не меняется."""
    if len(line) <= REPORT_LINE_LIMIT:
        return line
    mark = REPORT_LINE_MARK.format(n=len(line))
    return line[: REPORT_LINE_LIMIT - len(mark)] + mark


def _clip_report(lines: list[str]) -> list[str]:
    return [_clip_report_line(line) for line in lines]


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
        start = inspect_start(run, container)
        since = start[0] if start else None
        logs_start, logs_end = fetch_logs(run, container, since=since)
        logs = f"{logs_start}\n{SEPARATOR}\n{logs_end}"

        critical = dedup.extract_relevant_log_line(logs, scope=scope, severity="critical", container=container)
        if critical:
            critical_count += 1
            print(f"  ❌ CRITICAL errors found in {container}")
            report += [f"=== {container} CRITICAL ERRORS ===", critical, ""]
            findings.append(
                finding(
                    environment, scope, container, "critical_log", "critical", "Critical log pattern detected", critical
                )
            )

        warning = dedup.extract_relevant_log_line(logs, scope=scope, severity="warning", container=container)
        if warning:
            warning_count += 1
            print(f"  ⚠️  Warnings found in {container}")
            # Стартовый транзиент роутера не молчит: в сводке видно, что это окно
            # старта, а не глухая ошибка (issue #5124). Прочие warning без пометки.
            summary = "Warning log pattern detected"
            if dedup.is_zenoh_router_container(container) and dedup.ZENOH_ROUTER_PROPAGATION_RE.search(warning):
                summary = dedup.ZENOH_ROUTER_STARTUP_SUMMARY
                report.append(f"=== {container} {summary} ===")
            report += [f"=== {container} WARNINGS ===", warning, ""]
            findings.append(finding(environment, scope, container, "warning_log", "warning", summary, warning))

        if critical:
            report.append(f"=== {container} INITIALIZATION CONTEXT ===")
            report.extend(logs_start.splitlines())
            report.append("")

        # Отработанный рестарт виден в сводке, но не красит деплой: счётчики
        # выше считаются только по окну текущего запуска (issue #5135).
        if start and start[1] > 0:
            full_start, full_end = fetch_logs(run, container)
            before = recovered_lines(f"{full_start}\n{full_end}", logs, scope=scope, container=container, since=since)
            shown, rest = before[:RECOVERED_LINES_LIMIT], before[RECOVERED_LINES_LIMIT:]
            if shown:
                report.append(
                    f"=== {container} RECOVERED BEFORE START (RestartCount={start[1]}, StartedAt={start[0]}) ==="
                )
                report.extend(shown)
                if rest:
                    report.append(f"... ещё {len(rest)} строк старше StartedAt")
                report.append("")

    return {
        "critical": critical_count,
        "warning": warning_count,
        "report": _clip_report(report),
        "findings": findings,
    }


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
