"""Issue #5124: окно старта zenoh-роутера привязано к имени контейнера.

Три кейса карточки, тексты синтетические (реальные ZID не нужны):
(а) старт + ошибка через 1 с у контейнера роутера → warning;
(б) та же ошибка через 120 с → critical;
(в) ошибка без строки старта в дампе → critical.

Плюс узкость правила: та же строка в дампе чужого контейнера остаётся critical,
даже если рядом есть строка старта. Понижение не молчит — summary несёт пометку
стартового транзиента.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

DEDUP_PATH = Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
DEDUP_SPEC = importlib.util.spec_from_file_location("deployment_issue_dedup_5124", DEDUP_PATH)
DEDUP = importlib.util.module_from_spec(DEDUP_SPEC)
assert DEDUP_SPEC is not None and DEDUP_SPEC.loader is not None
DEDUP_SPEC.loader.exec_module(DEDUP)

LOGS_PATH = Path(__file__).resolve().parents[1] / "check_container_logs.py"
LOGS_SPEC = importlib.util.spec_from_file_location("check_container_logs_5124", LOGS_PATH)
LOGS = importlib.util.module_from_spec(LOGS_SPEC)
assert LOGS_SPEC is not None and LOGS_SPEC.loader is not None
LOGS_SPEC.loader.exec_module(LOGS)

ZID = "00112233445566778899aabbccddeeff"
ROUTER = "zenoh-router-sound"

START = "2026-11-22T15:58:01.394000Z  INFO main ThreadId(01) zenohd: Zenoh can be reached at: tcp/10.0.0.12:7447"
VERSION = "2026-11-22T15:58:01.100000Z  INFO main ThreadId(01) zenohd: zenohd v1.0.0"
ERROR_1S = (
    "2026-11-22T15:58:02.544000Z ERROR rx-1 ThreadId(07) zenoh::net::routing::hat::router::queries: "
    f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!"
)
ERROR_120S = (
    "2026-11-22T16:00:01.394000Z ERROR rx-1 ThreadId(07) zenoh::net::routing::hat::router::queries: "
    f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!"
)


def classify(log_text: str, severity: str, container: str) -> str | None:
    return DEDUP.extract_relevant_log_line(log_text, scope="sound", severity=severity, container=container)


def test_router_start_plus_error_one_second_later_is_warning() -> None:
    # (а) старт + ошибка через 1 с → warning, в critical её нет.
    logs = "\n".join([START, ERROR_1S])

    assert classify(logs, "critical", ROUTER) is None
    assert classify(logs, "warning", ROUTER) == ERROR_1S


def test_same_error_120_seconds_after_start_stays_critical() -> None:
    # (б) та же ошибка через 120 с — вне окна, critical.
    logs = "\n".join([START, ERROR_120S])

    assert classify(logs, "critical", ROUTER) == ERROR_120S
    assert classify(logs, "warning", ROUTER) is None


def test_error_without_startup_line_stays_critical() -> None:
    # (в) дамп без строки старта: понижать нечем.
    assert classify(ERROR_1S, "critical", ROUTER) == ERROR_1S
    assert classify(ERROR_1S, "warning", ROUTER) is None


@pytest.mark.parametrize("container", ["voice-assistant", "nav2", "cadvisor", ""])
def test_same_dump_in_a_foreign_container_stays_critical(container: str) -> None:
    # Правило узкое: чужой контейнер (и дамп без имени) не понижается.
    logs = "\n".join([VERSION, START, ERROR_1S])

    assert classify(logs, "critical", container) == ERROR_1S
    assert classify(logs, "warning", container) is None


@pytest.mark.parametrize(
    "container",
    [
        "zenoh-router",
        "zenoh-router-vision",
        "zenoh-router-sound",
        "rob-box-sim-zenoh-router",
        "zenohd",
        "eclipse/zenoh",
    ],
)
def test_known_router_container_names_open_the_window(container: str) -> None:
    logs = "\n".join([START, ERROR_1S])

    assert classify(logs, "critical", container) is None
    assert classify(logs, "warning", container) == ERROR_1S


STARTED_AT = "2026-11-22T15:58:01.000000000Z"


class _RouterPi:
    """Подмена SSH: один контейнер роутера.

    ``inspect_rc`` != 0 — путь «Logs window unavailable»: полный лог, без ``--since``.
    С заданным ``window`` inspect отдаёт StartedAt, и полный лог уходит только
    в обход RECOVERED (без ``--since``).
    """

    def __init__(self, name: str, logs: str, *, window: str | None = None, inspect_rc: int = 1) -> None:
        self.name = name
        self.logs = logs
        self.window = window
        self.inspect_rc = inspect_rc

    def __call__(self, command: str) -> tuple[int, str]:
        if "docker compose ps" in command:
            return 0, json.dumps({"Name": self.name, "State": "running"})
        if "docker inspect" in command:
            if self.inspect_rc != 0:
                return self.inspect_rc, "inspect failed"
            return 0, f"{STARTED_AT} 1"
        if command.startswith("docker logs"):
            if "--since" in command:
                return 0, (self.window if self.window is not None else self.logs) + "\n"
            return 0, self.logs + "\n"
        raise AssertionError(command)


def test_demoted_line_stays_in_the_report_and_summary_names_the_transient() -> None:
    # Понижение не молчит: строка в отчёте как warning, summary говорит «транзиент».
    pi = _RouterPi(ROUTER, "\n".join([START, ERROR_1S]))
    result = LOGS.check_logs(pi, scope="sound", environment="dev", compose_dir="~/unused")

    assert result["critical"] == 0
    assert result["warning"] == 1
    report = "\n".join(result["report"])
    assert ERROR_1S in report
    assert "WARNINGS" in report
    assert DEDUP.ZENOH_ROUTER_STARTUP_SUMMARY in report
    assert result["findings"][0]["severity"] == "warning"
    assert result["findings"][0]["summary"] == DEDUP.ZENOH_ROUTER_STARTUP_SUMMARY
    assert result["findings"][0]["raw_text"] == ERROR_1S


def test_startup_transient_of_this_start_is_not_listed_as_recovered() -> None:
    # Шов #5135: recovered_lines без имени контейнера вернул бы транзиент как
    # critical и положил бы его в RECOVERED BEFORE START. Старт не старше
    # StartedAt — это окно текущего запуска, не отработанный рестарт.
    pi = _RouterPi(ROUTER, "\n".join([START, ERROR_1S]), window="[INFO] router up", inspect_rc=0)
    result = LOGS.check_logs(pi, scope="sound", environment="dev", compose_dir="~/unused")
    report = "\n".join(result["report"])

    assert result["critical"] == 0
    assert result["warning"] == 0
    assert "RECOVERED BEFORE START" not in report
    assert ERROR_1S not in report


def test_startup_transient_of_a_previous_run_stays_recovered() -> None:
    # Транзиент прошлого запуска (старт раньше StartedAt) — отработанный рестарт:
    # в сводке виден, деплой не красит.
    previous = "\n".join(
        [
            "2026-11-22T15:50:01.000000Z  INFO zenohd: Zenoh can be reached at: tcp/10.0.0.12:7447",
            "2026-11-22T15:50:02.000000Z ERROR rx-1 ThreadId(07) zenoh::net::routing::hat::router::queries: "
            f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!",
        ]
    )
    pi = _RouterPi(ROUTER, previous, window="[INFO] router up", inspect_rc=0)
    result = LOGS.check_logs(pi, scope="sound", environment="dev", compose_dir="~/unused")
    report = "\n".join(result["report"])

    assert result["critical"] == 0
    assert "RECOVERED BEFORE START" in report
    assert f"@{ZID}/router/**" in report
