"""Issue #5124: стартовый транзиент zenoh-роутера — warning, не глушилка класса.

Роутер в первые секунды пишет ERROR «Error propagating <kind> @/<zid>/router/**»,
когда gossip объявил лицо и тут же его потерял (run 45040975024: +1.16 с после
«Zenoh can be reached at:»). В пределах ~10 с после строки старта того же процесса
строка понижается до warning. Вне окна и без строки старта в дампе класс остаётся
critical: шторм переподключений посреди работы должен красить деплой.

ZID и текст синтетические — реальные идентификаторы роутера не нужны.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
SPEC = importlib.util.spec_from_file_location("deployment_issue_dedup", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

ZID = "aabbccddeeff00112233445566778899"

START_REACHED = (
    "2026-11-22T15:58:01.394000Z  INFO main ThreadId(01) zenohd: Zenoh can be reached at: tcp/10.1.1.12:7447"
)
START_VERSION = "2026-11-22T15:58:01.100000Z  INFO main ThreadId(01) zenohd: zenohd v1.0.0"

PROPAGATION = (
    "2026-11-22T15:58:02.544000Z ERROR rx-1 ThreadId(07) zenoh::net::routing::hat::router::queries: "
    f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!"
)
PROPAGATION_LATE = (
    "2026-11-22T16:00:02.544000Z ERROR rx-1 ThreadId(07) zenoh::net::routing::hat::router::queries: "
    f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!"
)


def critical(log_text: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope="sound", severity="critical", container="zenoh-router-sound")


def warning(log_text: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope="sound", severity="warning", container="zenoh-router-sound")


def test_startup_window_demotes_propagation_one_second_after_start() -> None:
    # (а) строка старта + ошибка через ~1 с → warning, не critical.
    logs = "\n".join([START_REACHED, PROPAGATION])

    assert critical(logs) is None
    assert warning(logs) == PROPAGATION


def test_version_banner_also_opens_the_startup_window() -> None:
    # Вторая форма строки старта того же процесса — баннер «zenohd v».
    logs = "\n".join([START_VERSION, PROPAGATION])

    assert critical(logs) is None
    assert warning(logs) == PROPAGATION


def test_same_error_outside_window_stays_critical() -> None:
    # (б) та же ошибка через 120 с после старта — уже не транзиент.
    logs = "\n".join([START_REACHED, PROPAGATION_LATE])

    assert critical(logs) == PROPAGATION_LATE
    assert warning(logs) is None


def test_propagation_without_startup_line_stays_critical() -> None:
    # (в) дамп без строки старта: понижать нечем, класс остаётся critical.
    assert critical(PROPAGATION) == PROPAGATION
    assert warning(PROPAGATION) is None


def test_error_before_startup_line_is_not_inside_the_window() -> None:
    # Ошибка раньше строки старта в окно не попадает: дельта отрицательная.
    logs = "\n".join([PROPAGATION, START_REACHED])

    assert critical(logs) == PROPAGATION


def test_other_zenoh_error_is_not_demoted_even_inside_the_window() -> None:
    # Правило узкое: другое сообщение zenoh в том же окне остаётся critical.
    other = (
        "2026-11-22T15:58:03.000000Z ERROR rx-1 ThreadId(07) "
        "zenoh::net::routing::hat::router::queries: session closed unexpectedly"
    )
    logs = "\n".join([START_REACHED, other])

    assert critical(logs) == other


def test_propagation_outside_router_module_stays_critical() -> None:
    # Тот же текст «Error propagating», но не из модуля роутера — не наше окно.
    foreign = (
        "2026-11-22T15:58:02.544000Z ERROR rx-1 ThreadId(07) app::bridge: "
        f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!"
    )
    logs = "\n".join([START_REACHED, foreign])

    assert critical(logs) == foreign


def test_window_does_not_hide_a_real_error_in_the_same_dump() -> None:
    real = (
        "2026-11-22T15:58:04.000000Z ERROR main ThreadId(02) zenohd: "
        "failed to bind tcp listener: address already in use"
    )
    logs = "\n".join([START_REACHED, PROPAGATION, real])

    assert critical(logs) == real
    assert warning(logs) == PROPAGATION
