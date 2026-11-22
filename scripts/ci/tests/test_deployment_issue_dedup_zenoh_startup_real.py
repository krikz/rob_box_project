"""Issue #5124: реальная строка прогона 45040975024 должна проходить через гейт.

Строка взята из шага Deployment Summary (Sound Pi, zenoh-router-sound), как её
печатает zenohd: ANSI-коды и слэш после «@». Синтетический вариант без слэша
гейт не доказывает — в бою его нет.

    2026-11-22T15:58:02.543989Z ERROR ... router::queries: Error propagating qabl @/5d3d.../router/**
    2026-11-22T15:58:01.383656Z INFO ... zenohd: zenohd v1.6.2 ...
    2026-11-22T15:58:01.394563Z INFO ... orchestrator: Zenoh can be reached at: tcp/10.1.1.12:7447

Проверяется публичная функция extract_relevant_log_line, не внутренний шаблон.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
SPEC = importlib.util.spec_from_file_location("deployment_issue_dedup", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

# ANSI zenohd: тусклая метка времени, цвет уровня, тусклые модуль и двоеточие.
DIM = "\x1b[2m"
RESET = "\x1b[0m"
RED = "\x1b[31m"
GREEN = "\x1b[32m"

ZID = "5d3d4121ec8ef8f47ddb31e9d7b4a9ea"

ERROR_PLAIN = (
    "2026-11-22T15:58:02.543989Z ERROR rx-1 ThreadId(07) "
    "zenoh::net::routing::hat::router::queries: "
    f"Error propagating qabl @{ZID}/router/**: cannot get index of {ZID}!"
).replace(f"@{ZID}", f"@/{ZID}")

ERROR_ANSI = (
    f"{DIM}2026-11-22T15:58:02.543989Z{RESET} {RED}ERROR{RESET} rx-1 ThreadId(07) "
    f"{DIM}zenoh::net::routing::hat::router::queries{RESET}{DIM}:{RESET} "
    f"Error propagating qabl @/{ZID}/router/**: cannot get index of {ZID}!"
)

START_PLAIN = (
    "2026-11-22T15:58:01.383656Z  INFO main ThreadId(01) zenohd: "
    "zenohd v1.6.2 built with rustc 1.85.0 (4d91de4e4 2025-02-17)"
)
START_ANSI = (
    f"{DIM}2026-11-22T15:58:01.383656Z{RESET} {GREEN} INFO{RESET} main ThreadId(01) "
    f"{DIM}zenohd{RESET}{DIM}:{RESET} zenohd v1.6.2 built with rustc 1.85.0 (4d91de4e4 2025-02-17)"
)

REACHED_PLAIN = (
    "2026-11-22T15:58:01.394563Z  INFO main ThreadId(01) "
    "zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/10.1.1.12:7447"
)
REACHED_ANSI = (
    f"{DIM}2026-11-22T15:58:01.394563Z{RESET} {GREEN} INFO{RESET} main ThreadId(01) "
    f"{DIM}zenoh::net::runtime::orchestrator{RESET}{DIM}:{RESET} "
    "Zenoh can be reached at: tcp/10.1.1.12:7447"
)

ERROR_LATE = ERROR_PLAIN.replace("2026-11-22T15:58:02.543989Z", "2026-11-22T16:00:02.543989Z")


def _critical(log_text: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope="sound", severity="critical")


def _warning(log_text: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope="sound", severity="warning")


def test_real_run_45040975024_line_with_slash_is_demoted_inside_window() -> None:
    # Точная строка прогона: «@/<zid>/router/**», окно +1.16 с. Critical быть не должно.
    logs = "\n".join([START_PLAIN, REACHED_PLAIN, ERROR_PLAIN])

    assert _critical(logs) is None
    assert _warning(logs) == ERROR_PLAIN


def test_real_line_keeps_ansi_codes_and_is_still_demoted() -> None:
    # zenohd пишет метку времени, уровень и «модуль:» в ANSI. Гейт читает docker logs
    # без -t, поэтому коды доезжают в дамп как есть (run 45040975024, строки 3091/3102/3109).
    logs = "\n".join([START_ANSI, REACHED_ANSI, ERROR_ANSI])

    assert _critical(logs) is None
    warned = _warning(logs)
    assert warned is not None and warned.strip() == ERROR_ANSI.strip()


def test_real_line_shifted_120s_stays_critical() -> None:
    logs = "\n".join([START_PLAIN, REACHED_PLAIN, ERROR_LATE])

    assert _critical(logs) == ERROR_LATE
    assert _warning(logs) is None
