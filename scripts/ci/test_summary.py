#!/usr/bin/env python3
"""Честная сводка «G: Run Tests».

Джоба Integration Tests на pull_request skipped: её `if` требует ref
develop/main (карточка #5116). Раньше Test Summary не знала про неё и
писала «All tests passed!» — зелёный текст читался как «интеграция прошла».

Интеграцию воротами PR не делаем (решение Шифу 22.11.2026): skipped не
меняет код выхода. Меняется только текст: строка про Integration Tests
и итог, который не выдаёт непрогнанное за пройденное.

На push result джобы интеграции приходит из needs. Пустая строка там —
«результат не передан», не «не запускались»: так сводка не врёт в обратную
сторону про уже прогнанную джобу (run 45041513102).

CLI (из шага workflow, результаты джоб уже подставлены):

    python3 scripts/ci/test_summary.py \\
        --event pull_request \\
        --suite "Unit Tests=success" \\
        --suite "Unit Tests (rob_box_mcp_tools)=success" \\
        --integration-result skipped
"""

from __future__ import annotations

import argparse
import sys

INTEGRATION_SUITE = "Integration Tests"

# Итог, когда обязательные сюиты зелёные, а интеграция на PR не бежала.
# Не «All tests passed»: непрогнанное не называется пройденным.
PR_SKIPPED_FOOTER = (
    "Обязательные сюиты PR прошли. Integration Tests на PR не запускались "
    "(skipped: только develop/main) — это не «интеграция прошла»."
)

ALL_RAN_FOOTER = "Все запущенные сюиты прошли, включая Integration Tests."

FAILED_FOOTER = "Tests failed! Check logs above for details."


# Пустой --integration-result на push: шаг сводки не передал needs.*.result.
# Это не «не запускались» (так пишем только про skipped на PR) и не success.
RESULT_NOT_PASSED = "результат не передан"


def integration_status_line(event_name: str, result: str) -> str:
    """Строка таблицы для Integration Tests.

    На pull_request джоба skipped по условию ref, а не потому что тесты
    прошли. Пишем это явно, каким бы ни пришёл result.

    На push (develop/main) джоба в needs, и сюда приходит её result.
    Пустая строка — шаг не передал результат: это не «не запускались».
    """
    if event_name == "pull_request":
        return (
            f"| {INTEGRATION_SUITE} | не запускались "
            "(skipped на PR, только develop/main) |"
        )
    shown = result.strip() or RESULT_NOT_PASSED
    return f"| {INTEGRATION_SUITE} | {shown} |"


def render_summary(
    event_name: str, suites: list[tuple[str, str]], integration_result: str
) -> tuple[str, int]:
    """Текст сводки и код выхода.

    Код 1 — только если обязательная сюита не success. skipped интеграции
    код не меняет: PR-CI остаётся зелёным, текст — нет.
    """
    lines = [
        "## Test Results Summary",
        "",
        "| Test Suite | Status |",
        "|------------|--------|",
    ]
    failed = []
    for name, result in suites:
        shown = result.strip() or "unknown"
        lines.append(f"| {name} | {shown} |")
        if shown != "success":
            failed.append(name)
    lines.append(integration_status_line(event_name, integration_result))
    lines.append("")

    if failed:
        lines.append(f"⚠️ **{FAILED_FOOTER}**")
        lines.append("")
        lines.append("Не прошли: " + ", ".join(failed) + ".")
        return "\n".join(lines) + "\n", 1

    if event_name == "pull_request":
        lines.append(PR_SKIPPED_FOOTER)
    elif integration_result.strip() == "success":
        lines.append(ALL_RAN_FOOTER)
    else:
        shown = integration_result.strip() or RESULT_NOT_PASSED
        lines.append(
            "Обязательные сюиты прошли. "
            f"Integration Tests: {shown} — это не «интеграция прошла»."
        )
    return "\n".join(lines) + "\n", 0


def _parse_suite(raw: str) -> tuple[str, str]:
    name, sep, result = raw.partition("=")
    if not sep or not name.strip():
        raise argparse.ArgumentTypeError(
            f"--suite ждёт 'Имя=result', получено: {raw!r}"
        )
    return name.strip(), result.strip()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Честная сводка G: Run Tests")
    parser.add_argument("--event", required=True, help="github.event_name")
    parser.add_argument(
        "--suite",
        action="append",
        default=[],
        type=_parse_suite,
        help="Обязательная сюита: 'Имя=result'. Повторять.",
    )
    parser.add_argument(
        "--integration-result",
        default="",
        help="result джобы integration-tests; пусто — результат не передан",
    )
    args = parser.parse_args(argv)
    text, code = render_summary(args.event, args.suite, args.integration_result)
    sys.stdout.write(text)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
