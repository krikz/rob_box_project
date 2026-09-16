"""Unit tests for e2e-process pre-check UNREACHABLE handling (issue #2648).

Ретро 15.09 (t_2b4af5db): orchestrator на 156 живёт, но любой deploy
z-{e2e}/test-round-N на Vision Pi (10.1.1.21) падает пока хост лежит.
Если не делать pre-check, скрипт жжёт 40 мин build+deploy+e2e на лежащий
хост, fail-streak watchdog потом долбит Шифу метриками.

Фикс: добавить в ``agent-flow-e2e-process.sh`` функции
``e2e_target_pre_check()`` (ping + ssh true) и
``e2e_target_unreachable_inc()`` (textfile Prometheus counter), плюс блок
pre-check в начале per-issue loop ДО триггера build/deploy/e2e.

Acceptance — этот тест проверяет:

1. ``bash -n`` синтаксис скрипта (регресс).
2. Функции ``e2e_target_pre_check`` и ``e2e_target_unreachable_inc``
   определены на уровне скрипта (не внутри main()).
3. ``e2e_target_unreachable_inc`` корректно пишет textfile формата
   Prometheus exposition с лейблами {target_host, phase}.
4. ``e2e_target_unreachable_inc`` бампает counter при повторных вызовах
   (а не перезатирает на 1).
5. ``skip_robot_pre_check: 1`` распознаётся в ``## e2e`` блоке issue body
   (контракт для воркеров на Phase 1/2 perception-only).
6. Переменные ``E2E_PRECHECK_HOSTS`` / ``E2E_TARGET_UNREACHABLE_FILE`` /
   ``DEGRADED_LABEL`` / ``E2E_PRECHECK_PING_TIMEOUT`` /
   ``E2E_PRECHECK_SSH_TIMEOUT`` объявлены с разумными дефолтами.
7. Pre-check блок в per-issue loop находится ДО build/deploy/e2e-секций
   (структурный инвариант: при лежащем хосте build НЕ должен триггериться).

Why Python (не bash test): bash контракт «pre-check ДО build» — структурный
инвариант, который может дрейфовать при рефакторинге; pytest даёт единый
CLI entry point (через Makefile / CI) и ловит регрессии без сетевых вызовов.
"""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"


def _read_script() -> str:
    if not SCRIPT_PATH.exists():
        raise unittest.SkipTest(f"script not present at {SCRIPT_PATH}")
    return SCRIPT_PATH.read_text()


def _first_match_line(text: str, regex: re.Pattern[str]) -> int | None:
    """Return 1-indexed line number of first regex match, or None."""
    for i, line in enumerate(text.splitlines(), start=1):
        if regex.search(line):
            return i
    return None


PRECHECK_FN_RE = re.compile(r"^e2e_target_pre_check\s*\(\s*\)\s*\{")
INC_FN_RE = re.compile(r"^e2e_target_unreachable_inc\s*\(\s*\)\s*\{")
PRECHECK_BLOCK_RE = re.compile(
    r"Ретро 15\.09.*pre-check.*ping\+ssh", re.DOTALL
)
# В per-issue loop есть строка `for _h in ${E2E_PRECHECK_HOSTS}; do`
PRECHECK_LOOP_RE = re.compile(r"for\s+_h\s+in\s+\$\{E2E_PRECHECK_HOSTS\}")
# Build секция начинается с комментария "Ретро 12.08 (t_bff6eccf): round-1
# подвешен — процесс Terminated на ожидании билда". Найдём строку
# `triggering ${BUILD_WORKFLOW}` как маркер build dispatch.
BUILD_TRIGGER_RE = re.compile(
    r'log\s+"issue\s+#\$\{number\}:\s*triggering\s+\$\{BUILD_WORKFLOW\}'
)
# Метка degraded
DEGRADED_LABEL_RE = re.compile(
    r':\s*"\$\{DEGRADED_LABEL:=e2e:degraded\}"'
)
SKIP_FIELD_RE = re.compile(
    r"skip_robot_pre_check"
)


class PreCheckScript(unittest.TestCase):
    """Structural invariants for the new pre-check branch."""

    def test_script_parses(self) -> None:
        """``bash -n`` must succeed — no syntax errors."""
        if not SCRIPT_PATH.exists():
            self.skipTest(f"script not present at {SCRIPT_PATH}")
        result = subprocess.run(
            ["bash", "-n", str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"bash -n failed:\nstdout={result.stdout}\nstderr={result.stderr}",
        )

    def test_pre_check_fn_defined(self) -> None:
        text = _read_script()
        self.assertIsNotNone(
            _first_match_line(text, PRECHECK_FN_RE),
            "function `e2e_target_pre_check()` not defined — must be at top level",
        )

    def test_unreachable_inc_fn_defined(self) -> None:
        text = _read_script()
        self.assertIsNotNone(
            _first_match_line(text, INC_FN_RE),
            "function `e2e_target_unreachable_inc()` not defined — must be at top level",
        )

    def test_variables_declared_with_defaults(self) -> None:
        text = _read_script()
        for var in (
            "E2E_PRECHECK_HOSTS",
            "E2E_TARGET_UNREACHABLE_FILE",
            "DEGRADED_LABEL",
            "E2E_PRECHECK_PING_TIMEOUT",
            "E2E_PRECHECK_SSH_TIMEOUT",
        ):
            pat = re.compile(rf':\s*"\$\{{{var}:=')
            self.assertIsNotNone(
                _first_match_line(text, pat),
                f"variable {var} not declared with default — :\"${{{var}:=...}}\"",
            )
        # Дефолт DEGRADED_LABEL = 'e2e:degraded'
        self.assertIsNotNone(
            _first_match_line(text, DEGRADED_LABEL_RE),
            "DEGRADED_LABEL default must be 'e2e:degraded'",
        )

    def test_pre_check_loop_runs_per_host(self) -> None:
        """Per-issue loop must iterate over E2E_PRECHECK_HOSTS."""
        text = _read_script()
        self.assertIsNotNone(
            _first_match_line(text, PRECHECK_LOOP_RE),
            "pre-check must iterate `for _h in ${E2E_PRECHECK_HOSTS}`",
        )

    def test_pre_check_before_build_trigger(self) -> None:
        """Pre-check block must appear ABOVE build dispatch.

        При лежащем хосте build НЕ должен триггериться — иначе жжём 40 мин
        впустую. Структурный инвариант: pre-check строка выше первой build
        trigger log.
        """
        text = _read_script()
        precheck_line = _first_match_line(text, PRECHECK_BLOCK_RE)
        build_line = _first_match_line(text, BUILD_TRIGGER_RE)
        self.assertIsNotNone(
            precheck_line,
            "pre-check block comment not found (marker: 'Ретро 15.09.*pre-check.*ping+ssh')",
        )
        self.assertIsNotNone(
            build_line,
            "build trigger log not found — possibly renamed",
        )
        precheck_line_int: int = precheck_line  # type: ignore[assignment]
        build_line_int: int = build_line  # type: ignore[assignment]
        self.assertLess(
            precheck_line_int,
            build_line_int,
            msg=(
                f"pre-check block at line {precheck_line_int}, but build trigger "
                f"log at line {build_line_int}. Build MUST NOT fire on a dead host. "
                "Pre-check must run first."
            ),
        )

    def test_skip_robot_pre_check_field_supported(self) -> None:
        """`skip_robot_pre_check` в `## e2e` блоке — контракт для воркеров."""
        text = _read_script()
        self.assertIsNotNone(
            _first_match_line(text, SKIP_FIELD_RE),
            "skip_robot_pre_check field must be parsed from `## e2e` block — "
            "Phase 1/2 perception-only workers need to skip pre-check",
        )

    def test_degraded_counter_in_summary(self) -> None:
        """`degraded` counter must appear in tick summary и af_summary_set."""
        text = _read_script()
        # Декларация счётчика (re.MULTILINE — `^` = начало строки)
        self.assertRegex(
            text,
            r"(?m)^degraded=0",
            msg="degraded counter not initialized",
        )
        # Использование в summary
        self.assertRegex(
            text,
            r"degraded=\$\{degraded:-0\}",
            msg="degraded counter not referenced in tick summary",
        )
        # Итоговый summary ветвится на degraded > 0
        self.assertRegex(
            text,
            r'af_summary_set\s+degraded',
            msg="af_summary_set degraded branch missing — round must end with "
                "'DEGRADED' (not FAILURE) when pre-check fails",
        )

    def test_increment_writes_prometheus_textfile(self) -> None:
        """Run the e2e_target_unreachable_inc body against a temp file.

        Изолируем функцию: extract'им её определение из скрипта через awk
        (от `^e2e_target_unreachable_inc()` до следующего `^}` на col 1) и
        запускаем в standalone bash с stubbed `log()`. Это быстрее и
        надёжнее чем source всего скрипта (там ещё main() flow идёт после
        source).
        """
        if not shutil.which("bash"):
            self.skipTest("bash not available")
        text = _read_script()
        # Найти функцию и извлечь её тело до следующей `^}` строки.
        lines = text.splitlines()
        start_idx = None
        for i, line in enumerate(lines):
            if re.match(r"^e2e_target_unreachable_inc\s*\(\s*\)\s*\{", line):
                start_idx = i
                break
        if start_idx is None:
            self.skipTest("e2e_target_unreachable_inc function not found")
        end_idx = None
        for i in range(start_idx + 1, len(lines)):
            if lines[i].startswith("}"):
                end_idx = i + 1
                break
        if end_idx is None:
            self.skipTest("end of e2e_target_unreachable_inc not found")
        fn_body = "\n".join(lines[start_idx:end_idx])

        with tempfile.TemporaryDirectory() as tmp:
            prom_file = pathlib.Path(tmp) / "subdir" / "metrics.prom"
            stub = pathlib.Path(tmp) / "inc.sh"
            script_body = (
                "#!/usr/bin/env bash\n"
                "set +e\n"
                "log() { :; }\n"
                "export -f log\n"
                f'export E2E_TARGET_UNREACHABLE_FILE="{prom_file}"\n'
                'export DRY_RUN="false"\n'
                + fn_body
                + "\n"
                "e2e_target_unreachable_inc 10.1.1.21 deploy\n"
                "e2e_target_unreachable_inc 10.1.1.21 deploy\n"
                "e2e_target_unreachable_inc 10.1.1.249 build\n"
            )
            stub.write_text(script_body)
            stub.chmod(0o755)
            result = subprocess.run(
                ["bash", str(stub)],
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.assertEqual(
                result.returncode,
                0,
                msg=(
                    f"script failed (rc={result.returncode}): "
                    f"stdout={result.stdout!r} stderr={result.stderr!r}"
                ),
            )
            self.assertTrue(
                prom_file.exists(),
                msg=(
                    f"Prom file not written at {prom_file}. "
                    f"stderr={result.stderr!r}"
                ),
            )
            content = prom_file.read_text()
            self.assertIn("# TYPE e2e_target_unreachable_total counter", content)
            self.assertIn(
                'e2e_target_unreachable_total{target_host="10.1.1.21",phase="deploy"} 2',
                content,
                msg=f"counter not incremented twice:\n{content}",
            )
            self.assertIn(
                'e2e_target_unreachable_total{target_host="10.1.1.249",phase="build"} 1',
                content,
                msg=f"second host counter missing:\n{content}",
            )


class PreCheckBehaviorOnLiveHosts(unittest.TestCase):
    """Regression: at живой хост поведение скрипта НЕ должно меняться.

    Без полного live-теста (требует gh API) проверяем структурно:
    - skip_robot_pre_check=1 → pre-check SKIPPED, continue не срабатывает.
    - пустой E2E_PRECHECK_HOSTS → pre-check SKIPPED, безопасный путь.
    - degraded-counter НЕ увеличивается, если хосты живые.
    """

    def test_skip_robot_short_circuits(self) -> None:
        text = _read_script()
        # Проверяем что в коде есть условие skip
        self.assertRegex(
            text,
            r"if\s+\[\s+\"\$\{e2e_skip_robot_pre_check:-0\}\"\s*=\s*\"1\"\s*\]",
            msg="skip_robot_pre_check=1 must short-circuit the pre-check loop",
        )

    def test_empty_hosts_short_circuits(self) -> None:
        text = _read_script()
        # Проверяем что пустой E2E_PRECHECK_HOSTS → SKIPPED (без for loop)
        self.assertRegex(
            text,
            r'E2E_PRECHECK_HOSTS пуст',
            msg="empty E2E_PRECHECK_HOSTS must skip pre-check safely",
        )

    def test_errored_counter_not_incremented_on_degraded(self) -> None:
        """При degraded pre-check НЕ должно быть errored++ — иначе fail-streak
        watchdog зальёт Шифу ложными срабатываниями.
        """
        text = _read_script()
        # Найти блок pre-check в per-issue loop
        m = PRECHECK_BLOCK_RE.search(text)
        if not m:
            self.skipTest("pre-check block not found")
        # Следующие 60 строк после начала блока не должны содержать errored++
        start_line = text[: m.start()].count("\n") + 1
        snippet = "\n".join(text.splitlines()[start_line : start_line + 60])
        self.assertNotIn(
            "errored=$((errored+1))",
            snippet,
            msg=(
                "pre-check degraded branch must NOT call errored++ — "
                "это вызовет fail-streak watchdog. Должно быть только "
                "degraded++ и continue."
            ),
        )


if __name__ == "__main__":
    unittest.main()