"""Молчание акта не должно маскироваться под фичу (run 44963710006).

Акт 1 ночного марафона: 10/10 шагов ``cycle_unconfirmed_after_accept``, робот не
закрыл ни одного цикла. Харнесс пошагово честно ставил ``mark_fail_kind
no_reaction``, но провал агрегатного GATE-1 безусловно звал ``mark_fail_kind
feature``, а feature не понижается. Маркер акта выходил ``E2E_FEATURE_FAIL``
вместо ``E2E_NO_REACTION``, и гард ``STOP_ON_FOUNDATION_FAIL``
(``scripts/e2e/run_night_marathon.sh``) марафон не останавливал: он верит
маркеру, а маркер врал «робот ответил, но не сделал».

Правило: ``feature`` из GATE-1 честен, только если в акте есть хотя бы один
подтверждённый цикл. Ноль циклов -- провал агрегата есть следствие молчания,
вид не повышается, маркер акта остаётся ``E2E_NO_REACTION``.

Гоняются настоящие ``mark_fail_kind`` и блок выбора финального маркера из
``e2e_voice_test.sh``; развилка GATE-1 проверяется по тексту скрипта, потому что
``check_gate1_aggregate`` ходит на робота по SSH и в юните не исполняется.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_e2e_silence_not_feature.py -v --no-cov
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / "scripts" / "ci" / "e2e" / "e2e_voice_test.sh"
MARATHON = REPO_ROOT / "scripts" / "e2e" / "run_night_marathon.sh"
SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")

needs_bash = pytest.mark.skipif(shutil.which("bash") is None, reason="нужен bash")
BASH = os.environ.get("BASH_EXE") or shutil.which("bash") or "bash"


def extract_function(name):
    start = next(i for i, ln in enumerate(SCRIPT_LINES) if ln.startswith(name + "() {"))
    heredoc = None
    for i in range(start + 1, len(SCRIPT_LINES)):
        line = SCRIPT_LINES[i]
        if heredoc is not None:
            if line.strip() == heredoc:
                heredoc = None
            continue
        found = re.search(r"<<-?'([A-Za-z_][A-Za-z0-9_]*)'", line)
        if found:
            heredoc = found.group(1)
        elif line == "}":
            return "\n".join(SCRIPT_LINES[start : i + 1])
    raise AssertionError(name)


def final_marker_block():
    """Блок выбора финального маркера: ``case "${E2E_FAIL_KIND:-no_reaction}" in`` .. ``esac``."""
    start = next(
        i
        for i, ln in enumerate(SCRIPT_LINES)
        if 'case "${E2E_FAIL_KIND:-no_reaction}" in' in ln
    )
    end = next(
        i
        for i in range(start + 1, len(SCRIPT_LINES))
        if SCRIPT_LINES[i].strip() == "esac"
    )
    return "\n".join(SCRIPT_LINES[start : end + 1])


def run(body):
    script = "\n".join(
        [
            "set -u",
            "E2E_FAIL_KIND=''",
            "log() { :; }",
            extract_function("mark_fail_kind"),
            body,
            final_marker_block(),
            'echo "$fail_marker"',
            'echo "FAIL_KIND=${E2E_FAIL_KIND}"',
        ]
    )
    p = subprocess.run(
        [BASH, "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60
    )
    return (
        p.stdout.decode("utf-8", "replace"),
        p.stderr.decode("utf-8", "replace"),
        p.returncode,
    )


@needs_bash
class TestSilenceIsNotAFeature:
    def test_unconfirmed_cycles_keep_no_reaction(self):
        """10/10 cycle_unconfirmed_after_accept и ноль циклов: маркер NO_REACTION, не FEATURE_FAIL."""
        out, err, rc = run(
            "\n".join(
                [
                    "E2E_CONFIRMED_CYCLES=0",
                    "for _ in 1 2 3 4 5 6 7 8 9 10; do mark_fail_kind no_reaction; done",
                    'if [ "$E2E_CONFIRMED_CYCLES" -gt 0 ]; then mark_fail_kind feature; fi',
                ]
            )
        )
        assert rc == 0, (out, err)
        assert "E2E_NO_REACTION" in out, (out, err)
        assert "E2E_FEATURE_FAIL" not in out
        assert "FAIL_KIND=no_reaction" in out

    def test_one_confirmed_cycle_makes_gate1_a_feature(self):
        """Хоть один подтверждённый цикл: провал GATE-1 честно становится feature."""
        out, err, rc = run(
            "\n".join(
                [
                    "E2E_CONFIRMED_CYCLES=1",
                    "mark_fail_kind no_reaction",
                    'if [ "$E2E_CONFIRMED_CYCLES" -gt 0 ]; then mark_fail_kind feature; fi',
                ]
            )
        )
        assert rc == 0, (out, err)
        assert "E2E_FEATURE_FAIL" in out, (out, err)
        assert "FAIL_KIND=feature" in out

    def test_feature_still_never_downgrades(self):
        """Контракт приоритетов не сломан: feature не понижается поздним no_reaction."""
        out, err, rc = run("mark_fail_kind feature; mark_fail_kind no_reaction")
        assert rc == 0, (out, err)
        assert "FAIL_KIND=feature" in out and "E2E_FEATURE_FAIL" in out, (out, err)

    def test_silent_ok_rc_zero_does_not_count_as_a_cycle(self):
        """silent_ok даёт rc=0 без речи: такой шаг не подтверждённый цикл и не открывает feature."""
        out, err, rc = run(
            "\n".join(
                [
                    "E2E_CONFIRMED_CYCLES=0",
                    "for _ in 1 2 3 4 5 6 7 8 9 10; do mark_fail_kind no_reaction; done",
                    # Ход завершён без речи: rc=0, но робот не ответил (ветка silent_ok).
                    "rc=0; _silent_ok=1",
                    'if [ "$rc" = "0" ] && [ "${_silent_ok:-0}" != "1" ]; then',
                    "    E2E_CONFIRMED_CYCLES=$((${E2E_CONFIRMED_CYCLES:-0} + 1))",
                    "fi",
                    'if [ "${E2E_CONFIRMED_CYCLES:-0}" -gt 0 ]; then mark_fail_kind feature; fi',
                    'echo "CYCLES=${E2E_CONFIRMED_CYCLES}"',
                ]
            )
        )
        assert rc == 0, (out, err)
        assert "CYCLES=0" in out, (out, err)
        assert "E2E_NO_REACTION" in out and "E2E_FEATURE_FAIL" not in out, (out, err)
        assert "FAIL_KIND=no_reaction" in out


def test_gate1_fail_is_guarded_by_confirmed_cycles():
    """Развилка в самом скрипте: feature из GATE-1 стоит внутри проверки счётчика циклов."""
    text = "\n".join(SCRIPT_LINES)
    guard = re.search(
        r'if \[ "\$\{E2E_CONFIRMED_CYCLES:-0\}" -gt 0 \]; then\n'
        r"\s+mark_fail_kind feature\n"
        r"(?:.*\n){0,4}?"
        r"\s+echo \"E2E_GATE1_FAIL\"\n"
        r"\s+else\n"
        r"(?:.*\n){0,4}?"
        r"\s+echo \"E2E_GATE1_FAIL_NO_CYCLE\"",
        text,
    )
    assert (
        guard
    ), "mark_fail_kind feature на провале GATE-1 обязан стоять под проверкой E2E_CONFIRMED_CYCLES"


def test_confirmed_cycle_is_counted_where_both_judges_agree():
    """Счётчик растёт в единственной точке, где оба судьи признали цикл (rc=0)."""
    text = "\n".join(SCRIPT_LINES)
    assert "E2E_CONFIRMED_CYCLES=0" in text
    counted = re.search(
        r'log "STEP \$\{label\}: ✅ ПОЛНЫЙ ЦИКЛ \(акцепт \+ LLM \+ TTS\)"\n'
        r"\s+reaction=1\n"
        r"(?:[^\n]*\n){0,8}?"
        r'\s+if \[ "\$\{_silent_ok:-0\}" != "1" \]; then\n'
        r"\s+E2E_CONFIRMED_CYCLES=\$\(\(\$\{E2E_CONFIRMED_CYCLES:-0\} \+ 1\)\)",
        text,
    )
    assert (
        counted
    ), "инкремент счётчика обязан стоять сразу за логом подтверждённого цикла"


def test_marathon_guard_still_stops_on_no_reaction():
    """Гард марафона по-прежнему останавливает акт 1-2 по E2E_NO_REACTION -- фикс на это и опирается.

    С #5076 решение принимает ``marathon_foundation_decision``: стоп на всё,
    кроме настоящего ``E2E_FEATURE_FAIL``. ``E2E_NO_REACTION`` туда входит.
    """
    marathon = MARATHON.read_text(encoding="utf-8")
    assert 'marathon_foundation_decision "$act_log"' in marathon
    assert "останавливаю марафон" in marathon
