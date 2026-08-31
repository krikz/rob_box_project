"""Regression tests for pr_has_functional_files() and pr_has_process_changes()
in agent-flow-merge-gate.sh.

Контекст (ретро 31.08 t_04371252, PR #1753):

PR #1753 (`z-agent/t_1d0426e3-blocked-watchdog`) был открыт на ветке, которая
УЖЕ была влита в develop через PR #1704 (commit t_a2521b07). PR #1753 содержит
23 файла: 22 в `scripts/agent_flow/*` + 1 в `tests/agent_flow/*`. +754/-15 строк.

Проблема: старая `pr_has_functional_files()` СЧИТАЛА `scripts/agent_flow/*` за
ci-only (whitelist включал `.github/`, `scripts/agent_flow/`, `docs/`). Для PR,
состоящего ТОЛЬКО из `scripts/agent_flow/*` (типичный аддитивный фикс поверх
влитой процессной ветки), guard бы пропустил — а это явный stale-branch reuse.

Фикс: scripts/agent_flow/* и tests/agent_flow/* — это ПРОЦЕССНЫЕ скрипты, не
ci-only. Любой такой файл = функциональное изменение → блокируем stale-reuse.

Тесты (D1-D6):
- D1: PR #1753 file list (23 файла scripts/agent_flow/* + 1 tests/agent_flow/*)
       → functional=1 (regression для бага).
- D2: PR только в .github/ + docs/ (ci-only) → functional=0.
- D3: PR в scripts/agent_flow/ (process scripts) → functional=1 (новая логика).
- D4: PR в src/ (robot code) → functional=1 (уже работало).
- D5: PR пустой → functional=0 (не dead-content).
- D6: PR в tests/agent_flow/ (process tests) → functional=1 (новая логика).

Для pr_has_process_changes():
- P1: PR #1753 → 1 (process changes detected).
- P2: PR только docs/ → 0 (no process changes).
- P3: PR пустой → 0.

Mock strategy: как в test_stale_branch_sha_tag_whitelist.py — вытаскиваем
функцию через awk из скрипта, source'им в обёртке с PATH shim'ом для `gh`,
который возвращает file list из JSON fixture. Изолированно: реальный bin_dir,
реальный PATH с подменой.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-merge-gate.sh"


# --------------------------------------------------------------------------- #
# Mock strategy: gh shim, который отдаёт file list из FIXTURE_FILE.
# --------------------------------------------------------------------------- #


GH_SHIM_TEMPLATE = r"""#!/usr/bin/env bash
# test shim for gh — see tests/agent_flow/test_merge_gate_stale_branch_process_scripts.py
subcommand="$1"; shift || true
case "$subcommand" in
  pr)
    action="$1"; shift || true
    case "$action" in
      view)
        # gh pr view N --repo R --json files --jq '[.files[].path]'
        # Detect "files" в аргументах --json.
        _want_files=0
        for arg in "$@"; do
          case "$arg" in
            --json)
              _want_files=1
              ;;
            files)
              if [ "$_want_files" = "1" ]; then
                if [ -n "${FIXTURE_FILE:-}" ] && [ -f "${FIXTURE_FILE}" ]; then
                  cat "${FIXTURE_FILE}"
                else
                  printf '[]'
                fi
                exit 0
              fi
              ;;
          esac
        done
        # Если не files — пустой ответ.
        exit 0
        ;;
      *) exit 0 ;;
    esac
    ;;
  *) exit 0 ;;
esac
"""


def _make_gh_shim(bin_dir: Path) -> None:
    (bin_dir / "gh").write_text(GH_SHIM_TEMPLATE)
    (bin_dir / "gh").chmod(0o755)


# --------------------------------------------------------------------------- #
# Helpers: extract functions from script + run in isolated env.
# --------------------------------------------------------------------------- #


def _extract_functions(*names: str) -> str:
    """Извлекает функции с указанными именами из merge-gate.sh.

    Подход как в test_stale_branch_sha_tag_whitelist.py — awk по скобкам.
    Возвращает текст функций, готовый для source'а.
    """
    content = SCRIPT.read_text()
    lines = content.split("\n")
    result_lines: list[str] = []
    for name in names:
        start_idx = None
        for i, ln in enumerate(lines):
            if ln.startswith(f"{name}()"):
                start_idx = i
                break
        if start_idx is None:
            raise RuntimeError(f"function {name}() not found in script")
        depth = 0
        end_idx = None
        for j in range(start_idx, len(lines)):
            depth += lines[j].count("{") - lines[j].count("}")
            if depth == 0 and j > start_idx:
                end_idx = j
                break
        if end_idx is None:
            raise RuntimeError(f"could not find end of {name}()")
        result_lines.extend(lines[start_idx:end_idx + 1])
        result_lines.append("")
    return "\n".join(result_lines) + "\n"


def _run_func(
    *,
    bin_dir: Path,
    pr_num: str,
    fixture_file: Path | None,
) -> subprocess.CompletedProcess[str]:
    """Запускает pr_has_functional_files + pr_has_process_changes на pr_num."""
    fn_text = _extract_functions("pr_has_functional_files", "pr_has_process_changes")
    wrapper = bin_dir / "_funcs.sh"
    wrapper.write_text(fn_text)
    wrapper.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/test-repo"
    if fixture_file is not None:
        env["FIXTURE_FILE"] = str(fixture_file)

    bash_cmd = (
        f"source {shlex.quote(str(wrapper))}; "
        f'printf "F=%s\\n" "$(pr_has_functional_files {shlex.quote(pr_num)})"; '
        f'printf "P=%s\\n" "$(pr_has_process_changes {shlex.quote(pr_num)})"; '
    )
    return subprocess.run(
        ["bash", "-c", bash_cmd],
        capture_output=True,
        text=True,
        env=env,
        timeout=15,
    )


@pytest.fixture()
def shim_dir(tmp_path: Path) -> Path:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _make_gh_shim(bin_dir)
    return bin_dir


def _write_fixture(bin_dir: Path, files: list[str]) -> Path:
    fx = bin_dir / ".fixture_files.json"
    fx.write_text(json.dumps(files))
    return fx


# --------------------------------------------------------------------------- #
# Test data: PR #1753 file list (extracted via gh pr view 1753 --json files).
# --------------------------------------------------------------------------- #

PR_1753_FILES = [
    "scripts/agent_flow/README.md",
    "scripts/agent_flow/agent-flow-blocked-watchdog.sh",
    "scripts/agent_flow/agent-flow-cleanup-249.sh",
    "scripts/agent_flow/agent-flow-completion-check.sh",
    "scripts/agent_flow/agent-flow-deploy-sweep.sh",
    "scripts/agent_flow/agent-flow-e2e-drift-watchdog.sh",
    "scripts/agent_flow/agent-flow-e2e-process-launcher.sh",
    "scripts/agent_flow/agent-flow-e2e-process.sh",
    "scripts/agent_flow/agent-flow-handoff.sh",
    "scripts/agent_flow/agent-flow-merge-gate.sh",
    "scripts/agent_flow/agent-flow-post-merge-build.sh",
    "scripts/agent_flow/agent-flow-rotation-watchdog.sh",
    "scripts/agent_flow/agent-flow-triage.sh",
    "scripts/agent_flow/agent-flow-unlabeled-sweep.sh",
    "scripts/agent_flow/agents_sleep.sh",
    "scripts/agent_flow/cross-task-archive-sweeper.sh",
    "scripts/agent_flow/hermes_github.sh",
    "scripts/agent_flow/install.sh",
    "scripts/agent_flow/lib_cron_env.sh",
    "scripts/agent_flow/patch_lib_cron_env.sh",
    "scripts/agent_flow/push-via-gh-api.sh",
    "scripts/agent_flow/watchdog.sh",
    "tests/agent_flow/test_lib_cron_env.sh",
]


# --------------------------------------------------------------------------- #
# Test cases
# --------------------------------------------------------------------------- #


def test_pr_1753_process_scripts_reuse_is_functional(shim_dir: Path) -> None:
    """D1: PR #1753 — 23 файла в scripts/agent_flow/* + 1 в tests/agent_flow/*.

    Functional=1 (новый guard), ProcessChanges=1 (есть scripts/agent_flow/*).
    Это REGRESSION для бага, где old guard считал scripts/agent_flow/* за ci-only.
    """
    fx = _write_fixture(shim_dir, PR_1753_FILES)
    proc = _run_func(bin_dir=shim_dir, pr_num="1753", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=1" in out, (
        f"PR #1753 (23 process scripts + 1 process test) должен быть "
        f"FUNCTIONAL=1 (блокируем stale-branch reuse). Got: {out!r}"
    )
    assert "P=1" in out, (
        f"PR #1753 меняет scripts/agent_flow/* — ProcessChanges должен быть 1. "
        f"Got: {out!r}"
    )


def test_ci_only_pr_is_not_functional(shim_dir: Path) -> None:
    """D2: PR только в .github/ + docs/ — это аддитивный ci-only (НЕ блокируем).

    Functional=0, ProcessChanges=0.
    """
    fx = _write_fixture(shim_dir, [
        ".github/workflows/test.yml",
        ".github/actions/setup/action.yml",
        "docs/adr/0001-test.md",
        "docs/design/agent-flow.md",
    ])
    proc = _run_func(bin_dir=shim_dir, pr_num="9999", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=0" in out, (
        f"PR только в .github/ + docs/ должен быть ci-only (F=0). Got: {out!r}"
    )
    assert "P=0" in out, (
        f"PR без scripts/agent_flow/* и tests/agent_flow/* должен иметь P=0. "
        f"Got: {out!r}"
    )


def test_only_process_scripts_is_functional_RED(shim_dir: Path) -> None:
    """D3: PR только в scripts/agent_flow/ — process scripts = functional.

    Без фикса старый guard считал scripts/agent_flow/* за ci-only и возвращал 0.
    После фикса — 1. Это ГЛАВНЫЙ RED-кейс, который ловит регрессию.
    """
    fx = _write_fixture(shim_dir, [
        "scripts/agent_flow/test.sh",
        "scripts/agent_flow/lib_x.sh",
    ])
    proc = _run_func(bin_dir=shim_dir, pr_num="8888", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=1" in out, (
        f"PR только в scripts/agent_flow/ ДОЛЖЕН быть functional=1 "
        f"(процессные скрипты, не ci-only). Это ловит регрессию. Got: {out!r}"
    )
    assert "P=1" in out, (
        f"PR в scripts/agent_flow/ — ProcessChanges=1. Got: {out!r}"
    )


def test_robot_code_pr_is_functional(shim_dir: Path) -> None:
    """D4: PR в src/ — робочий код, всегда функциональный.

    Functional=1, ProcessChanges=0.
    """
    fx = _write_fixture(shim_dir, [
        "src/rob_box_voice/voice_node.py",
        "src/rob_box_teleop/joystick.py",
    ])
    proc = _run_func(bin_dir=shim_dir, pr_num="7777", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=1" in out, f"PR в src/ — функциональный (F=1). Got: {out!r}"
    assert "P=0" in out, f"PR в src/ — не process changes (P=0). Got: {out!r}"


def test_empty_pr_is_not_functional(shim_dir: Path) -> None:
    """D5: пустой PR (0 файлов) — не функциональный (L3 acceptance)."""
    fx = _write_fixture(shim_dir, [])
    proc = _run_func(bin_dir=shim_dir, pr_num="6666", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=0" in out, f"Пустой PR — не functional (F=0). Got: {out!r}"
    assert "P=0" in out, f"Пустой PR — не process (P=0). Got: {out!r}"


def test_only_process_tests_is_functional(shim_dir: Path) -> None:
    """D6: PR в tests/agent_flow/ — process tests, тоже функциональный."""
    fx = _write_fixture(shim_dir, [
        "tests/agent_flow/test_new.sh",
        "tests/agent_flow/test_x.py",
    ])
    proc = _run_func(bin_dir=shim_dir, pr_num="5555", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=1" in out, (
        f"PR в tests/agent_flow/ ДОЛЖЕН быть functional=1. Got: {out!r}"
    )
    assert "P=1" in out, f"PR в tests/agent_flow/ — ProcessChanges=1. Got: {out!r}"


def test_mixed_ci_only_and_process_is_functional(shim_dir: Path) -> None:
    """D7: смесь .github/workflows/* + scripts/agent_flow/* — функциональный.

    Functional=1 (есть process), ProcessChanges=1.
    Это кейс «аддитивный docs/ci PR + процессный фикс в том же PR» —
    блокируем как stale-reuse.
    """
    fx = _write_fixture(shim_dir, [
        ".github/workflows/ci.yml",
        "scripts/agent_flow/install.sh",
    ])
    proc = _run_func(bin_dir=shim_dir, pr_num="4444", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=1" in out, (
        f"Смешанный PR с process changes — functional=1. Got: {out!r}"
    )
    assert "P=1" in out, f"Смешанный PR — ProcessChanges=1. Got: {out!r}"


def test_dockerfile_only_is_functional(shim_dir: Path) -> None:
    """D8: PR только в docker/ — это docker code, не ci-only. Functional=1."""
    fx = _write_fixture(shim_dir, [
        "docker/main/Dockerfile",
        "docker/voice/Dockerfile",
    ])
    proc = _run_func(bin_dir=shim_dir, pr_num="3333", fixture_file=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"
    out = proc.stdout
    assert "F=1" in out, f"PR в docker/ — functional=1. Got: {out!r}"
    assert "P=0" in out, f"PR в docker/ (не scripts/agent_flow/) — P=0. Got: {out!r}"


def test_label_constant_present() -> None:
    """Защита от регрессии: константа STALE_BRANCH_REUSE_LABEL должна быть в скрипте.

    Без неё downstream-фильтры (e2e-process, clean-pr-sweep) не смогут skip'нуть
    stale-branch-reuse PR. Это страховка от переноса константы.
    """
    content = SCRIPT.read_text()
    assert "STALE_BRANCH_REUSE_LABEL=" in content, (
        "STALE_BRANCH_REUSE_LABEL константа отсутствует в agent-flow-merge-gate.sh"
    )
    assert 'stale-branch-reuse' in content, (
        "Имя метки 'stale-branch-reuse' должно быть в скрипте (default value)"
    )


def test_label_set_in_stale_branch_scan_all() -> None:
    """Защита: stale_branch_scan_all ДОЛЖЕН ставить STALE_BRANCH_REUSE_LABEL.

    Это контракт: при обнаружении stale-branch reuse с функциональным диффом —
    merge-gate ставит явную метку для downstream-фильтров.
    """
    content = SCRIPT.read_text()
    # Проверяем что в функции stale_branch_scan_all есть whoami_add_label
    # с STALE_BRANCH_REUSE_LABEL.
    fn_text = _extract_functions("stale_branch_scan_all")
    assert "STALE_BRANCH_REUSE_LABEL" in fn_text, (
        "stale_branch_scan_all() должна использовать STALE_BRANCH_REUSE_LABEL "
        "при обнаружении stale-branch reuse с функциональным диффом"
    )
    assert "whoami_add_label" in fn_text, (
        "stale_branch_scan_all() должна использовать whoami_add_label для "
        "атомарной установки метки (с whoami-comment)"
    )