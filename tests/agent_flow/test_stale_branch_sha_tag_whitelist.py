"""Regression tests for SHA-tag whitelist in stale_branch_check().

Контекст (ретро-план t_fdb19f7b, Phase 3, kanban t_23a51e4d):

PR #1547 (z-{agent}/1546-fix-voice-1544-dialogue-node-music-state) был
stale-by-70 коммитов при threshold=10 → guard блокировал e2e-ротацию.
Из 70 коммитов ~57 = SHA-tag noise от CI auto-tagger:

    ci: vision SHA tags → dev-<sha>
    ci: main SHA tags → dev-<sha>

Они идут автоматически, не меняют код, но rev-list --count их считает.
Раньше скрипт сравнивал raw `_behind` с threshold → false-positive stale.

РЕШЕНИЕ: после получения raw rev-list count, если он превышает threshold,
делаем второй запрос `git log --oneline origin/develop..HEAD` и считаем
real_behind после фильтра SHA-tag noise. Сравниваем с threshold уже
real_behind.

Контракт:

* `rev-list --count origin/develop..origin/<branch>` = 70, threshold=10
  → после whitelist: real_behind=13 (или меньше) → OK, guard возвращает 0.
* Если ВСЕ behind-коммиты — SHA-tag noise (real_behind=0) → OK.
* Если после фильтра остаются коммиты > threshold → BLOCKED.
* Если rev-list возвращает 0 → cheap-path → OK (без log-вызова).

Mock strategy: shim'ы git и gh в tmp bin/, PATH указывает на них, source'им
agent-flow-e2e-process.sh и вызываем stale_branch_check() напрямую через
bash subprocess. Тест изолирован: реальный REPO_DIR/tmp, реальный PATH, но
все внешние вызовы (git/gh) перехвачены shim'ами.
"""

from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"


# --------------------------------------------------------------------------- #
# Mock strategy: перехватываем git и gh через PATH-shim'ы.                     #
# --------------------------------------------------------------------------- #


GIT_SHIM_TEMPLATE = r"""#!/usr/bin/env bash
# test shim for git — see tests/agent_flow/test_stale_branch_sha_tag_whitelist.py
case "$1" in
  -C) shift 2 ;;
esac
subcommand="$1"; shift
case "$subcommand" in
  rev-list)
    # ищем --count и range аргумент.
    _range=""
    for arg in "$@"; do
      case "$arg" in
        --count) ;;
        *..*) _range="$arg" ;;
      esac
    done
    _tip="${_range%..*}"
    _dev="${_range#*..}"
    _behind=""
    if [ -n "${GIT_STATE:-}" ] && [ -f "${GIT_STATE}" ]; then
      _br="$(grep -E '^BRANCH_TIP_' "${GIT_STATE}" 2>/dev/null \
        | awk -F= -v t="$_tip" '$2==t {print $1}' | sed 's/^BRANCH_TIP_//' | head -n1)"
      if [ -n "$_br" ]; then
        _behind="$(grep -E "^STALE_BEHIND_${_br}=" "${GIT_STATE}" 2>/dev/null \
          | head -n1 | sed 's/^STALE_BEHIND_[^=]*=//')"
      fi
    fi
    if [ -z "$_behind" ] && [ -n "${STALE_BEHIND_DEFAULT:-}" ]; then
      _behind="${STALE_BEHIND_DEFAULT}"
    fi
    printf '%s' "${_behind}"
    ;;
  log)
    # Возвращаем содержимое GIT_LOG_FILE, если задано (иначе — пусто).
    if [ -n "${GIT_LOG_FILE:-}" ] && [ -f "${GIT_LOG_FILE}" ]; then
      cat "${GIT_LOG_FILE}"
    fi
    ;;
  rev-parse)
    _arg="${@: -1}"
    case "$_arg" in
      origin/develop|origin/main)
        printf '%s' "deadbeefdeadbeefdeadbeefdeadbeefdeadbeef"
        ;;
      origin/*)
        _br="${_arg#origin/}"
        if [ -n "${GIT_STATE:-}" ] && [ -f "${GIT_STATE}" ]; then
          _sha="$(grep -E "^BRANCH_TIP_${_br}=" "${GIT_STATE}" 2>/dev/null \
            | head -n1 | sed 's/^BRANCH_TIP_[^=]*=//')"
          if [ -n "$_sha" ]; then
            printf '%s' "$_sha"
            exit 0
          fi
        fi
        printf '%s' "feedfacefeedfacefeedfacefeedfacefeedface"
        ;;
      *) printf '%s' "0000000000000000000000000000000000000000" ;;
    esac
    ;;
  fetch|ls-remote|push|pull|show|worktree|rm|commit|prune|checkout|merge|branch)
    :
    ;;
  *) ;;
esac
"""

GH_SHIM_TEMPLATE = r"""#!/usr/bin/env bash
subcommand="$1"; shift || true
case "$subcommand" in
  issue)
    action="$1"; shift || true
    case "$action" in
      comment|edit|close|view)
        printf '%s\tissue_%s %s\n' "$(date -Iseconds)" "$action" "$*" >>"${GH_JOURNAL:-/dev/null}"
        exit 0
        ;;
      *) exit 0 ;;
    esac
    ;;
  api)
    if printf '%s' "$1" | grep -q '/comments'; then
      printf '0'
      exit 0
    fi
    if printf '%s' "$1" | grep -q 'rate_limit'; then
      printf '{"resources":{"core":{"remaining":5000},"graphql":{"remaining":5000}}}'
      exit 0
    fi
    exit 0
    ;;
  auth) exit 0 ;;
  *) exit 0 ;;
esac
"""


def _make_git_shim(bin_dir: Path) -> None:
    (bin_dir / "git").write_text(GIT_SHIM_TEMPLATE)
    (bin_dir / "git").chmod(0o755)


def _make_gh_shim(bin_dir: Path) -> None:
    (bin_dir / "gh").write_text(GH_SHIM_TEMPLATE)
    (bin_dir / "gh").chmod(0o755)


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def _run_stale_check(
    *,
    bin_dir: Path,
    repo_dir: Path,
    branch: str,
    pr_number: str,
    issue_number: str,
    threshold: int = 10,
    behind_raw: int = 70,
    log_lines: list[str] | None = None,
) -> subprocess.CompletedProcess[str]:
    """Извлекаем функцию stale_branch_check из скрипта и source'им в обёртке.

    Подход: скрипт agent-flow-e2e-process.sh слишком большой (3162 строки),
    source-импорт выполнит ВСЮ инициализацию (lock, MAINTENANCE gate, gh auth,
    основной loop). Для теста нужна только функция stale_branch_check. Извлекаем
    её через awk в обёрточный скрипт и source'им его.

    Это сохраняет 100% fidelity тестируемого кода (мы тестируем ровно ту
    функцию, что в production) и изолирует от побочных эффектов.
    """
    # Читаем script и вытаскиваем строки от stale_branch_check() { до
    # закрывающей } на той же глубине.
    content = SCRIPT.read_text()
    lines = content.split("\n")
    start_idx = None
    for i, ln in enumerate(lines):
        if ln.startswith("stale_branch_check()"):
            start_idx = i
            break
    if start_idx is None:
        raise RuntimeError("stale_branch_check() not found in script")

    # Ищем matching close brace.
    depth = 0
    end_idx = None
    for j in range(start_idx, len(lines)):
        depth += lines[j].count("{") - lines[j].count("}")
        if depth == 0 and j > start_idx:
            end_idx = j
            break
    if end_idx is None:
        raise RuntimeError("could not find end of stale_branch_check()")

    # Извлекаем функцию + минимальные хелперы (log и gh-rate-limit detection).
    # log() определена на строке 172, нам она нужна.
    log_line_idx = None
    for i, ln in enumerate(lines):
        if ln.startswith("log() {") or ln.startswith("log() {"):
            log_line_idx = i
            break

    fn_lines = lines[start_idx:end_idx + 1]
    wrapper_lines = [
        "#!/usr/bin/env bash",
        "# Auto-generated test wrapper for stale_branch_check()",
        "set -euo pipefail",
    ]
    if log_line_idx is not None:
        wrapper_lines.append(lines[log_line_idx])
    wrapper_lines.extend(fn_lines)
    wrapper_text = "\n".join(wrapper_lines) + "\n"

    wrapper_path = bin_dir / "_stale_branch_check.sh"
    wrapper_path.write_text(wrapper_text)
    wrapper_path.chmod(0o755)

    # State file для shim'ов: BRANCH_TIP_<br>=<sha> + STALE_BEHIND_<br>=<n>.
    state_file = bin_dir / ".git_state"
    sha = "a" * 40
    state_file.write_text(
        f"BRANCH_TIP_{branch}={sha}\nSTALE_BEHIND_{branch}={behind_raw}\n"
    )

    # Сохраняем log_lines в файл, чтобы shim прочитал через GIT_LOG_FILE.
    log_file = bin_dir / ".git_log"
    if log_lines is not None:
        log_file.write_text("\n".join(log_lines) + "\n")
    else:
        log_file.write_text("")

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GIT_STATE"] = str(state_file)
    env["GIT_LOG_FILE"] = str(log_file)
    env["GH_JOURNAL"] = str(bin_dir / ".gh_journal")
    env["REPO_DIR"] = str(repo_dir)
    env["FOUNDATION_BRANCH"] = "develop"
    env["E2E_STALE_BRANCH_THRESHOLD"] = str(threshold)
    env["DRY_RUN"] = "true"
    env["KANBAN_BOARD"] = "robbox"
    env["GH_REPO"] = "krikz/test-repo"
    env["LOG_PREFIX"] = "[test]"
    env["HOME"] = "/tmp"
    env.pop("STALE_BEHIND_DEFAULT", None)

    bash_cmd = (
        f'set +e; '
        f'source {shlex.quote(str(wrapper_path))}; '
        f'stale_branch_check {shlex.quote(branch)} '
        f'{shlex.quote(pr_number)} {shlex.quote(issue_number)}'
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
    """Создаёт bin/ с git+gh shim'ами."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _make_git_shim(bin_dir)
    _make_gh_shim(bin_dir)
    return bin_dir


@pytest.fixture()
def repo_dir(tmp_path: Path) -> Path:
    """Пустой фейковый REPO_DIR."""
    rd = tmp_path / "repo"
    rd.mkdir()
    return rd


# --------------------------------------------------------------------------- #
# Test cases
# --------------------------------------------------------------------------- #


def _sha_tag_noise(n: int) -> list[str]:
    """Генерит n SHA-tag noise onelines (по реальному develop pattern)."""
    lines = []
    for i in range(n):
        lines.append(
            f"a3777b1{i:02d} ci: main SHA tags → dev-3c9b44{i:02d} [skip ci]"
        )
        lines.append(
            f"ea8d9b0{i:02d} ci: vision SHA tags → dev-2bb777{i:02d} [skip ci]"
        )
    return lines[:n]


# --------------------------------------------------------------------------- #
# RED-phase tests: код ещё НЕ имеет whitelist. Эти тесты должны провалиться,
# показывая что whitelist нужен.
# --------------------------------------------------------------------------- #


def test_sha_tag_noise_majority_passes_guard_RED(
    shim_dir: Path, repo_dir: Path
) -> None:
    """RED: 67 SHA-tag noise + 3 real → raw=70 > 10.

    Без whitelist: raw=70 > 10 → BLOCKED → returncode=1.
    С whitelist: real=3 <= 10 → OK → returncode=0.

    Этот тест проходит ТОЛЬКО после имплементации whitelist.
    """
    real_commits = [f"feat: real change #{i}" for i in range(3)]
    noise = _sha_tag_noise(67)
    log_lines = noise + real_commits

    proc = _run_stale_check(
        bin_dir=shim_dir,
        repo_dir=repo_dir,
        branch="z-{agent}/1546-fix-voice-1544-dialogue-node-music-state",
        pr_number="1547",
        issue_number="1546",
        threshold=10,
        behind_raw=70,
        log_lines=log_lines,
    )

    assert proc.returncode == 0, (
        f"sha-tag whitelist должен пропустить PR (real=3 <= threshold=10). "
        f"Stderr: {proc.stderr!r}"
    )


def test_all_sha_tag_noise_passes_guard_RED(
    shim_dir: Path, repo_dir: Path
) -> None:
    """RED: все 70 — SHA-tag noise, 0 реальных. raw=70 > 10 → false-positive.

    Без whitelist: raw=70 > 10 → BLOCKED → returncode=1.
    С whitelist: real=0 <= 10 → OK → returncode=0.
    """
    log_lines = _sha_tag_noise(70)

    proc = _run_stale_check(
        bin_dir=shim_dir,
        repo_dir=repo_dir,
        branch="z-{agent}/1546-fix-voice-1544-dialogue-node-music-state",
        pr_number="1547",
        issue_number="1546",
        threshold=10,
        behind_raw=70,
        log_lines=log_lines,
    )

    assert proc.returncode == 0, (
        f"100% sha-tag noise → real=0 → должен быть OK. stderr: {proc.stderr!r}"
    )


# --------------------------------------------------------------------------- #
# GREEN-phase tests: после имплементации whitelist. Они должны проходить
# ОБА РАЗА — и сейчас (red), и после фикса (green). Контроль регрессии.
# --------------------------------------------------------------------------- #


def test_raw_below_threshold_skips_whitelist(
    shim_dir: Path, repo_dir: Path
) -> None:
    """Cheap-path: rev-list raw=5 <= threshold=10 → OK, whitelist не нужен.

    Этот тест проходит ВСЕГДА (red+green): rev-list даёт 5, guard сразу OK.
    """
    proc = _run_stale_check(
        bin_dir=shim_dir,
        repo_dir=repo_dir,
        branch="z-{agent}/1546-fix-voice",
        pr_number="1547",
        issue_number="1546",
        threshold=10,
        behind_raw=5,
        log_lines=None,
    )

    assert proc.returncode == 0, (
        f"raw=5 <= threshold → OK (cheap path). stderr: {proc.stderr!r}"
    )
    assert "OK" in proc.stderr, (
        f"ожидаем OK в логе; stderr: {proc.stderr!r}"
    )


def test_raw_above_threshold_real_above_threshold_blocks(
    shim_dir: Path, repo_dir: Path
) -> None:
    """Raw=80, после whitelist 60 noise + 20 real = real=20 > 10 → BLOCKED.

    Этот тест проходит ВСЕГДА (red+green): rev-list даёт 80 > 10, guard
    сразу BLOCKED (без whitelist). С whitelist: real=20 > 10 → BLOCKED.
    """
    real_commits = [f"feat: real change #{i}" for i in range(20)]
    noise = _sha_tag_noise(60)
    log_lines = noise + real_commits

    proc = _run_stale_check(
        bin_dir=shim_dir,
        repo_dir=repo_dir,
        branch="z-{agent}/1546-fix-voice",
        pr_number="1547",
        issue_number="1546",
        threshold=10,
        behind_raw=80,
        log_lines=log_lines,
    )

    assert proc.returncode == 1, (
        f"real=20 > 10 → должен быть BLOCKED. stderr: {proc.stderr!r}"
    )


def test_real_commits_dominate_blocks_GREEN(
    shim_dir: Path, repo_dir: Path
) -> None:
    """Реальные изменения доминируют: 5 noise + 30 real = real=30 > 10 → BLOCKED.

    Этот тест проходит ВСЕГДА (red+green): guard видит raw=35 > 10 → BLOCKED.
    С whitelist: real=30 > 10 → BLOCKED. Семантика та же.
    """
    real_commits = [f"feat: real change #{i}" for i in range(30)]
    noise = _sha_tag_noise(5)
    log_lines = noise + real_commits

    proc = _run_stale_check(
        bin_dir=shim_dir,
        repo_dir=repo_dir,
        branch="z-{agent}/1546-fix-voice",
        pr_number="1547",
        issue_number="1546",
        threshold=10,
        behind_raw=35,
        log_lines=log_lines,
    )

    assert proc.returncode == 1, (
        f"real=30 > 10 → BLOCKED. stderr: {proc.stderr!r}"
    )
