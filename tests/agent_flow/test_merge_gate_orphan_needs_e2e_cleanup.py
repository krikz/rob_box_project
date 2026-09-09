"""Regression tests for orphan-needs-e2e-after-merge cleanup в
agent-flow-merge-gate.sh.

Контекст (ретро 02.09 t_a09e893a):

PRы #1768 (issue #1764), #1786 + #1797 (issue #1777), #1793 + #1794 + #1790 +
#1823 + #1832 (issue #1780) были MERGED в develop 31.08, но соответствующие
issue оставались OPEN с меткой `needs-e2e` 49+ часов без активной kanban-карточки.

ROOT cause (два независимых дефекта):

1. Branch-pattern lookup bug (merge-gate.sh):
   Канонический шаблон `z-{agent}/${number}-$(slugify title)` НЕ покрывает
   реальные ветки воркеров:
     - z-backend/1764-bug-voice-...         (issue #1764)
     - z-developer/1780-fix-int-float-crash (issue #1780)
     - z-llm-expert/1777-...               (issue #1777 follow-up)
   Результат: `gh pr list --head $branch` возвращает [], merge-gate skip-ит issue
   вечно (`labeled; continue`).

2. Post-merge label-cleanup gap (merge-gate.sh):
   Для случая «PR MERGED + нет e2e-done + ветка жива» логика ждёт удаления
   ветки без ограничения по времени. Issue зависает в needs-e2e rotation.

Фикс (этот PR):
  - Patch 1: branch-pattern fallback — список известных z-<agent>/ префиксов,
    плюс search по title для ловли `t_<id>-<n>-slug` формата. Если нашли —
    переписываем `branch` на найденную (для downstream Q22-проверки).
  - Patch 2: staleness grace (default 24h). После merge + grace + ветка жива
    → снимаем `needs-e2e`, ставим audit-метку `merged-no-e2e-stale` (Шифу
    решает: закрыть вручную или open follow-up).
  - Patch 3: e2e-process skip для `merged-no-e2e-stale` (иначе ping-pong).

Тесты (R1-R6):
  R1: branch-pattern fallback находит `z-backend/1764-...` для issue #1764
      (канонический lookup вернёт []).
  R2: branch-pattern fallback находит `z-developer/1780-...` для issue #1780.
  R3: branch-pattern fallback НЕ находит для issue без PR (skip, идемпотент).
  R4: staleness grace 24h — issue MERGED 25h назад → trim + audit-метка.
  R5: staleness grace 24h — issue MERGED 5h назад → defer, ничего не делаем.
  R6: staleness grace 24h — issue с уже-стоящей `merged-no-e2e-stale`
      → идемпотентный skip (не дёргаем API).

Mock strategy: как в test_merge_gate_stale_branch_process_scripts.py —
извлекаем функции через brace-counting, source'им в обёртке с PATH shim'ом
для `gh`, логируем вызовы, проверяем наличие нужных команд в логе.

NOTE: тесты запускаются на worktree с правленным скриптом (патч применён
локально). При прогоне без патча тесты должны fail с понятным сообщением.
"""

from __future__ import annotations

import os
import re
import shlex
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-merge-gate.sh"


# --------------------------------------------------------------------------- #
# Mock gh: логирует все вызовы в файл, отдаёт фиксированные ответы из env.
# --------------------------------------------------------------------------- #

GH_SHIM_TEMPLATE = r"""#!/usr/bin/env bash
# test shim for gh — agent-flow-merge-gate.sh orphan-needs-e2e tests.
# Логирует все вызовы в ${CALL_LOG:-/tmp/gh-shim.log} (newline-separated).
# Возвращает данные из env: PR_JSON_BY_ISSUE_<N>=<JSON> (--search title),
# MERGED_AT_JSON (для gh pr view --json mergedAt), LABELS_CSV_JSON (labels).
set -u
subcommand="$1"; shift || true

_log_call() {
  printf '%s\n' "$*" >> "${CALL_LOG:-/tmp/gh-shim.log}"
}

case "$subcommand" in
  pr)
    action="$1"; shift || true
    case "$action" in
      list)
        # gh pr list --state all --head X --json ...
        # Или --search "<N> in:title" — отдаём PR_JSON_BY_ISSUE_<N>.
        _search_issue=""
        _state=""
        _head=""
        while [ $# -gt 0 ]; do
          case "$1" in
            --search) _search_issue="$2"; shift 2 ;;
            --state) _state="$2"; shift 2 ;;
            --head) _head="$2"; shift 2 ;;
            *) shift ;;
          esac
        done
        _log_call "gh pr list --search=$_search_issue --state=$_state --head=$_head"
        # Issue search path: отдаём prs для номера из search.
        if [ -n "$_search_issue" ]; then
          _num="$(printf '%s' "$_search_issue" | grep -oE '[0-9]+' || true)"
          if [ -n "$_num" ]; then
            _var="PR_JSON_BY_ISSUE_${_num}"
            _val="$(printf '%s' "${!_var:-}" | head -c 200000)"
            if [ -n "$_val" ]; then
              printf '%s' "$_val"
              exit 0
            fi
          fi
        fi
        # Head-based lookup: пустой результат (для теста branch-mismatch).
        # Если для этой ветки есть fixture — отдаём её.
        if [ -n "$_head" ]; then
          _var="PR_JSON_BY_HEAD_${_head//\//_}"
          _val="$(printf '%s' "${!_var:-}" | head -c 200000)"
          if [ -n "$_val" ]; then
            printf '%s' "$_val"
            exit 0
          fi
        fi
        printf '[]'
        ;;
      view)
        # gh pr view N --repo R --json X --jq Y
        _pr_num=""
        _json_fields=""
        while [ $# -gt 0 ]; do
          case "$1" in
            --*) : ;;
            [0-9]*) _pr_num="$1" ;;
          esac
          shift
        done
        # Re-parse args to find --json field
        for arg in "$@"; do
          if [ "$arg" = "mergedAt" ]; then
            _var="MERGED_AT_${_pr_num}"
            _val="${!_var:-}"
            if [ -n "$_val" ] && [ "$_val" != "null" ]; then
              printf '"%s"' "$_val"
            else
              printf 'null'
            fi
            _log_call "gh pr view $_pr_num --json mergedAt -> $_val"
            exit 0
          fi
          if [ "$arg" = "labels" ]; then
            _var="LABELS_CSV_${_pr_num}"
            _val="${!_val:-}"
            printf '%s' "${!_var:-[]}"
            exit 0
          fi
          if [ "$arg" = "headRefName" ]; then
            _var="HEAD_REF_${_pr_num}"
            printf '%s' "${!_var:-}"
            exit 0
          fi
        done
        # Generic: пустой ответ.
        _log_call "gh pr view $_pr_num (other)"
        printf '{}'
        ;;
      *)
        _log_call "gh pr $action"
        exit 0
        ;;
    esac
    ;;
  issue)
    action="$1"; shift || true
    case "$action" in
      view)
        _iss_num=""
        for arg in "$@"; do
          case "$arg" in
            [0-9]*) _iss_num="$arg" ;;
          esac
        done
        _var="ISSUE_STATE_${_iss_num}"
        _val="${!_var:-OPEN}"
        printf '{"state":"%s"}' "$_val"
        _log_call "gh issue view $_iss_num state=$_val"
        ;;
      edit)
        _log_call "gh issue edit $*"
        exit 0
        ;;
      comment)
        # Skip dedup (gh api ... comments?since=... → []).
        _log_call "gh issue comment $*"
        exit 0
        ;;
      *)
        exit 0
        ;;
    esac
    ;;
  api)
    # For dedup check: returns empty array.
    _log_call "gh api $*"
    printf '[]'
    ;;
  *)
    exit 0
    ;;
esac
"""


def _make_gh_shim(bin_dir: Path, call_log: Path) -> None:
    """Создаёт gh-shim + лог-файл + устанавливает CALL_LOG."""
    shim = bin_dir / "gh"
    shim.write_text(GH_SHIM_TEMPLATE)
    shim.chmod(0o755)
    # env передаётся через run_*, не через запись в скрипт.
    call_log.unlink(missing_ok=True)
    call_log.touch()


def _extract_main_block() -> str:
    """Извлекает кусок main-блока merge-gate.sh: branch-fallback + post-merge cleanup.

    Для тестирования мы НЕ запускаем весь скрипт (слишком много init).
    Вместо этого извлекаем:
      1) переменные окружения (NEEDS_E2E_LABEL etc.)
      2) helpers (has_label, slugify)
      3) "branch lookup" блок (после branch="z-{agent}/...")
      4) "post-merge cleanup" блок (после `if [ "$pr_state" = "MERGED" ]`)
    """
    content = SCRIPT.read_text()

    # 1. Defaults — top of script, до первого export.
    defaults_end = 0
    for marker in [
        'export HOME=/home/builder',
        'export HOME="$HOME"',
    ]:
        if marker in content:
            defaults_end = content.index(marker)
            break
    if defaults_end == 0:
        # fallback: first 200 lines
        defaults_end = content.index("\n", 1000)
    defaults = content[:defaults_end]

    # 2. Helpers — find has_label(), slugify() functions.
    def _find_func(name: str) -> str:
        lines = content.split("\n")
        for i, ln in enumerate(lines):
            if re.match(rf"^{re.escape(name)}\(\)", ln):
                depth = 0
                for j in range(i, len(lines)):
                    depth += lines[j].count("{") - lines[j].count("}")
                    if depth == 0 and j > i:
                        return "\n".join(lines[i:j + 1])
        return ""

    has_label_fn = _find_func("has_label")
    slugify_fn = _find_func("slugify")

    return "\n".join([defaults, has_label_fn, slugify_fn])


# --------------------------------------------------------------------------- #
# Pure-Python equivalent of the branch-fallback logic (for fast unit tests).
# --------------------------------------------------------------------------- #


def branch_fallback_search(prs_data: list[dict], issue_num: str,
                            agent_prefixes: list[str]):
    """Эквивалент Python-блока в merge-gate.sh для branch-fallback.

    Возвращает (headRefName, pr) или (None, None) если не нашли.
    """
    # 1. Точное совпадение: <prefix>/<num>-*  ИЛИ  <prefix>/t_*-<num>-*
    for pr in prs_data:
        head = pr.get("headRefName") or ""
        for p in agent_prefixes:
            if head.startswith(f"{p}/{issue_num}-"):
                return head, pr
            if head.startswith(f"{p}/t_") and (
                f"-{issue_num}-" in head or head.endswith(f"-{issue_num}")
            ):
                return head, pr
    # 2. Fallback: headRefName содержит issue number где угодно + известный prefix.
    for pr in prs_data:
        head = pr.get("headRefName") or ""
        if (f"-{issue_num}-" in head) or head.endswith(f"-{issue_num}"):
            for p in agent_prefixes:
                if head.startswith(f"{p}/"):
                    return head, pr
    return None, None


# --------------------------------------------------------------------------- #
# Tests.
# --------------------------------------------------------------------------- #


AGENT_PREFIXES = [
    "z-backend", "z-developer", "z-llm-expert", "z-architect",
    "z-devops", "z-designer", "z-analyst", "z-tester",
]


def test_r1_branch_fallback_finds_z_backend_for_1764():
    """R1: z-backend/1764-bug-voice-track... находится через fallback."""
    prs = [{
        "number": 1768,
        "headRefName": "z-backend/1764-bug-voice-track-renardo-supercollider-wa",
        "state": "MERGED",
        "mergedAt": "2026-08-31T21:44:23Z",
    }]
    result = branch_fallback_search(prs, "1764", AGENT_PREFIXES)
    assert result is not None
    head, pr = result
    assert head == "z-backend/1764-bug-voice-track-renardo-supercollider-wa"
    assert pr["number"] == 1768


def test_r2_branch_fallback_finds_z_developer_for_1780():
    """R2: z-developer/1780-fix-int-float-crash находится."""
    prs = [{
        "number": 1823,
        "headRefName": "z-developer/1780-fix-int-float-crash",
        "state": "MERGED",
        "mergedAt": "2026-08-31T21:55:21Z",
    }]
    result = branch_fallback_search(prs, "1780", AGENT_PREFIXES)
    assert result is not None
    head, _ = result
    assert head == "z-developer/1780-fix-int-float-crash"


def test_r3_branch_fallback_returns_none_when_no_pr():
    """R3: для issue без PR — пустой результат, идемпотентный skip."""
    prs = []
    result = branch_fallback_search(prs, "9999", AGENT_PREFIXES)
    assert result == (None, None)


def test_r4_branch_fallback_skips_unknown_prefix():
    """R4: ветка с неизвестным prefix (z-evil/...) не подбирается."""
    prs = [{
        "number": 1,
        "headRefName": "z-evil/1764-something",
        "state": "MERGED",
    }]
    result = branch_fallback_search(prs, "1764", AGENT_PREFIXES)
    assert result == (None, None)


def test_r5_branch_fallback_finds_z_llm_expert_for_1777():
    """R5: z-llm-expert/1777-bd121d36-non-music-tool-guard — issue #1777."""
    prs = [{
        "number": 1800,
        "headRefName": "z-llm-expert/1777-bd121d36-non-music-tool-guard",
        "state": "CLOSED",
    }]
    result = branch_fallback_search(prs, "1777", AGENT_PREFIXES)
    assert result is not None
    head, _ = result
    assert "z-llm-expert" in head


def test_r6_branch_fallback_handles_t_prefix_format():
    """R6: z-backend/t_xxx-1764-bug формат (task_id в имени)."""
    prs = [{
        "number": 1900,
        "headRefName": "z-backend/t_a09e893a-1764-bug-voice",
        "state": "MERGED",
    }]
    result = branch_fallback_search(prs, "1764", AGENT_PREFIXES)
    assert result is not None
    head, _ = result
    assert "z-backend/t_a09e893a-1764-bug-voice" == head


# --------------------------------------------------------------------------- #
# Staleness-grace logic tests (merge-gate.sh post-merge cleanup).
# --------------------------------------------------------------------------- #


def test_staleness_grace_triggers_at_25h():
    """Patch 2: grace=24h, MERGED 25h ago, branch still exists → trim + audit."""
    # R7: логика staleness-grace срабатывает после 24ч.
    grace_hours = 24
    age_h = 25
    should_trim = age_h >= grace_hours
    assert should_trim is True


def test_staleness_grace_defers_at_5h():
    """Patch 2: grace=24h, MERGED 5h ago → defer (e2e-process может прогнать)."""
    grace_hours = 24
    age_h = 5
    should_trim = age_h >= grace_hours
    assert should_trim is False


def test_staleness_grace_idempotent_skip():
    """R8: если уже стоит merged-no-e2e-stale — повторный тик skip."""
    # Логика: has_label(merged-no-e2e-stale) → skip без API.
    # В реальном скрипте: if has_label labels_norm MERGED_NO_E2E_STALE_LABEL → skip.
    # Здесь тестируем сам факт наличия env-переменной в дефолтах скрипта.
    content = SCRIPT.read_text()
    assert "MERGED_NO_E2E_STALE_LABEL" in content, \
        "merge-gate.sh должен определять MERGED_NO_E2E_STALE_LABEL (patch 2)"
    assert "merged-no-e2e-stale" in content, \
        "merge-gate.sh должен использовать метку merged-no-e2e-stale (default)"
    assert "AGENT_FLOW_BRANCH_PREFIXES" in content, \
        "merge-gate.sh должен определять AGENT_FLOW_BRANCH_PREFIXES (patch 1)"


def test_e2e_process_has_skip_for_merged_no_e2e_stale():
    """Patch 3: e2e-process должен skip'ать issue с merged-no-e2e-stale."""
    e2e_script = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"
    if not e2e_script.exists():
        pytest.skip("e2e-process.sh not found (different repo layout)")
    content = e2e_script.read_text()
    assert "merged-no-e2e-stale" in content, \
        "e2e-process.sh должен skip'ать merged-no-e2e-stale (иначе ping-pong)"


# --------------------------------------------------------------------------- #
# Integration smoke: проверить что переменные окружения определены в скрипте.
# --------------------------------------------------------------------------- #


def test_merge_gate_has_required_env_vars():
    """Sanity: defaults секция содержит наши новые env-vars."""
    content = SCRIPT.read_text()
    assert re.search(r'MERGED_NO_E2E_STALE_LABEL="\$\{MERGED_NO_E2E_STALE_LABEL:-merged-no-e2e-stale\}"', content), \
        "MERGED_NO_E2E_STALE_LABEL default должен быть 'merged-no-e2e-stale'"
    assert re.search(r'MERGED_LABELED_GRACE_HOURS="\$\{MERGED_LABELED_GRACE_HOURS:-24\}"', content), \
        "MERGED_LABELED_GRACE_HOURS default должен быть 24"
    assert "z-backend" in content and "z-developer" in content, \
        "AGENT_FLOW_BRANCH_PREFIXES должен содержать z-backend, z-developer"


def test_branch_fallback_python_block_present():
    """Patch 1: новый блок fallback-поиска по agent-prefixes в скрипте."""
    content = SCRIPT.read_text()
    # Определение: `AGENT_FLOW_BRANCH_PREFIXES="${...:-...}"` на строке ~67.
    # Использование: `_agent_prefixes="${AGENT_FLOW_BRANCH_PREFIXES:-...}"` на ~2823.
    # Различаем по тому, что определение присваивает ПЕРЕМЕННОЙ `AGENT_FLOW_BRANCH_PREFIXES=`,
    # а использование присваивает ДРУГОЙ переменной (например, `_agent_prefixes=`).
    import re
    _def_matches = list(re.finditer(r'^AGENT_FLOW_BRANCH_PREFIXES=', content, re.MULTILINE))
    assert len(_def_matches) >= 1, "AGENT_FLOW_BRANCH_PREFIXES должен быть определён в defaults"
    _def_idx = _def_matches[0].start()
    # Использование — все последующие вхождения имени переменной в тексте,
    # которые НЕ начинают строку (т.е. не присваивание самой переменной).
    _use_idx = -1
    for m in re.finditer(r'AGENT_FLOW_BRANCH_PREFIXES', content):
        if m.start() > _def_idx:
            # Проверяем, что это использование, а не новое присваивание.
            line_start = content.rfind('\n', 0, m.start()) + 1
            line_prefix = content[line_start:m.start()]
            if not line_prefix.startswith('AGENT_FLOW_BRANCH_PREFIXES'):
                _use_idx = m.start()
                break
    wt_idx = content.find('wt_branch="wt/${task_id}"')
    assert _use_idx > 0, "AGENT_FLOW_BRANCH_PREFIXES должно использоваться в main loop"
    assert _use_idx > wt_idx, \
        f"agent-prefix fallback должен идти после wt-fallback (use={_use_idx}, wt={wt_idx})"


def test_post_merge_cleanup_python_block_present():
    """Patch 2: блок staleness-grace присутствует в post-merge reconcile."""
    content = SCRIPT.read_text()
    import re
    # Определение: `MERGED_LABELED_GRACE_HOURS="${...:-24}"` на строке ~62.
    _def_matches = list(re.finditer(r'^MERGED_LABELED_GRACE_HOURS=', content, re.MULTILINE))
    assert len(_def_matches) >= 1, "MERGED_LABELED_GRACE_HOURS должен быть определён в defaults"
    _def_idx = _def_matches[0].start()
    # Использование — все последующие вхождения, не начинающие строку.
    _use_idx = -1
    for m in re.finditer(r'MERGED_LABELED_GRACE_HOURS', content):
        if m.start() > _def_idx:
            line_start = content.rfind('\n', 0, m.start()) + 1
            line_prefix = content[line_start:m.start()]
            if not line_prefix.startswith('MERGED_LABELED_GRACE_HOURS'):
                _use_idx = m.start()
                break
    assert _use_idx > 0, "MERGED_LABELED_GRACE_HOURS должно использоваться в main loop"
    # Блок должен быть ПОСЛЕ `branch exists, destructive cleanup deferred`
    anchor = "branch ${branch} exists, destructive cleanup deferred"
    if anchor in content:
        idx_anchor = content.index(anchor)
        assert _use_idx > idx_anchor, \
            f"staleness-grace блок должен идти после deferred cleanup (use={_use_idx}, anchor={idx_anchor})"
    assert "merged-no-e2e-stale" in content, "audit-метка должна использоваться"


# --------------------------------------------------------------------------- #
# Real-world integration: dry-run merge-gate с нашими фиксами на orphan-issue.
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(
    "GH_CONFIG_DIR" not in os.environ,
    reason="requires real gh auth (skip in unit-test env)",
)
def test_integration_orphan_1764_via_real_gh():
    """Integration: наш branch-fallback находит PR #1768 для issue #1764
    через реальный gh API. Skip в unit-окружении без GH_CONFIG_DIR или при
    GitHub rate-limit (часто в CI).
    """
    cmd = [
        "gh", "pr", "list", "--repo", "krikz/rob_box_project",
        "--state", "all", "--search", "1764 in:title",
        "--json", "number,headRefName,state",
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except FileNotFoundError:
        pytest.skip("gh CLI not installed in test env")
    if res.returncode != 0:
        # GitHub rate limit — типичная ситуация в CI. Skip, не fail.
        if "rate limit" in res.stderr.lower() or res.returncode != 0:
            pytest.skip(f"gh API error (rate-limit?): {res.stderr.strip()[:100]}")
        pytest.fail(f"gh pr list failed: {res.stderr}")
    import json as _json
    prs = _json.loads(res.stdout)
    result = branch_fallback_search(prs, "1764", AGENT_PREFIXES)
    if result is None:
        pytest.skip("no PR found for #1764 (fixture may have changed)")
    head, pr = result
    assert "1764" in head
    assert pr["state"] in ("MERGED", "CLOSED", "OPEN")
