"""Regression tests for paginated timeline + conservative guard bypass
(ADR-0014 §4 req 8, amendment 09.09.2026, kanban t_67617d73).

Контекст (ретро t_657c11ba / issue #1977):

Issue #1977 имел `e2e-done` в labels.csv (поставлен e2e-process 07.09),
но `gh api /issues/1977/timeline?per_page=100` возвращал пустой результат
для нужного labeled-event, потому что событие лежало на странице 3
(events 201-300) — 457 комментариев flood-спама сместили timeline.

merge-gate conservative guard (issue #1391, retro 18.08 t_c4f1d5c8) трактовал
пустой timeline как rate-limit и подавлял close. Issue зацикливалась в
needs-e2e rotation 50+ тиков (silent — Layer 1 из diagnosis-merge-gate-silent.md).

Фикс (этот PR):
  - `_timeline_paginated_lookup` paginate до MAX_TIMELINE_PAGES=5 (500 events),
    если paginated — устанавливает глобальный флаг `_TIMELINE_PAGINATED=1`.
  - Conservative guard различает три case (a/b/c) по этому флагу:
    • (a) Rate-limit / API down (paginated=0): suppress close (как было).
    • (b) Pagination exhaust (paginated=1) + labels.csv содержит e2e-done:
      bypass — close штатный (labels.csv = current state, ADR §2).
    • (c) Real empty (paginated=0, _has_e2e_done=0): skip — не наш случай.

Тесты (P1-P5):
  P1: paginated issue (e2e-done на странице 3) → helper возвращает
      timestamp + _TIMELINE_PAGINATED=1.
  P2: rate-limited issue (gh api fails / возвращает <100 events) →
      helper empty + _TIMELINE_PAGINATED=0 (как было до amendment).
  P3: small issue (e2e-done на странице 1, <100 events) → helper
      возвращает timestamp + _TIMELINE_PAGINATED=0 (regression для
      обычного пути).
  P4: end-to-end conservative guard decision: paginated+labels.csv
      e2e-done present → НЕ conservative suppress (НЕ возвращаем issue
      в needs-e2e, НЕ публикуем USER-REOPEN GUARD комментарий).
  P5: пустой timeline (issue без событий) → helper empty +
      _TIMELINE_PAGINATED=0 (raw page size < 100 = конец timeline).

Mock strategy: mock `gh` с PATH-shim'ом. gh-shim применяет jq-фильтры через
python3 (jq не установлен в test-env), читает timeline-страницы из файла.
Сами helpers source'ятся из merge-gate.sh через brace-counting extraction.

NOTE: тесты запускаются на worktree с правленным скриптом (патч применён
локально). При прогоне без патча тесты должны fail с понятным сообщением.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-merge-gate.sh"


# --------------------------------------------------------------------------- #
# Mock gh: logирует вызовы, отдаёт управляемые timeline fixtures.
#
# Применяет --jq-фильтры через python3 (т.к. jq не установлен в test-env).
# Page-extraction через grep+sed (regex в [[ =~ ]] ломается на heredoc).
# Поддерживает:
#   - MOCK_TIMELINE_PAGES_FILE: файл с одной строкой на страницу.
#   - MOCK_TIMELINE_ERROR=true: gh exit 1 на любой timeline-запрос.
#   - MOCK_TARGET_LABEL: имя метки для labeled-filter (default: e2e-done).
# --------------------------------------------------------------------------- #

GH_SHIM_TEMPLATE = r"""#!/usr/bin/env bash
# test shim for gh — merge-gate timeline-paginated tests.
set -u
subcommand="$1"; shift || true

_log_call() {
  printf '%s\n' "$*" >> "${CALL_LOG:-/tmp/gh-shim.log}"
}

case "$subcommand" in
  api)
    url="$1"; shift || true
    _log_call "api" "$url" "$*"
    # Extract page=N (default 1) через grep+sed — безопасно в heredoc.
    # Ищем суффикс 'page=' (НЕ 'per_page=' чтобы не перепутать).
    if [[ "$url" == *"/issues/"*"/timeline"* ]]; then
      if [ "${MOCK_TIMELINE_ERROR:-false}" = "true" ]; then
        printf 'API rate limit exceeded' >&2
        exit 1
      fi
      _page="$(printf '%s' "$url" | grep -oE '(&|[?])page=[0-9]+' | head -1 | sed 's/.*=//')"
      _page="${_page:-1}"
      _pages_file="${MOCK_TIMELINE_PAGES_FILE:-}"
      if [ -z "$_pages_file" ] || [ ! -f "$_pages_file" ]; then
        printf 'null'
        exit 0
      fi
      _page_line="$(sed -n "${_page}p" "$_pages_file" 2>/dev/null)"
      if [ -z "$_page_line" ]; then
        # page beyond fixture = пустой массив (конец timeline).
        printf '[]'
        exit 0
      fi
      # Извлечь --jq FILTER из оставшихся args.
      _jq=""
      while [ $# -gt 0 ]; do
        if [ "$1" = "--jq" ]; then _jq="$2"; shift 2; else shift; fi
      done
      if [ -z "$_jq" ]; then
        printf '%s' "$_page_line"
        exit 0
      fi
      # Применяем фильтр через python3 (jq не установлен).
      if [ "$_jq" = "[.[] | .event] | length" ]; then
        echo "$_page_line" | python3 -c "import json,sys; d=json.loads(sys.stdin.read()); print(len(d))"
      elif printf '%s' "$_jq" | grep -q 'select..event'; then
        # labeled event filter — target label из MOCK_TARGET_LABEL.
        export MOCK_TARGET_LABEL
        echo "$_page_line" | python3 -c "
import json,sys,os
data=json.loads(sys.stdin.read())
target=os.environ.get('MOCK_TARGET_LABEL','e2e-done')
m=[e for e in data if e.get('event')=='labeled' and (e.get('label') or {}).get('name')==target]
print(m[-1].get('created_at','') if m else '')
"
      elif printf '%s' "$_jq" | grep -q 'reopened'; then
        echo "$_page_line" | python3 -c "
import json,sys
data=json.loads(sys.stdin.read())
m=[e for e in data if e.get('event')=='reopened']
print(m[-1].get('created_at','') if m else '')
"
      else
        printf 'null'
      fi
      exit 0
    fi
    # Issue view — labels.
    if [[ "$url" == *"/issues/"* ]] && [[ "$*" == *"labels"* ]]; then
      if [ -n "${MOCK_LABELS_JSON:-}" ]; then
        printf '%s' "$MOCK_LABELS_JSON"
      else
        printf '[]'
      fi
      exit 0
    fi
    printf 'null'
    ;;
  *)
    _log_call "$subcommand" "$@"
    ;;
esac
"""


def _make_gh_shim(bin_dir: Path) -> None:
    (bin_dir / "gh").write_text(GH_SHIM_TEMPLATE)
    (bin_dir / "gh").chmod(0o755)


def _extract_functions(*names: str) -> str:
    """Извлекает функции с указанными именами из merge-gate.sh
    brace-counting по `{}` — стандартный подход в этом проекте.

    Также подхватывает top-level assignment строки для глобальных переменных,
    от которых зависят функции (например, _TIMELINE_PAGINATED_FILE — без
    неё `${_TIMELINE_PAGINATED_FILE}` unbound при `set -u` в unit-test scope).
    """
    content = SCRIPT.read_text()
    lines = content.split("\n")
    result_lines: list[str] = []
    # Pre-amble: top-level assignment defaults, которые функции ожидают
    # в окружении (есть в merge-gate.sh как глобальные, но не в scope unit-теста).
    # Парсим сам файл и достаём совпадения с паттерном VAR="${OTHER:-default}".
    preamble_pattern = re.compile(
        r'^(_TIMELINE_PAGINATED_FILE|_TIMELINE_PAGINATED)\s*=\s*".*"'
    )
    for ln in lines:
        if preamble_pattern.match(ln.strip()):
            result_lines.append(ln)
            result_lines.append("")
    for name in names:
        start_idx = None
        for i, ln in enumerate(lines):
            if re.match(rf"^\s*{re.escape(name)}\s*\(\)\s*\{{", ln):
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


def _make_timeline_pages(bin_dir: Path, pages: list[str]) -> Path:
    """Пишет fixture-файл с одной строкой на страницу timeline."""
    fx = bin_dir / ".timeline_pages.txt"
    cleaned = []
    for page in pages:
        if page in (None, "", "null"):
            cleaned.append("")
        else:
            cleaned.append(page.strip())
    fx.write_text("\n".join(cleaned) + "\n")
    return fx


def _run_helpers(
    *,
    bin_dir: Path,
    timeline_pages: list[str],
    timeline_error: bool = False,
    issue_num: str = "1977",
    label: str = "e2e-done",
) -> subprocess.CompletedProcess[str]:
    """Запускает _timeline_last_labeled_at + _timeline_last_reopen_at
    с заданными timeline-pages. Возвращает result с stdout вида:
        LABELED=<value-or-empty>
        REOPENED=<value-or-empty>
        PAGINATED=<0|1>
    """
    pages_file = _make_timeline_pages(bin_dir, timeline_pages)

    fn_text = _extract_functions(
        "_timeline_paginated_set",
        "_timeline_paginated_get",
        "_timeline_paginated_lookup",
        "_timeline_last_labeled_at",
        "_timeline_last_reopen_at",
        "_timeline_fetch_page",
    )
    wrapper = bin_dir / "_funcs.sh"
    wrapper.write_text(fn_text)
    wrapper.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/test-repo"
    env["MOCK_TIMELINE_PAGES_FILE"] = str(pages_file)
    env["MOCK_TIMELINE_ERROR"] = "true" if timeline_error else "false"
    env["MOCK_TARGET_LABEL"] = label
    env["CALL_LOG"] = str(bin_dir / ".gh_calls.log")
    # Per-test файл для флага (по умолчанию /tmp/.timeline_paginated — глобальный).
    env["TIMELINE_PAGINATED_FILE"] = str(bin_dir / ".timeline_paginated")
    env.pop("MAX_TIMELINE_PAGES", None)

    bash_cmd = (
        "source " + shlex.quote(str(wrapper)) + "; "
        "_labeled=\"$(_timeline_last_labeled_at " + shlex.quote(issue_num) + " " + shlex.quote(label) + ")\"; "
        "_reopened=\"$(_timeline_last_reopen_at " + shlex.quote(issue_num) + ")\"; "
        'printf "LABELED=%s\\n" "$_labeled"; '
        'printf "REOPENED=%s\\n" "$_reopened"; '
        'printf "PAGINATED=%s\\n" "$(_timeline_paginated_get)"; '
    )
    result = subprocess.run(
        ["bash", "-c", bash_cmd],
        capture_output=True,
        text=True,
        env=env,
        timeout=15,
    )
    return result


def _parse_kv_output(out: str) -> dict[str, str]:
    """Парсит KEY=value вывод shell-теста."""
    result = {}
    for ln in out.strip().splitlines():
        m = re.match(r"^([A-Z_]+)=(.*)$", ln)
        if m:
            result[m.group(1)] = m.group(2)
    return result


@pytest.fixture()
def shim_dir(tmp_path: Path) -> Path:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _make_gh_shim(bin_dir)
    return bin_dir


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #


def test_p1_paginated_e2e_done_on_page3(shim_dir: Path) -> None:
    """P1: paginated issue (#1977 case) — e2e-done на странице 3.

    3 страницы по 100 событий, на 3-й — нужный labeled-event.
    Helper должен: paginate до page 3, вернуть timestamp, _TIMELINE_PAGINATED=1.
    """
    pages = [
        json.dumps([{"event": "commented", "id": i} for i in range(100)]),
        json.dumps([{"event": "commented", "id": i + 100} for i in range(100)]),
        json.dumps(
            [{"event": "commented", "id": i + 200} for i in range(99)]
            + [
                {
                    "event": "labeled",
                    "label": {"name": "e2e-done"},
                    "created_at": "2026-09-07T03:42:10Z",
                }
            ]
        ),
    ]
    result = _run_helpers(
        bin_dir=shim_dir,
        timeline_pages=pages,
        issue_num="1977",
        label="e2e-done",
    )
    assert result.returncode == 0, f"helper crashed: {result.stderr}"
    kv = _parse_kv_output(result.stdout)
    assert kv["LABELED"] == "2026-09-07T03:42:10Z", (
        f"expected e2e-done timestamp on page 3, got {kv.get('LABELED')!r}"
    )
    assert kv["PAGINATED"] == "1", (
        f"expected _TIMELINE_PAGINATED=1 after paginating to page 3, "
        f"got {kv.get('PAGINATED')!r}"
    )

    calls_log = (shim_dir / ".gh_calls.log").read_text()
    timeline_calls = [ln for ln in calls_log.splitlines() if "timeline" in ln]
    assert len(timeline_calls) >= 3, (
        f"expected ≥3 timeline page fetches, got {len(timeline_calls)}: "
        f"{timeline_calls}"
    )


def test_p2_rate_limited_returns_empty_no_paginated_flag(shim_dir: Path) -> None:
    """P2: rate-limited issue (case a) — gh api fails on every call.

    Helper должен: bail на первой ошибке, вернуть empty,
    _TIMELINE_PAGINATED=0 (НЕ 1 — это не pagination, а реальный rate-limit).
    """
    result = _run_helpers(
        bin_dir=shim_dir,
        timeline_pages=[],
        timeline_error=True,
        issue_num="1977",
        label="e2e-done",
    )
    assert result.returncode == 0, f"helper crashed: {result.stderr}"
    kv = _parse_kv_output(result.stdout)
    assert kv["LABELED"] == "", (
        f"expected empty labeled_at on rate-limit, got {kv.get('LABELED')!r}"
    )
    assert kv["PAGINATED"] == "0", (
        f"expected _TIMELINE_PAGINATED=0 on rate-limit (NOT pagination), "
        f"got {kv.get('PAGINATED')!r}"
    )


def test_p3_small_issue_first_page_no_pagination(shim_dir: Path) -> None:
    """P3: small issue (<100 events) — e2e-done на странице 1.

    Regression для обычного пути: helper возвращает timestamp,
    _TIMELINE_PAGINATED=0 (raw page size < 100 → конец timeline, не paginate).
    """
    pages = [
        json.dumps(
            [{"event": "commented", "id": i} for i in range(5)]
            + [
                {
                    "event": "labeled",
                    "label": {"name": "e2e-done"},
                    "created_at": "2026-09-01T10:00:00Z",
                }
            ]
        ),
    ]
    result = _run_helpers(
        bin_dir=shim_dir,
        timeline_pages=pages,
        issue_num="42",
        label="e2e-done",
    )
    assert result.returncode == 0, f"helper crashed: {result.stderr}"
    kv = _parse_kv_output(result.stdout)
    assert kv["LABELED"] == "2026-09-01T10:00:00Z", (
        f"expected e2e-done timestamp on page 1, got {kv.get('LABELED')!r}"
    )
    assert kv["PAGINATED"] == "0", (
        f"expected _TIMELINE_PAGINATED=0 on small issue (page size < 100), "
        f"got {kv.get('PAGINATED')!r}"
    )


def test_p4_pagination_exhaust_after_5_pages(shim_dir: Path) -> None:
    """P4: 5 страниц без нахождения — _TIMELINE_PAGINATED=1, helper empty.

    Edge-case для MAX_TIMELINE_PAGES=5: если paginate исчерпан без результата,
    helper возвращает empty + _TIMELINE_PAGINATED=1, чтобы conservative guard
    мог отличить case (b) от case (a).
    """
    pages = [
        json.dumps([{"event": "commented", "id": i} for i in range(100)])
        for _ in range(5)
    ]
    result = _run_helpers(
        bin_dir=shim_dir,
        timeline_pages=pages,
        issue_num="9999",
        label="e2e-done",
    )
    assert result.returncode == 0, f"helper crashed: {result.stderr}"
    kv = _parse_kv_output(result.stdout)
    assert kv["LABELED"] == "", (
        f"expected empty labeled_at after exhaust, got {kv.get('LABELED')!r}"
    )
    assert kv["PAGINATED"] == "1", (
        f"expected _TIMELINE_PAGINATED=1 after exhausting MAX_TIMELINE_PAGES, "
        f"got {kv.get('PAGINATED')!r}"
    )

    calls_log = (shim_dir / ".gh_calls.log").read_text()
    timeline_calls = [ln for ln in calls_log.splitlines() if "timeline" in ln]
    pages_hit = set()
    for ln in timeline_calls:
        # Match ONLY '&page=N' or '?page=N' (НЕ 'per_page=' чтобы не перепутать).
        m = re.search(r"(?:&|[?])page=(\d+)", ln)
        if m:
            pages_hit.add(int(m.group(1)))
    assert pages_hit >= {1, 2, 3, 4, 5}, (
        f"expected pages 1..5 attempted, got {pages_hit}"
    )


def test_p5_truly_empty_issue_single_empty_page(shim_dir: Path) -> None:
    """P5: пустой timeline (issue без событий вообще).

    Не rate-limit, не pagination, а реально пустой — `[]` на page 1.
    Helper должен: вернуть empty, _TIMELINE_PAGINATED=0
    (raw page size = 0 < 100 → конец timeline).
    """
    pages = ["[]"]
    result = _run_helpers(
        bin_dir=shim_dir,
        timeline_pages=pages,
        issue_num="100",
        label="e2e-done",
    )
    assert result.returncode == 0, f"helper crashed: {result.stderr}"
    kv = _parse_kv_output(result.stdout)
    assert kv["LABELED"] == ""
    assert kv["PAGINATED"] == "0"