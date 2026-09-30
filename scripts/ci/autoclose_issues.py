#!/usr/bin/env python3
"""Закрывает issues по closing-keywords смерженного в `develop` PR (issue #3013).

Почему это нужно
----------------
GitHub учитывает `Fixes #N` / `Closes #N` / `Resolves #N` ТОЛЬКО при merge в
default-ветку репозитория. У нас рабочие PR вливаются в `develop`, который
default-веткой не является, поэтому issues остаются OPEN (PR #3092 → #3089,
#3055 → #3049, #3040 → #3024, #3159 → #3044/#3045/#3047/#3048 закрывали руками;
18 случаев за 24.09.2026 перечислены в #3013).

Скрипт вызывается workflow "G: Auto-close Issues on Develop Merge" на событии
`pull_request: closed` (merged) и вручную (`workflow_dispatch`) для backfill.

Что считается ссылкой на закрытие
---------------------------------
Ключевые слова GitHub (регистр не важен):
    close closes closed fix fixes fixed resolve resolves resolved
с необязательным `:` после слова, затем одна или несколько ссылок:
    #N,  owner/repo#N (только ЭТОТ репозиторий),
    https://github.com/owner/repo/issues/N (только ЭТОТ репозиторий).
Несколько ссылок подряд после одного слова разделяются пробелами, запятыми
или `and`: `Closes #1 #2`, `Closes #1, #2`, `Fixes #1 and #2`.

`Refs #N`, `Part of #N`, голый `#N` — НЕ закрывают. Ссылки внутри
fenced-блоков кода (```), inline-кода (`...`) и HTML-комментариев
игнорируются: там обычно цитаты (например, в самом #3013 перечислены
чужие `Fixes #2926`), а не намерение закрыть.

Парсятся и body, и title PR (GitHub сам title не парсит; у ~половины наших
PR ключевое слово живёт только в title — см. #3013).

Использование
-------------
    autoclose_issues.py parse --repo krikz/rob_box_project  < text
    autoclose_issues.py close --repo krikz/rob_box_project --pr 3092 [--dry-run]

`close` ходит в GitHub через `gh api` (нужен GH_TOKEN с issues:write) и
для каждого найденного issue:
  * пропускает, если это на самом деле PR, или issue уже закрыт;
  * оставляет комментарий со ссылкой на PR и merge-sha;
  * закрывает с state_reason=completed.
Выход: 0 — всё обработано (в т.ч. «нечего закрывать»), 1 — хотя бы один
issue закрыть не удалось, 2 — PR не смержен / ошибка входа.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from typing import Callable, Iterable, List, Optional, Sequence

KEYWORDS = (
    "close",
    "closes",
    "closed",
    "fix",
    "fixes",
    "fixed",
    "resolve",
    "resolves",
    "resolved",
)

# Одна ссылка на issue: #N | owner/repo#N | https://github.com/owner/repo/issues/N
_REF = (
    r"(?:"
    r"https?://github\.com/(?P<url_repo>[\w.-]+/[\w.-]+)/issues/(?P<url_num>\d+)"
    r"|(?P<slug_repo>[\w.-]+/[\w.-]+)?#(?P<num>\d+)"
    r")"
)
_REF_RE = re.compile(_REF, re.IGNORECASE)
# Та же ссылка без именованных групп — для повторения внутри _CLAUSE_RE.
_REF_ANON = re.sub(r"\(\?P<\w+>", "(?:", _REF)

# Ключевое слово + цепочка ссылок. Слово не должно быть хвостом другого
# слова ("prefixes #1", "unresolved #1") — отсюда (?<![\w-]).
_CLAUSE_RE = re.compile(
    r"(?<![\w-])(?:" + "|".join(KEYWORDS) + r")"
    r"\s*:?\s+"
    r"(?P<refs>" + _REF_ANON + r"(?:\s*(?:,|\band\b)?\s*" + _REF_ANON + r")*)"
    r"(?![\w/])",
    re.IGNORECASE,
)

_FENCED_RE = re.compile(r"(?ms)^[ \t]*(`{3,}|~{3,}).*?^[ \t]*\1[^\n]*$")
# Inline-код: открывающая и закрывающая серии бэктиков одной длины
# (```` ``` ```` — валидный inline-код с тройным бэктиком внутри).
_INLINE_CODE_RE = re.compile(r"(?<!`)(`+)(?!`)[^\n]*?(?<!`)\1(?!`)")
_HTML_COMMENT_RE = re.compile(r"(?s)<!--.*?-->")


def _strip_quoted(text: str) -> str:
    """Убирает код и HTML-комментарии — там цитаты, а не намерение закрыть."""
    # Порядок важен: сначала код (внутри него `<!--` — просто текст),
    # потом HTML-комментарии.
    text = _FENCED_RE.sub(" ", text)
    text = _INLINE_CODE_RE.sub(" ", text)
    text = _HTML_COMMENT_RE.sub(" ", text)
    return text


def parse_closing_refs(text: Optional[str], repo: str) -> List[int]:
    """Номера issues ЭТОГО репо, которые `text` закрывает keyword'ами GitHub.

    Возвращает отсортированный список без дублей.
    """
    if not text:
        return []
    repo_l = repo.lower()
    found = set()
    for clause in _CLAUSE_RE.finditer(_strip_quoted(text)):
        # Отдельные ссылки из цепочки достаём вторым проходом.
        for ref in _REF_RE.finditer(clause.group("refs")):
            if ref.group("url_num"):
                ref_repo, num = ref.group("url_repo"), ref.group("url_num")
            else:
                ref_repo, num = ref.group("slug_repo"), ref.group("num")
            if ref_repo is not None and ref_repo.lower() != repo_l:
                continue  # чужой репозиторий — не наше дело
            found.add(int(num))
    return sorted(found)


# ---------------------------------------------------------------- GitHub I/O

Runner = Callable[[Sequence[str], Optional[str]], str]


def _gh(args: Sequence[str], stdin: Optional[str] = None) -> str:
    res = subprocess.run(
        ["gh", *args], input=stdin, capture_output=True, text=True, check=False
    )
    if res.returncode != 0:
        raise RuntimeError(f"gh {' '.join(args)} failed: {res.stderr.strip()}")
    return res.stdout


def _api_json(run: Runner, path: str) -> dict:
    return json.loads(run(["api", path], None))


def close_for_pr(
    repo: str,
    pr_number: int,
    dry_run: bool = False,
    run: Runner = _gh,
    log: Callable[[str], None] = print,
) -> int:
    pr = _api_json(run, f"repos/{repo}/pulls/{pr_number}")
    if not pr.get("merged_at"):
        log(f"PR #{pr_number} is not merged — nothing to do")
        return 2
    base = (pr.get("base") or {}).get("ref", "?")
    sha = pr.get("merge_commit_sha") or "?"
    pr_url = pr.get("html_url") or f"https://github.com/{repo}/pull/{pr_number}"

    numbers = sorted(
        set(parse_closing_refs(pr.get("title"), repo))
        | set(parse_closing_refs(pr.get("body"), repo))
    )
    log(f"PR #{pr_number} (base={base}, merge={sha[:12]}): closing refs {numbers}")
    if not numbers:
        return 0

    failures = 0
    for num in numbers:
        if num == pr_number:
            continue
        try:
            issue = _api_json(run, f"repos/{repo}/issues/{num}")
        except (RuntimeError, ValueError) as exc:
            log(f"  #{num}: lookup failed: {exc}")
            failures += 1
            continue
        if "pull_request" in issue:
            log(f"  #{num}: is a pull request — skip")
            continue
        if issue.get("state") != "open":
            log(f"  #{num}: already {issue.get('state')} — skip")
            continue
        if dry_run:
            log(f"  #{num}: OPEN — would close (dry-run)")
            continue
        body = (
            f"🤖 auto-close (#3013): закрыто смерженным PR #{pr_number} "
            f"({pr_url}) — merge commit `{sha}` в `{base}`.\n\n"
            f"GitHub учитывает closing-keywords только при merge в default-ветку, "
            f"поэтому для `{base}` это делает workflow "
            f'"G: Auto-close Issues on Develop Merge".'
        )
        try:
            run(
                [
                    "api",
                    "-X",
                    "POST",
                    f"repos/{repo}/issues/{num}/comments",
                    "--input",
                    "-",
                ],
                json.dumps({"body": body}),
            )
            run(
                ["api", "-X", "PATCH", f"repos/{repo}/issues/{num}", "--input", "-"],
                json.dumps({"state": "closed", "state_reason": "completed"}),
            )
        except RuntimeError as exc:
            log(f"  #{num}: close failed: {exc}")
            failures += 1
            continue
        log(f"  #{num}: closed")
    return 1 if failures else 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_parse = sub.add_parser("parse", help="print closing refs found in stdin")
    p_parse.add_argument("--repo", required=True)
    p_close = sub.add_parser("close", help="close issues referenced by a merged PR")
    p_close.add_argument("--repo", required=True)
    p_close.add_argument("--pr", type=int, required=True, action="append")
    p_close.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(list(argv) if argv is not None else None)

    if args.cmd == "parse":
        print(" ".join(str(n) for n in parse_closing_refs(sys.stdin.read(), args.repo)))
        return 0
    rc = 0
    for pr_number in args.pr:
        rc = max(rc, close_for_pr(args.repo, pr_number, dry_run=args.dry_run))
    return rc


if __name__ == "__main__":
    sys.exit(main())
