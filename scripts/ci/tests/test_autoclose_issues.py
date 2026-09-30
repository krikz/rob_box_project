"""Регресс-тесты scripts/ci/autoclose_issues.py (issue #3013).

Парсер closing-keywords + логика закрытия с подменённым `gh` (runner),
без сети. Кейсы взяты из реальных PR, которые НЕ закрыли свои issues при
merge в develop: #3092 "Closes #3089", #3055 "Closes #3049",
#3040 "Fixes #3024", #3159 "Closes #3044 #3045 #3047 #3048".
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "ci" / "autoclose_issues.py"
WORKFLOW_PATH = (
    REPO_ROOT / ".github" / "workflows" / "G-Auto-close Issues on Develop Merge.yml"
)

_SPEC = importlib.util.spec_from_file_location(
    "autoclose_issues_under_test", SCRIPT_PATH
)
assert _SPEC is not None and _SPEC.loader is not None
ac = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(ac)  # type: ignore[union-attr]

REPO = "krikz/rob_box_project"


# ------------------------------------------------------------------ parser


@pytest.mark.parametrize(
    "text, expected",
    [
        # реальные PR из #3013
        ("Closes #3089", [3089]),
        ("## Summary\n...\nCloses #3049\n", [3049]),
        ("Fixes #3024", [3024]),
        ("Closes #3044 #3045 #3047 #3048", [3044, 3045, 3047, 3048]),
        # все 9 ключевых слов, регистр не важен
        ("close #1", [1]),
        ("CLOSES #2", [2]),
        ("Closed #3", [3]),
        ("fix #4", [4]),
        ("FIXES #5", [5]),
        ("Fixed #6", [6]),
        ("resolve #7", [7]),
        ("Resolves #8", [8]),
        ("RESOLVED #9", [9]),
        # двоеточие после слова
        ("Fixes: #10", [10]),
        # несколько номеров после одного слова
        ("Closes #1, #2", [1, 2]),
        ("Closes #1,#2 , #3", [1, 2, 3]),
        ("Fixes #1 and #2", [1, 2]),
        # несколько ключевых слов, дубли схлопываются
        ("Fixes #5\nCloses #3\nResolves #5", [3, 5]),
        # owner/repo#N — только этот репозиторий (регистр не важен)
        ("Closes krikz/rob_box_project#42", [42]),
        ("Closes Krikz/Rob_Box_Project#42", [42]),
        ("Closes other/repo#42", []),
        ("Fixes other/repo#1 krikz/rob_box_project#2", [2]),
        # полная ссылка на issue
        ("Resolves https://github.com/krikz/rob_box_project/issues/77", [77]),
        ("Resolves https://github.com/other/repo/issues/77", []),
        # в середине строки / в скобках
        ("fix(ci): thing (closes #12)", [12]),
        ("- Closes #13.", [13]),
    ],
)
def test_parse_closing_refs_matches(text, expected):
    assert ac.parse_closing_refs(text, REPO) == expected


@pytest.mark.parametrize(
    "text",
    [
        "",
        None,
        "Refs #3013",
        "refs #1, #2",
        "Part of #5",
        "See #6",
        "#7",
        # слово — хвост другого слова
        "prefixes #8",
        "unresolved #9",
        "hotfix-closes #10",
        # без ссылки / без пробела
        "Fixes the bug",
        "Fixes#11",
        # номер, прилипший к слову
        "Fixes #12abc",
        # цитаты в коде и HTML-комментариях (как в теле самого #3013)
        "PR #2927 «`Fixes #2926`»",
        "```\nCloses #13\n```",
        "~~~text\nFixes #14\n~~~",
        "<!-- Closes #15 -->",
    ],
)
def test_parse_closing_refs_ignores(text):
    assert ac.parse_closing_refs(text, REPO) == []


def test_real_pr_3159_line():
    # Дословная строка из тела PR 3159 (merged в develop 2026-09-29,
    # issues 3044/3045/3047/3048 остались OPEN). Refs-номера и «см.» — не
    # закрываются.
    line = "Refs #3149 #3150 #3151 #3152 · Closes #3044 #3045 #3047 #3048 · ⚠ см. #3162"
    assert ac.parse_closing_refs(line, REPO) == [3044, 3045, 3047, 3048]


def test_mixed_code_spans_do_not_leak_refs():
    # Регресс: тело ЭТОГО PR (#3013) — HTML-комментарий внутри inline-кода
    # и inline-код с тройным бэктиком сбивали парность бэктиков, и
    # `Fixes #2926` из цитаты утекал в «закрыть».
    line = (
        "Ссылки внутри ```` ``` ````/`~~~`-блоков игнорируются: цитаты "
        "`Fixes #2926`; шаблон PR содержит `<!-- - Closes #1234 -->`)."
    )
    assert ac.parse_closing_refs(line, REPO) == []
    assert ac.parse_closing_refs("````\nCloses #1\n````", REPO) == []
    assert ac.parse_closing_refs("``Fixes #1`` then Fixes #2", REPO) == [2]


def test_code_stripping_keeps_real_refs_around_quotes():
    body = "Цитата: `Fixes #1`\n```\nCloses #2\n```\nCloses #3\n<!-- fixes #4 -->"
    assert ac.parse_closing_refs(body, REPO) == [3]


# ------------------------------------------------------------ close logic


class FakeGh:
    """Подмена `gh api`: отдаёт заготовленные объекты, пишет вызовы."""

    def __init__(self, pr, issues, fail_on=None):
        self.pr = pr
        self.issues = issues
        self.fail_on = fail_on or set()
        self.calls = []

    def __call__(self, args, stdin=None):
        self.calls.append((list(args), stdin))
        if args[:2] == ["api", "-X"]:
            path = args[3]
            num = int(path.split("/issues/")[1].split("/")[0])
            if num in self.fail_on:
                raise RuntimeError("HTTP 403")
            return "{}"
        path = args[1]
        if "/pulls/" in path:
            return json.dumps(self.pr)
        num = int(path.rsplit("/", 1)[1])
        if num not in self.issues:
            raise RuntimeError("HTTP 404")
        return json.dumps(self.issues[num])

    def mutations(self):
        return [
            (a[2], a[3], json.loads(s)) for a, s in self.calls if a[:2] == ["api", "-X"]
        ]


def _pr(**kw):
    pr = {
        "number": 3159,
        "title": "fix: stuff",
        "body": "Closes #3044 #3045",
        "merged_at": "2026-09-29T10:00:00Z",
        "merge_commit_sha": "abc123def4567890",
        "html_url": "https://github.com/krikz/rob_box_project/pull/3159",
        "base": {"ref": "develop"},
    }
    pr.update(kw)
    return pr


def test_close_comments_and_closes_each_open_issue():
    gh = FakeGh(_pr(), {3044: {"state": "open"}, 3045: {"state": "open"}})
    rc = ac.close_for_pr(REPO, 3159, run=gh, log=lambda _m: None)
    assert rc == 0
    muts = gh.mutations()
    assert [(m, p) for m, p, _ in muts] == [
        ("POST", f"repos/{REPO}/issues/3044/comments"),
        ("PATCH", f"repos/{REPO}/issues/3044"),
        ("POST", f"repos/{REPO}/issues/3045/comments"),
        ("PATCH", f"repos/{REPO}/issues/3045"),
    ]
    comment = muts[0][2]["body"]
    assert "#3159" in comment and "abc123def4567890" in comment and "develop" in comment
    assert muts[1][2] == {"state": "closed", "state_reason": "completed"}


def test_title_refs_are_closed_too():
    gh = FakeGh(
        _pr(title="fix(x): y (Fixes #3089)", body="no keywords"),
        {3089: {"state": "open"}},
    )
    assert ac.close_for_pr(REPO, 3159, run=gh, log=lambda _m: None) == 0
    assert ("PATCH", f"repos/{REPO}/issues/3089") in [
        (m, p) for m, p, _ in gh.mutations()
    ]


def test_skips_closed_issues_pull_requests_and_self():
    gh = FakeGh(
        _pr(body="Closes #1 #2 #3159"),
        {1: {"state": "closed"}, 2: {"state": "open", "pull_request": {}}},
    )
    assert ac.close_for_pr(REPO, 3159, run=gh, log=lambda _m: None) == 0
    assert gh.mutations() == []


def test_unmerged_pr_is_noop():
    gh = FakeGh(_pr(merged_at=None), {3044: {"state": "open"}})
    assert ac.close_for_pr(REPO, 3159, run=gh, log=lambda _m: None) == 2
    assert gh.mutations() == []


def test_dry_run_does_not_mutate():
    logs = []
    gh = FakeGh(_pr(), {3044: {"state": "open"}, 3045: {"state": "open"}})
    assert ac.close_for_pr(REPO, 3159, dry_run=True, run=gh, log=logs.append) == 0
    assert gh.mutations() == []
    assert any("would close" in m for m in logs)


def test_refs_only_pr_closes_nothing():
    gh = FakeGh(_pr(body="Refs #3013"), {3013: {"state": "open"}})
    assert ac.close_for_pr(REPO, 3159, run=gh, log=lambda _m: None) == 0
    assert gh.mutations() == []


def test_failure_on_one_issue_does_not_stop_others_and_sets_rc1():
    gh = FakeGh(
        _pr(body="Closes #3044 #3045 #9999"),
        {3044: {"state": "open"}, 3045: {"state": "open"}},
        fail_on={3044},
    )
    assert ac.close_for_pr(REPO, 3159, run=gh, log=lambda _m: None) == 1
    assert ("PATCH", f"repos/{REPO}/issues/3045") in [
        (m, p) for m, p, _ in gh.mutations()
    ]


# ---------------------------------------------------------------- workflow


def test_workflow_is_wired_to_develop_merges_with_minimal_permissions():
    yaml = pytest.importorskip("yaml")
    wf = yaml.safe_load(WORKFLOW_PATH.read_text(encoding="utf-8"))
    triggers = wf.get("on", wf.get(True))
    assert triggers["pull_request"]["types"] == ["closed"]
    assert triggers["pull_request"]["branches"] == ["develop"]
    assert "workflow_dispatch" in triggers
    assert wf["permissions"] == {
        "issues": "write",
        "pull-requests": "read",
        "contents": "read",
    }
    job = next(iter(wf["jobs"].values()))
    assert "github.event.pull_request.merged" in job["if"]
    run_text = "\n".join(s.get("run", "") for s in job["steps"])
    assert "scripts/ci/autoclose_issues.py" in run_text
    # PR title/body — недоверенный ввод: не интерполировать ${{ }} в run:
    assert "github.event.pull_request.body" not in run_text
    assert "github.event.pull_request.title" not in run_text
