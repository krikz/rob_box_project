#!/usr/bin/env python3
"""validate_test_ws_dirs.py — pre-PR guard на молчаливый контракт test_ws.

ЗАЧЕМ
=====
`G-Run Tests.yml` собирает CI-workspace `test_ws/` из ПОДМНОЖЕСТВА корня репо::

    rsync src/ -> test_ws/src/
    for d in docker migrations docs .github; do rsync "$d" test_ws/; done

Список ``for d in ...`` — молчаливый контракт. Тест, который ходит walk-up'ом
до корня репо и читает корневой каталог ВНЕ этого списка, локально зелёный, а
на CI падает collect-error'ом (или тихо скипается, что хуже — покрытие теряется
без единого красного сигнала).

Этот баг случался дважды в одном и том же файле:

* ``t_29b9ce36`` -> PR #1874 (02.09): не было ``docker/``  -> metrics_server.py not found
* ``t_cfa21388`` -> PR #1958 (03.09): не было ``scripts/`` -> score.py not found,
  develop RED ~9ч, 20+ PR заблокированы одним и тем же collect-error'ом.

Guard закрывает КЛАСС, а не третий экземпляр.

ЧТО ИМЕННО ПРОВЕРЯЕТСЯ
======================
Только ссылки, ПРИВЯЗАННЫЕ К КОРНЮ РЕПО, — иначе получаются ложные
срабатывания на package-local каталоги (``src/rob_box_animations/scripts/``
это НЕ корневой ``scripts/``). Распознаются два способа добраться до корня:

1. ``Path(__file__).resolve().parents[N]`` где N ровно столько, чтобы попасть
   в корень репо (считается от реального пути файла).
2. ``os.environ["ROB_BOX_REPO_ROOT"]`` — якорь, который CI экспортирует.

Severity
--------
* ``FAIL`` — каталог не скопирован, и обращение к нему не защищено ``skipif``
  -> CI падает collect-error'ом. Блокирует.
* ``WARN`` — обращение защищено ``pytest.mark.skipif(...exists())`` -> на CI
  тест молча скипается. Не блокирует, но это дыра в покрытии.

ИСПОЛЬЗОВАНИЕ
=============
    python3 scripts/agent_flow/validate_test_ws_dirs.py
    python3 scripts/agent_flow/validate_test_ws_dirs.py --workflow <path>
    python3 scripts/agent_flow/validate_test_ws_dirs.py --strict   # WARN тоже валит

EXIT CODES
==========
0 — покрыто (или только WARN без --strict); 1 — есть FAIL; 2 — ошибка использования.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

DEFAULT_WORKFLOW = ".github/workflows/G-Run Tests.yml"

# Каталоги, которые не являются «данными для тестов» и не должны попадать
# в test_ws (или уже там есть через отдельный rsync src/).
IGNORED_ROOT_DIRS = {
    ".git",
    "src",
    "build",
    "install",
    "log",
    "node_modules",
    ".worktrees",
    "test_ws",
    ".venv",
    "venv",
    "__pycache__",
}

# `for d in docker migrations docs .github; do`
RSYNC_LINE_RE = re.compile(r"for d in (?P<dirs>[^;]+); do")

# `_X = Path(__file__).resolve().parents[4]`  (возможен перенос строки)
PARENTS_ASSIGN_RE = re.compile(
    r"(?P<var>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*\(?\s*"
    r"Path\(\s*__file__\s*\)\.resolve\(\)\.parents\[\s*(?P<n>\d+)\s*\]",
    re.MULTILINE,
)

# `Path(__file__).resolve().parents[4] / "scripts"` — инлайн, без переменной
PARENTS_INLINE_RE = re.compile(
    r"Path\(\s*__file__\s*\)\.resolve\(\)\.parents\[\s*(?P<n>\d+)\s*\]"
    r"\s*/\s*[\"'](?P<dir>[^\"'/]+)[\"']",
)

# `a / "scripts" / "voice_bench"` в генераторе по всем ancestors
PARENTS_ITER_RE = re.compile(
    r"for\s+(?P<var>[A-Za-z_][A-Za-z0-9_]*)\s+in\s+"
    r"Path\(\s*__file__\s*\)\.resolve\(\)\.parents\b",
)

ENV_ROOT_RE = re.compile(r"ROB_BOX_REPO_ROOT")


def workflow_rsync_lists(workflow: Path) -> list[tuple[int, set[str]]]:
    """Вернуть [(номер строки, множество каталогов)] для каждого job'а."""
    out: list[tuple[int, set[str]]] = []
    for lineno, line in enumerate(workflow.read_text(encoding="utf-8").splitlines(), 1):
        m = RSYNC_LINE_RE.search(line)
        if m:
            out.append((lineno, set(m.group("dirs").split())))
    return out


def root_dirs(repo: Path) -> set[str]:
    return {
        p.name for p in repo.iterdir() if p.is_dir() and p.name not in IGNORED_ROOT_DIRS
    }


def depth_to_root(test_file: Path, repo: Path) -> int:
    """Сколько .parents[] нужно от файла, чтобы попасть в корень репо."""
    return len(test_file.relative_to(repo).parts) - 1


def _skipif_guarded(src: str, var: str) -> bool:
    """Обращение защищено ``pytest.mark.skipif(...)``?

    Проверяем не только сам ``var``, но и переменные, ПРОИЗВОДНЫЕ от него:
    ``_GENERATOR = _REPO_ROOT / "tools" / "gen.py"`` -> skipif ссылается на
    ``_GENERATOR``, а каталог ``tools`` мы нашли через ``_REPO_ROOT``. Без
    транзитивного разрешения такой тест ошибочно попадал в FAIL, хотя на CI
    он молча скипается (severity WARN, не блокер).
    """
    candidates = {var}
    # транзитивно: NAME = <candidate> / ... (несколько уровней)
    for _ in range(3):
        grown = set(candidates)
        for cand in candidates:
            for m in re.finditer(
                rf"^\s*(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*\(?\s*"
                rf"{re.escape(cand)}\s*/",
                src,
                re.MULTILINE,
            ):
                grown.add(m.group("name"))
        if grown == candidates:
            break
        candidates = grown

    return any(
        re.search(rf"skipif\([^)]*{re.escape(c)}[^)]*\)", src) for c in candidates
    )


def scan_test_refs(repo: Path, known: set[str]) -> dict[str, list[tuple[str, bool]]]:
    """dir -> [(файл:строка, guarded_by_skipif)] для repo-root-anchored ссылок."""
    found: dict[str, list[tuple[str, bool]]] = {}
    for test_file in sorted(repo.glob("src/*/test/**/*.py")):
        try:
            src = test_file.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        want = depth_to_root(test_file, repo)
        env_anchored = bool(ENV_ROOT_RE.search(src))

        # переменные, указывающие на корень репо (или на env-якорь)
        root_vars: set[str] = set()
        for m in PARENTS_ASSIGN_RE.finditer(src):
            if int(m.group("n")) == want:
                root_vars.add(m.group("var"))
        for m in PARENTS_ITER_RE.finditer(src):
            # `for a in ...parents` перебирает ВСЕ ancestors, включая корень
            root_vars.add(m.group("var"))
        if env_anchored:
            for m in re.finditer(
                r"(?P<var>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*Path\(\s*_?override", src
            ):
                root_vars.add(m.group("var"))

        def _record(dname: str, pos: int, guard_var: str) -> None:
            if dname not in known:
                return
            lineno = src.count("\n", 0, pos) + 1
            loc = f"{test_file.relative_to(repo)}:{lineno}"
            entry = (loc, _skipif_guarded(src, guard_var))
            found.setdefault(dname, [])
            if entry not in found[dname]:
                found[dname].append(entry)

        # inline: Path(__file__)...parents[N] / "dir"
        for m in PARENTS_INLINE_RE.finditer(src):
            if int(m.group("n")) == want:
                _record(m.group("dir"), m.start(), m.group("dir"))

        # через переменную: ROOT / "dir"
        for var in root_vars:
            for m in re.finditer(
                rf"\b{re.escape(var)}\s*/\s*[\"'](?P<dir>[^\"'/]+)[\"']", src
            ):
                _record(m.group("dir"), m.start(), var)

        # env-якорь напрямую: environ[...ROB_BOX_REPO_ROOT...] ) / "dir"
        if env_anchored:
            for m in re.finditer(
                r"ROB_BOX_REPO_ROOT[^\n]*?\)\s*/\s*[\"'](?P<dir>[^\"'/]+)[\"']", src
            ):
                _record(m.group("dir"), m.start(), m.group("dir"))

    return found


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(add_help=True, description=__doc__)
    ap.add_argument("--workflow", default=DEFAULT_WORKFLOW)
    ap.add_argument("--repo", default=".")
    ap.add_argument(
        "--strict", action="store_true", help="WARN (skipif-guarded) тоже валит"
    )
    args = ap.parse_args(argv)

    repo = Path(args.repo).resolve()
    workflow = repo / args.workflow
    if not workflow.is_file():
        print(f"ERROR: workflow не найден: {workflow}", file=sys.stderr)
        return 2

    jobs = workflow_rsync_lists(workflow)
    if not jobs:
        print(
            f"ERROR: в {args.workflow} нет ни одной строки 'for d in ...; do' — "
            "структура workflow изменилась, обнови guard",
            file=sys.stderr,
        )
        return 2

    known = root_dirs(repo)
    refs = scan_test_refs(repo, known)

    fails: list[str] = []
    warns: list[str] = []
    for lineno, listed in jobs:
        for dname, sites in sorted(refs.items()):
            if dname in listed:
                continue
            hard = [loc for loc, guarded in sites if not guarded]
            soft = [loc for loc, guarded in sites if guarded]
            where = f"{args.workflow}:{lineno}"
            if hard:
                fails.append(
                    f"FAIL: '{dname}/' читается тестами от корня репо, но НЕ "
                    f"копируется в test_ws\n"
                    f"      workflow: {where}\n"
                    f"      список:   for d in {' '.join(sorted(listed))}; do\n"
                    f"      тесты:    {', '.join(hard[:3])}\n"
                    f"      фикс:     добавь '{dname}' в этот список"
                )
            elif soft:
                warns.append(
                    f"WARN: '{dname}/' не копируется в test_ws; обращение "
                    f"защищено skipif -> тест молча СКИПАЕТСЯ на CI\n"
                    f"      workflow: {where}\n"
                    f"      тесты:    {', '.join(soft[:3])}"
                )

    for block in fails:
        print(block)
        print()
    for block in warns:
        print(block)
        print()

    if fails:
        print("validate_test_ws_dirs: FAIL — CI упадёт collect-error'ом.")
        return 1
    if warns and args.strict:
        print("validate_test_ws_dirs: FAIL (--strict) — есть молча скипаемые тесты.")
        return 1

    print("validate_test_ws_dirs: OK")
    print(f"  job-списков проверено: {len(jobs)}")
    print(f"  корневых каталогов, читаемых тестами: {sorted(refs)}")
    if warns:
        print(f"  предупреждений (skipif-guarded): {len(warns)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
