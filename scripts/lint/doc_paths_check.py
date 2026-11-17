#!/usr/bin/env python3
"""Гард протухших путей в живых документах (ADR-0166).

Находит в ``docs/**/*.md`` и ``README*.md`` упоминания путей ``scripts/…``,
``docker/…``, ``src/…``, ``.github/…`` (в бэктиках и в ссылках markdown) и
проверяет, что файл или каталог существует в репо.

Не проверяются: ``docs/adr/``, ``docs/research/``, ``docs/plans/``, ``docs/recon/``,
``docs/analysis/``, ``docs/investigations/``, ``docs/reports/``, ``docs/fixes/``,
``docs/design/``, ``docs/dialog/`` (история, пути на момент написания), ``docs/architecture/adr/``,
``docs/architecture/recon/``, строки с пометкой ``<!-- stale-paths: ignore -->``, файлы с
``<!-- stale-paths: ignore-file -->`` в первых строках
и разделы под заголовком с «Историческ» / «Historical» до следующего заголовка того же уровня.
Пути с ``*``, ``<``, ``{``, ``$``, ``...`` считаются шаблонами и пропускаются.

Usage:
  python -X utf8 scripts/lint/doc_paths_check.py            # отчёт, exit 1 при битых путях
  python -X utf8 scripts/lint/doc_paths_check.py --report   # отчёт, всегда exit 0
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PREFIXES = ("scripts/", "docker/", "src/", ".github/")
# История: пути на момент написания. Датированные планы, разборы, отчёты и дизайн-спеки
# не правим задним числом (ADR-0163, ADR-0166).
SKIP_DIRS = (
    "docs/adr/",
    "docs/research/",
    "docs/plans/",
    "docs/recon/",
    "docs/analysis/",
    "docs/investigations/",
    "docs/reports/",
    "docs/fixes/",
    "docs/design/",
    "docs/dialog/",
    "docs/architecture/adr/",
    "docs/architecture/recon/",
)
IGNORE_MARK = "stale-paths: ignore"
# Снимок на дату (аудит, разбор, план): метка в первых строках файла отключает проверку целиком.
IGNORE_FILE_MARK = "stale-paths: ignore-file"
# docker/build-push-action@v5 — это GitHub Action, а не путь.
ACTION_REF = re.compile(r"^docker/[\w-]+@")
# Имена-заготовки из примеров («my-service», «your_submodule»).
PLACEHOLDER = re.compile(r"my-service|my_service|your_submodule|some_service|^src/(package|submodule)/?$")
# Рантайм-файлы вне репозитория: секреты, .env, состояние студии, каталоги run/.
RUNTIME = re.compile(
    r"(^|/)(\.env(\.secrets|\.staging)?|state\.json|\.image-versions\.local)$|/run/|^docker/\w+/maps/?$"
)
HISTORY_HEADING = re.compile(r"^(#{1,6})\s+.*(историческ|historical)", re.IGNORECASE)
HEADING = re.compile(r"^(#{1,6})\s")
# Кандидат: префикс + символы пути. Слева — не часть другого пути/слова.
CANDIDATE = re.compile(r"(?<![\w./~-])((?:scripts|docker|src|\.github)/[\w@.+/\-*<>{}$%:]*[\w*/])")
TEMPLATE_CHARS = ("*", "<", ">", "{", "}", "$", "%", "...")


def doc_files() -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "*.md"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=True,
    ).stdout.split("\n")
    files = []
    for rel in out:
        rel = rel.strip()
        if not rel or rel.startswith(SKIP_DIRS):
            continue
        name = rel.rsplit("/", 1)[-1]
        if rel.startswith("docs/") or name.startswith("README"):
            files.append(ROOT / rel)
    return sorted(set(files))


def clean(raw: str) -> str:
    path = raw.split("#", 1)[0]
    # file.py:123 или file.py:12-30
    path = re.sub(r"(:\d+)+(-\d+)?$|:[A-Za-z_]\w*$", "", path)
    return path.rstrip(".:,;")


def exists(path: str, md: Path) -> bool:
    rel = path.rstrip("/")
    if (ROOT / rel).exists():
        return True
    # Имена workflow в тексте обрывают на пробеле: «.github/workflows/L-Build» = «L-Build All Services.yml».
    if rel.startswith(".github/workflows/") and "." not in rel.rsplit("/", 1)[-1]:
        wf = ROOT / ".github/workflows"
        if wf.is_dir() and any(p.name.startswith(rel.rsplit("/", 1)[-1]) for p in wf.iterdir()):
            return True
    # Неинициализированный сабмодуль: каталог есть, но пуст (src/vesc_nexus).
    top = ROOT / "/".join(rel.split("/")[:2])
    if top.is_dir() and not any(top.iterdir()):
        return True
    # Пути в README пакета часто даны от каталога пакета (src/scene/... в webxr_client).
    for parent in md.parents:
        if parent == ROOT.parent:
            break
        if (parent / rel).exists():
            return True
    return False


def check_file(md: Path) -> list[tuple[int, str]]:
    bad: list[tuple[int, str]] = []
    history_level = 0
    in_fence = False
    text = md.read_text(encoding="utf-8", errors="replace")
    if IGNORE_FILE_MARK in "\n".join(text.split("\n")[:12]):
        return bad
    for no, line in enumerate(text.split("\n"), 1):
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
        m = HEADING.match(line) if not in_fence else None
        if m:
            level = len(m.group(1))
            if HISTORY_HEADING.match(line):
                history_level = level
            elif history_level and level <= history_level:
                history_level = 0
        # Строки дерева каталогов (├── └── │) даны от своего корня, а не от корня репо.
        if history_level or IGNORE_MARK in line or line.lstrip().startswith(("├", "└", "│")):
            continue
        for found in CANDIDATE.finditer(line):
            path = clean(found.group(1))
            if not path.startswith(PREFIXES) or any(t in path for t in TEMPLATE_CHARS):
                continue
            if "/" not in path.rstrip("/"):
                continue
            if ACTION_REF.match(path) or RUNTIME.search(path) or PLACEHOLDER.search(path):
                continue
            if not exists(path, md):
                bad.append((no, path))
    return bad


def gitignored(paths: list[str]) -> set[str]:
    """Пути, которые .gitignore исключает (секреты, .env, рантайм-каталоги): их в репо быть не должно."""
    if not paths:
        return set()
    out = subprocess.run(
        ["git", "check-ignore", "--stdin"],
        cwd=ROOT,
        input="\n".join(paths),
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout
    return {p.strip() for p in out.split("\n") if p.strip()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="store_true", help="только отчёт, exit 0")
    args = ap.parse_args()
    found = [(md, no, path) for md in doc_files() for no, path in check_file(md)]
    ignored = gitignored(sorted({p.rstrip("/") for _, _, p in found}))
    total = 0
    for md, no, path in found:
        if path.rstrip("/") in ignored:
            continue
        print(f"{md.relative_to(ROOT).as_posix()}:{no}: {path}")
        total += 1
    print(f"doc_paths_check: битых путей {total}")
    return 0 if args.report or total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
