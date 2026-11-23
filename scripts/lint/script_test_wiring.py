#!/usr/bin/env python3
"""Гард «тест написан, но не в CI» для двух каталогов (issue #5176, ADR-0057).

``tests/unit/scripts/`` и ``tests/unit/e2e_scripts/`` ни один workflow не гоняет
целиком: там есть файлы, которым нужен робот или сеть (``G-Lint Code.yml``,
комментарий у шага issue #2846). Новые ``test_*.py`` подключаются поштучно.
Без гарда откат фикса проходит CI незамеченным — так прошли #5170 и #5077.

Правило для ``test_*.py`` этих каталогов:

* имя файла встречается в каком-нибудь ``.github/workflows/*.yml`` — ок;
* либо в ``scripts/lint/script_test_wiring_allow.json`` есть запись с непустой
  причиной — ок (файл гоняется вручную или на роботе, и это сказано явно);
* либо файл уже лежал на ``origin/develop`` и там ни один workflow его
  не называл — ок. Таких файлов 123, и список причин на каждый раздул бы
  PR; их разбор — отдельная карточка. Файл, который на develop уже
  подключён, при удалении строки из workflow краснеет;
* иначе exit 1.

Сравнение с develop идёт через ``git ls-tree``: гард читает только имена,
дерево не копирует. Пустая причина в allowlist — ошибка загрузки
(ADR-0018), а не пропуск.

    python scripts/lint/script_test_wiring.py [--ref origin/develop]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Optional, Sequence

REPO = Path(__file__).resolve().parents[2]
ALLOW = Path(__file__).resolve().with_name("script_test_wiring_allow.json")
DIRS = ("tests/unit/scripts", "tests/unit/e2e_scripts")
WF_DIR = REPO / ".github" / "workflows"


def _git(args: Sequence[str], cwd: Path = REPO) -> str:
    out = subprocess.run(
        ["git", "-C", str(cwd), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return out.stdout


def ensure_ref(ref: str, cwd: Path = REPO) -> None:
    """Подтянуть ``ref``, если его ещё нет локально.

    CI чекаутит SHA, а не ветку: без fetch ``origin/develop`` может
    отсутствовать, и гард упал бы ошибкой git, а не вердиктом. Fetch
    узкий (одна ветка), сеть нужна только когда ref не резолвится.
    """
    probe = subprocess.run(
        ["git", "-C", str(cwd), "rev-parse", "--verify", "--quiet", ref],
        capture_output=True,
        text=True,
    )
    if probe.returncode == 0:
        return
    if ref == "origin/develop":
        subprocess.run(
            ["git", "-C", str(cwd), "fetch", "--no-tags", "origin", "develop"],
            check=True,
        )
        return
    raise subprocess.CalledProcessError(probe.returncode, ["git", "rev-parse", ref])


def _ref_exists(ref: str) -> bool:
    probe = subprocess.run(
        ["git", "-C", str(REPO), "rev-parse", "--verify", "--quiet", ref],
        capture_output=True,
    )
    return probe.returncode == 0


def names_at(ref: str) -> set[str]:
    """Имена ``test_*.py`` этих двух каталогов на ``ref`` (коммит, не дерево)."""
    found: set[str] = set()
    for directory in DIRS:
        listing = _git(["ls-tree", "-r", "--name-only", ref, "--", directory])
        for line in listing.splitlines():
            name = Path(line).name
            if name.startswith("test_") and name.endswith(".py"):
                found.add(name)
    return found


def workflow_text_at(ref: str) -> str:
    """Текст workflow на ``ref``: по нему видно, был ли файл подключён."""
    listing = _git(["ls-tree", "-r", "--name-only", ref, "--", ".github/workflows"])
    chunks = []
    for path in listing.splitlines():
        if path.endswith((".yml", ".yaml")):
            chunks.append(_git(["show", f"{ref}:{path}"]))
    return "\n".join(chunks)


def names_in_tree(root: Path = REPO) -> set[str]:
    found: set[str] = set()
    for directory in DIRS:
        base = root / directory
        if not base.is_dir():
            continue
        for path in base.glob("test_*.py"):
            found.add(path.name)
    return found


def workflow_text(root: Path = REPO) -> str:
    chunks = []
    for path in sorted(WF_DIR.glob("*.yml")) + sorted(WF_DIR.glob("*.yaml")):
        chunks.append(path.read_text(encoding="utf-8"))
    return "\n".join(chunks)


def load_allow(path: Path = ALLOW) -> dict[str, str]:
    """``{имя файла: причина}``. Пустая причина — ошибка загрузки, не пропуск."""
    if not path.is_file():
        return {}
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: ожидался объект имя→причина, пришло {type(raw).__name__}")
    allow: dict[str, str] = {}
    for name, reason in raw.items():
        if not isinstance(name, str) or not name.startswith("test_") or not name.endswith(".py"):
            raise ValueError(f"{path}: ключ {name!r} — не имя test_*.py")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError(f"{path}: у {name} пустая причина (ADR-0018: молча не пропускаем)")
        allow[name] = reason.strip()
    return allow


def grandfathered(develop_ref: str) -> set[str]:
    """Файлы, которые на develop ни один workflow не называл.

    Это исторический хвост (123 файла на момент #5176). Причины на каждый
    выдумывать нельзя — отдельная карточка. После мёржа этого гарда из
    хвоста вычитается всё, что HEAD уже подключил: откат строки краснеет.
    """
    ref_text = workflow_text_at(develop_ref)
    exempt = {name for name in names_at(develop_ref) if name not in ref_text}
    if _ref_exists("HEAD") and "script_test_wiring.py" in workflow_text_at("HEAD"):
        wired_now = {name for name in names_at("HEAD") if name in workflow_text_at("HEAD")}
        exempt -= wired_now
    return exempt


def unwired(develop_ref: str, root: Path = REPO, allow_path: Path = ALLOW) -> list[str]:
    """``test_*.py`` без упоминания в workflow и без причины в allowlist."""
    present = names_in_tree(root)
    exempt = grandfathered(develop_ref)
    text = workflow_text(root)
    allow = load_allow(allow_path)
    return [name for name in sorted(present) if name not in text and name not in allow and name not in exempt]


def check(ref: str, root: Path = REPO, allow_path: Path = ALLOW) -> tuple[int, str]:
    missing = unwired(ref, root, allow_path)
    if missing:
        listed = "\n".join(f"  - {name}" for name in missing)
        return 1, (
            "FAIL: новый test_*.py в tests/unit/scripts или tests/unit/e2e_scripts "
            "не подключён ни к одному .github/workflows/*.yml и не назван в "
            f"scripts/lint/script_test_wiring_allow.json с причиной (issue #5176):\n{listed}\n"
            "Каталоги целиком не гоняются: там есть тесты робота и сети. "
            "Подключи файл явным шагом или запиши причину, почему он гоняется вручную."
        )
    return 0, f"OK: новые test_*.py в {', '.join(DIRS)} либо в workflow, либо в allowlist ({ref})."


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ref", default="origin/develop")
    ap.add_argument("--allow", type=Path, default=ALLOW)
    args = ap.parse_args(argv)
    try:
        ensure_ref(args.ref)
        code, message = check(args.ref, allow_path=args.allow)
    except (subprocess.CalledProcessError, ValueError, json.JSONDecodeError) as exc:
        print(f"FAIL: гард не смог провериться: {exc}", file=sys.stderr)
        return 1
    print(message, file=sys.stderr if code else sys.stdout)
    return code


if __name__ == "__main__":
    sys.exit(main())
