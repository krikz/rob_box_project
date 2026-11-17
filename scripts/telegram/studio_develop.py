"""Read-only worktree на origin/develop — источник фактов и медиа для студии.

Зачем. Клон студии живёт на рабочей ветке (`z-architect/...`), и свежие материалы,
влитые в develop, в ней не видны: `docs/channel/media/locdrift-before-after.png`
из #4508 в рабочей ветке отсутствует. Факты и медиа читаются поэтому из отдельного
worktree, который каждый раз приводится к `origin/develop` и больше нигде не меняется.

Правила (требование координатора, 06.11):
  * каждая сессия начинается с `git fetch origin develop`;
  * worktree — detached, то есть он НЕ ветка: ни одна правка студии в него не попадёт;
  * ничего из этого worktree студия не пишет — только читает.

Путь: `STUDIO_DEVELOP_WORKTREE` или `../rob_box_develop` рядом с клоном.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Optional


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def develop_worktree_path(root: Optional[Path] = None) -> Path:
    """Путь к worktree с develop."""
    root = root or repo_root()
    override = os.environ.get("STUDIO_DEVELOP_WORKTREE")
    if override:
        return Path(override).expanduser().resolve()
    return (root.parent / "rob_box_develop").resolve()


def _git(args: list, cwd: Path, timeout: int = 180) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", *args],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def ensure_develop_worktree(root: Optional[Path] = None, out=print) -> Optional[Path]:
    """Привести worktree к origin/develop. None — если не получилось.

    Порядок: fetch → создать worktree, если его нет → checkout --detach origin/develop.
    Checkout detached важен: ветка в worktree означала бы, что студия может
    случайно закоммитить туда что-то своё.
    """
    root = root or repo_root()
    path = develop_worktree_path(root)

    fetched = _git(["fetch", "origin", "develop"], cwd=root)
    if fetched.returncode != 0:
        out(f"[studio-develop] fetch не удался: {fetched.stderr.strip()[:160]}")
        return None

    if not (path / ".git").exists():
        if path.exists() and any(path.iterdir()):
            out(f"[studio-develop] {path} есть, но не git-worktree — не трогаю")
            return None
        added = _git(["worktree", "add", "--detach", str(path), "origin/develop"], cwd=root)
        if added.returncode != 0:
            out(f"[studio-develop] worktree не создан: {added.stderr.strip()[:160]}")
            return None
        out(f"[studio-develop] worktree создан: {path}")
        return path

    # reset --hard и clean -fd — только в отдельном worktree. Если путь указывает на сам клон
    # студии (или на обычный клон, где .git — каталог), сброс стёр бы рабочую ветку и state.
    if path == root.resolve() or not (path / ".git").is_file():
        out(f"[studio-develop] {path} — не отдельный worktree (это клон или .git-каталог) — не трогаю")
        return None
    checked = _git(["checkout", "--detach", "origin/develop"], cwd=path)
    if checked.returncode != 0:
        out(f"[studio-develop] checkout не удался: {checked.stderr.strip()[:160]}")
        return None
    # Мусор в worktree (сгенерированные файлы) сбрасываем: он должен быть эталоном develop.
    _git(["reset", "--hard", "origin/develop"], cwd=path)
    _git(["clean", "-fd"], cwd=path)
    out(f"[studio-develop] worktree на origin/develop: {path}")
    return path


def develop_head(path: Optional[Path] = None) -> str:
    """SHA, на котором стоит worktree develop — для provenance в фактах."""
    target = path or develop_worktree_path()
    if not (target / ".git").exists():
        return ""
    done = _git(["rev-parse", "--short", "HEAD"], cwd=target)
    return done.stdout.strip() if done.returncode == 0 else ""


def main() -> int:
    root = repo_root()
    path = ensure_develop_worktree(root)
    if path is None:
        return 1
    print(f"  develop: {path} @ {develop_head(path)}")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
