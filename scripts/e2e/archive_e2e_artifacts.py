#!/usr/bin/env python3
"""Архив артефактов прогонов `L-E2E Voice Test` из GitHub Actions (issue #4512).

logs/timing живут 7 дней, artifacts 14, model 30: после марафона их надо забрать.
Артефакт `recording` НЕ берём: там голоса людей. Архив в репо не кладём.

Раскладка: <root>/<run_id>/<имя артефакта>/... и рядом _artifact.json.
"""

from __future__ import annotations

import argparse
import io
import json
import re
import subprocess
import sys
import time
import zipfile
from pathlib import Path
from typing import Callable, Dict, List, Optional

REPO = "krikz/rob_box_project"
NAME_RE = re.compile(r"^e2e-voice-(logs|artifacts|timing|model|diff)-\d+$")
RETRIES = 4
RETRY_PAUSE_SEC = 3.0

# (args) -> raw stdout bytes; бросает RuntimeError при ненулевом коде gh
GhRunner = Callable[[List[str]], bytes]


def run_gh(args: List[str]) -> bytes:
    proc = subprocess.run(["gh"] + args, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode("utf-8", "replace").strip() or f"gh exit {proc.returncode}")
    return proc.stdout


def _with_retry(fn: Callable[[], object], sleep: Callable[[float], None], what: str):
    last: Optional[Exception] = None
    for attempt in range(1, RETRIES + 1):
        try:
            return fn()
        except (RuntimeError, ValueError) as exc:
            last = exc
            print(f"повтор {attempt}/{RETRIES} ({what}): {exc}", file=sys.stderr)
            sleep(RETRY_PAUSE_SEC)
    raise RuntimeError(f"{what}: {last}")


def wanted(art: Dict, since: Optional[str]) -> bool:
    if art.get("expired"):
        return False
    if not NAME_RE.match(art.get("name", "")):
        return False
    if since and str(art.get("created_at", ""))[:10] < since:
        return False
    return True


def list_artifacts(gh: GhRunner, since: Optional[str], sleep: Callable[[float], None] = time.sleep) -> List[Dict]:
    result: List[Dict] = []
    page = 1
    while True:
        endpoint = f"repos/{REPO}/actions/artifacts?per_page=100&page={page}"
        data = _with_retry(lambda: json.loads(gh(["api", endpoint]).decode("utf-8")), sleep, f"страница {page}")
        items = data.get("artifacts", [])
        if not items:
            break
        result.extend(a for a in items if wanted(a, since))
        if since and all(str(a.get("created_at", ""))[:10] < since for a in items):
            break  # список идёт от новых к старым
        page += 1
    return result


def target_dir(root: Path, art: Dict) -> Path:
    run_id = (art.get("workflow_run") or {}).get("id")
    return root / str(run_id) / art["name"]


def download(gh: GhRunner, art: Dict, root: Path, sleep: Callable[[float], None] = time.sleep) -> str:
    """Вернуть 'ok' или 'skip'; при ошибке бросить RuntimeError."""
    dest = target_dir(root, art)
    if dest.is_dir() and any(dest.iterdir()):
        return "skip"

    def fetch() -> bytes:
        raw = gh(["api", f"repos/{REPO}/actions/artifacts/{art['id']}/zip"])
        if not raw.startswith(b"PK"):
            raise ValueError("ответ не zip")
        return raw

    raw = _with_retry(fetch, sleep, f"артефакт {art['id']}")
    dest.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(raw)) as zf:
        zf.extractall(dest)
    wr = art.get("workflow_run") or {}
    meta = {
        "id": art["id"],
        "name": art["name"],
        "size_in_bytes": art.get("size_in_bytes"),
        "created_at": art.get("created_at"),
        "expires_at": art.get("expires_at"),
        "run_id": wr.get("id"),
        "head_branch": wr.get("head_branch"),
        "head_sha": wr.get("head_sha"),
    }
    (dest / "_artifact.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return "ok"


def archive(
    gh: GhRunner, root: Path, since: Optional[str], dry_run: bool, sleep: Callable[[float], None] = time.sleep
) -> int:
    arts = list_artifacts(gh, since, sleep)
    if dry_run:
        total = 0
        for art in arts:
            size = art.get("size_in_bytes") or 0
            total += size
            print(f"{art['id']} {art['name']} {size / 1048576:.1f} МБ {art.get('created_at', '')}")
        print(f"всего {len(arts)} артефактов, {total / 1048576:.1f} МБ")
        return 0
    counts = {"ok": 0, "skip": 0, "fail": 0}
    for art in arts:
        try:
            counts[download(gh, art, root, sleep)] += 1
        except (RuntimeError, OSError, zipfile.BadZipFile) as exc:
            counts["fail"] += 1
            print(f"FAIL {art['id']} {art['name']}: {exc}", file=sys.stderr)
    print(f"ok {counts['ok']} skip {counts['skip']} fail {counts['fail']}")
    return 1 if counts["fail"] else 0


def main(argv: Optional[List[str]] = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path, help="каталог архива (вне репо)")
    parser.add_argument("--since", help="YYYY-MM-DD: только артефакты не старше")
    parser.add_argument("--dry-run", action="store_true", help="только список и сумма МБ")
    args = parser.parse_args(argv)
    if args.since and not re.fullmatch(r"\d{4}-\d{2}-\d{2}", args.since):
        parser.error("--since: формат YYYY-MM-DD")
    return archive(run_gh, args.root, args.since, args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
