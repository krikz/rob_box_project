"""Снимок «эталон хуков» по всему RTTTL-архиву репо (ADR-0154 PR-1).

``python src/rob_box_music/test/gen_hook_golden.py`` пересоздаёт ``data/hook_golden.json.gz``. Эталон снят со
старого ``hook.from_rtttl`` ДО рефакторинга на ``from_notes``; ``test_hook_golden.py`` сверяет текущий код с ним.
Пересоздавать — только осознанно (правка алгоритма хука или архива) и с diff-ом счётчиков в PR.
"""

from __future__ import annotations

import gzip
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARCHIVE = HERE.parents[1] / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
GOLDEN = HERE / "data" / "hook_golden.json.gz"
#: (bpm, root, mode): минор и мажор плана, разные темпы (time_scale) и тоники (track_key).
COMBOS = ((132, 9, "minor"), (100, 0, "major"))


def archive_rows():
    with gzip.open(ARCHIVE, "rt", encoding="utf-8") as fh:
        for i, line in enumerate(fh):
            row = json.loads(line)
            yield f"{i}:{row['name']}", row["rtttl"]


def fingerprint(hook_fn, rtttl: str, melody_id: str, bpm: int, root: int, mode: str) -> str:
    """``ok:<sha1 repr(Hook, Key)>`` или ``err:<причина>`` — отказ тоже часть контракта."""
    from rob_box_music.arrange.hook import HookError
    try:
        hook, key = hook_fn(rtttl, melody_id, bpm, root, mode)
    except HookError as exc:
        return f"err:{exc}"
    return "ok:" + hashlib.sha1(repr((hook, key)).encode("utf-8")).hexdigest()[:12]


def snapshot(hook_fn) -> dict:
    out = {}
    for rid, rtttl in archive_rows():
        out[rid] = [fingerprint(hook_fn, rtttl, rid, *combo) for combo in COMBOS]
    return out


if __name__ == "__main__":
    from rob_box_music.arrange import hook as hooks
    snap = snapshot(hooks.from_rtttl)
    GOLDEN.parent.mkdir(exist_ok=True)
    with gzip.open(GOLDEN, "wt", encoding="utf-8", compresslevel=9) as fh:
        json.dump({"combos": COMBOS, "records": snap}, fh, ensure_ascii=False, separators=(",", ":"))
    ok = sum(v.startswith("ok:") for fp in snap.values() for v in fp)
    print(f"записей: {len(snap)}, комбинаций: {len(COMBOS)}, ok: {ok}, err: {len(snap) * len(COMBOS) - ok}",
          file=sys.stderr)
