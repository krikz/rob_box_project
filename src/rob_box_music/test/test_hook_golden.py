"""ADR-0154 PR-1, главный гард: ``hook.from_rtttl`` (теперь через ``from_notes``) даёт побайтно те же хуки, что
старый код, на ВСЁМ RTTTL-архиве репо — не на выборке. Эталон снят со старого кода до рефакторинга
(``gen_hook_golden.py``): отпечаток ``sha1(repr((Hook, Key)))`` либо причина отказа, по двум комбинациям
(темп, тоника, лад плана)."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

from gen_hook_golden import ARCHIVE, COMBOS, GOLDEN, archive_rows, fingerprint
from rob_box_music.arrange import hook as hooks


def test_golden_covers_the_whole_archive():
    with gzip.open(ARCHIVE, "rt", encoding="utf-8") as fh:
        n_archive = sum(1 for _ in fh)
    golden = json.loads(gzip.open(GOLDEN, "rt", encoding="utf-8").read())
    assert Path(GOLDEN).exists()
    assert [tuple(c) for c in golden["combos"]] == [tuple(c) for c in COMBOS]
    assert len(golden["records"]) == n_archive, (
        f"архив изменился ({n_archive} записей, в эталоне {len(golden['records'])}): пересоздать эталон "
        f"осознанно, python src/rob_box_music/test/gen_hook_golden.py, и показать diff счётчиков в PR")


def test_from_rtttl_gives_byte_identical_hooks_on_whole_archive():
    golden = json.loads(gzip.open(GOLDEN, "rt", encoding="utf-8").read())["records"]
    checked, diffs = 0, []
    for rid, rtttl in archive_rows():
        got = [fingerprint(hooks.from_rtttl, rtttl, rid, *combo) for combo in COMBOS]
        checked += len(got)
        diffs += [(rid, c, w, g) for c, w, g in zip(COMBOS, golden[rid], got) if w != g]
    print(f"сверено хуков: {checked} ({len(golden)} записей × {len(COMBOS)} комбинаций), расхождений: {len(diffs)}")
    assert not diffs, f"{len(diffs)} расхождений из {checked}, первые: {diffs[:3]}"
    assert checked == len(golden) * len(COMBOS)
