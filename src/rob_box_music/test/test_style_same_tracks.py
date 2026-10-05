"""ADR-0153 S0: перенос клубных таблиц в ``knowledge.STYLES["club"]`` не меняет ни одного трека.

Эталон ``data/style_s0_golden.json`` снят на ``origin/develop`` @ ``a138b8c7c`` ДО переноса (``python
test_style_same_tracks.py --write``): на каждый сид × тему × трек сета — sha256 канонического вида ``Track`` и
текста программы Renardo. Тот же код после переноса обязан дать побайтно те же значения.

Канонический вид: dataclass → (имя, поля по порядку), множества — отсортированы (``repr(frozenset)`` зависит от
``PYTHONHASHSEED``), словари — в порядке вставки (порядок ролей виден рендеру).
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from melodies import MELODIES  # noqa: E402
from rob_box_music.arrange.compose import club_track, compose  # noqa: E402
from rob_box_music.arrange.song import SongMaterial, song_track  # noqa: E402
from rob_box_music.diversity import track_history  # noqa: E402
from rob_box_music.render.renardo import render  # noqa: E402
from rob_box_music.set_plan import seeded_plan  # noqa: E402
from rob_box_music.theme import seeded_profile  # noqa: E402

GOLDEN = Path(__file__).resolve().parent / "data" / "style_s0_golden.json"
SEEDS = range(50)
#: Темы: строки таблицы (окно темпа и лад строки, семья тембров) и тема не из таблицы (окно стиля, лад по хешу).
THEMES = ("космос", "киберпанк", "детский праздник", "калинка", "бухгалтерский отчёт")
HOOKS = tuple(MELODIES)
TRACKS = 3
SONG_LEAD = ((69, 0.75), (72, 0.375), (71, 0.125), (None, 0.75), (76, 2.0), (74, 1.0), (72, 1.0), (69, 2.0))


def _canon(value):
    text = repr(value)
    if "{" not in text and "frozenset(" not in text:
        return text  # лист без множеств и словарей (ноты, шаги сетки): repr от сида процесса не зависит
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return [type(value).__name__] + [[f.name, _canon(getattr(value, f.name))] for f in dataclasses.fields(value)]
    if isinstance(value, (set, frozenset)):
        return sorted(repr(_canon(v)) for v in value)
    if isinstance(value, dict) or hasattr(value, "items"):
        return [[_canon(k), _canon(v)] for k, v in value.items()]
    if isinstance(value, (list, tuple)):
        return [_canon(v) for v in value]
    return text


def _digest(track, deck: str) -> str:
    program = render(track, deck)
    body = json.dumps([_canon(track), program.code, _canon(program)], ensure_ascii=False)
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def digests() -> dict:
    """``{случай: sha256}``: сеты по темам с историей прошлых треков, трек без темы, песня."""
    out = {}
    for seed in SEEDS:
        for theme in THEMES:
            profile = dataclasses.replace(seeded_profile(theme), hook_ids=HOOKS)
            plan = seeded_plan(profile, seed, set_id=f"g{seed}")
            history: list = []
            for no in range(1, TRACKS + 1):
                deck = "AB"[no % 2]
                track = compose(plan, no, melodies=MELODIES, history=history, deck=deck)
                out[f"{seed}:{theme}:{no}"] = _digest(track, deck)
                history.insert(0, track_history(track, plan.set_id))
        out[f"{seed}:club_track"] = _digest(club_track(seed), "A")
        material = SongMaterial(melody_id="am", title="Ля", bpm=96, root=9, mode="minor", lead=SONG_LEAD,
                                bass=((45, 2.0),) * 4, pad=(((57, 60, 64), 2.0),) * 4, pad_sus=0.4,
                                drums="X...o...X...o...", hats="-.-.-.-.-.-.-.-.")
        out[f"{seed}:song"] = _digest(song_track(material, seed=seed), "A")
    return out


@pytest.fixture(scope="module")
def actual() -> dict:
    return digests()


def test_golden_covers_every_case(actual):
    assert set(json.loads(GOLDEN.read_text(encoding="utf-8"))) == set(actual)


def test_same_tracks_as_before_style_table(actual):
    """Каждый трек и его программа — побайтно как до переноса в ``STYLES`` (ADR-0153 §6 S0)."""
    golden = json.loads(GOLDEN.read_text(encoding="utf-8"))
    changed = sorted(case for case, sha in golden.items() if actual.get(case) != sha)
    assert not changed, f"{len(changed)}/{len(golden)} треков изменились: {changed[:10]}"


if __name__ == "__main__" and "--write" in sys.argv:
    GOLDEN.parent.mkdir(exist_ok=True)
    GOLDEN.write_text(json.dumps(digests(), ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
