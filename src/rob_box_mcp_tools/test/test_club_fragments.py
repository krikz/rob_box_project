"""Тесты club_fragments (issue #3225, umbrella #3223): RTTTL-фрагмент как хук club-трека."""

import gzip
import json
import logging
import random
from pathlib import Path

import pytest

from rob_box_mcp_tools.core import club_fragments
from rob_box_mcp_tools.core.club_fragments import (
    FRAGMENT_BARS, FragmentUnavailable, fragment_pool, fragment_windows, hook_fingerprint, is_musical,
    pick_club_hook, pick_fragment,
)
from rob_box_mcp_tools.core.club_history import recent_club_rows, remember_club
from rob_box_mcp_tools.core.club_hook import extract_hook
from rob_box_mcp_tools.core.music_diversity import MusicHistory
from rob_box_mcp_tools.core.rtttl_catalog import add_melody
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary

MAJOR = (0, 2, 4, 5, 7, 9, 11)
BPM = 124


def _walk_rtttl(name: str, seed: int, notes: int = 96, base_octave: int = 5) -> str:
    """Сидированное блуждание по мажору: 12 тактов восьмыми, тактовые фразы различаются."""
    rng = random.Random(seed)
    letters = "cdefgab"
    degree, out = 2, []
    for _ in range(notes):
        degree = max(0, min(13, degree + rng.choice((-2, -1, -1, 1, 1, 2))))
        letter = letters[degree % 7]
        octave = base_octave + degree // 7
        out.append(f"8{letter}{octave}")
    return f"{name}:d=8,o=5,b={BPM}:" + ",".join(out)


def _make_library(tmp_path: Path, count: int = 8) -> RtttlLibrary:
    archive = tmp_path / "fixture.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for i in range(count):
            rec = {"name": f"fix{i}", "title": f"Fixture Tune {i}", "artist": "", "source": "test",
                   "tags": ["fixture"], "rtttl": _walk_rtttl(f"Fix{i}", 100 + i)}
            fh.write(json.dumps(rec) + "\n")
        # мусор: одна нота, и пауз-много — в пул попасть не должны / окон нет
        fh.write(json.dumps({"name": "beep", "title": "Beep", "tags": [], "rtttl": "Beep:d=4,o=5,b=124:c,c,c,c"}) + "\n")
    return RtttlLibrary(db_path=str(tmp_path / "lib.db"), archive_path=str(archive))


@pytest.fixture
def library(tmp_path):
    lib = _make_library(tmp_path)
    yield lib


def _transpose(rtttl: str, octaves: int) -> str:
    name, defaults, body = rtttl.split(":", 2)
    moved = [t[:-1] + str(int(t[-1]) + octaves) for t in body.split(",")]
    return f"{name}:{defaults}:{','.join(moved)}"


# ------------------------------------------------------------------ чистые функции
def test_is_musical_rejects_degenerate_windows():
    assert not is_musical(())
    assert not is_musical(((0, 4, 60),) * 8)  # одна высота
    assert not is_musical(((0, 4, 60), (4, 4, 61), (8, 4, 60), (12, 4, 61), (16, 4, 60), (20, 4, 61)))  # диапазон 1
    few = ((0, 4, 60), (4, 4, 64), (8, 4, 67))
    assert not is_musical(few)  # мало нот
    sparse = tuple((i * 8, 1, 60 + (i % 4) * 2) for i in range(6))  # 6 нот, но почти сплошные паузы
    assert not is_musical(sparse)
    good = tuple((i * 4, 3, 60 + (i % 5) * 2) for i in range(8))
    assert is_musical(good)


def test_fingerprint_is_transposition_invariant():
    base = _walk_rtttl("Base", 7)
    up = _transpose(base, 1)
    h0 = extract_hook(base, BPM, offset=16, bars=FRAGMENT_BARS)
    h1 = extract_hook(up, BPM, offset=16, bars=FRAGMENT_BARS)
    assert h0.notes != h1.notes  # ноты действительно другие
    assert hook_fingerprint(h0.notes) == hook_fingerprint(h1.notes) != ""
    other = extract_hook(_walk_rtttl("Other", 99), BPM, offset=16, bars=FRAGMENT_BARS)
    assert hook_fingerprint(other.notes) != hook_fingerprint(h0.notes)


def test_extract_hook_offset_zero_is_unchanged():
    rtttl = _walk_rtttl("Base", 3)
    assert extract_hook(rtttl, BPM) == extract_hook(rtttl, BPM, offset=0)
    shifted = extract_hook(rtttl, BPM, offset=32, bars=2)
    assert shifted.offset == 32 and shifted.notes and shifted.notes[0][0] >= 0
    assert all(s < 2 * 16 for s, _l, _m in shifted.notes)


def test_fragment_windows_have_nonzero_offsets_and_are_musical():
    windows = fragment_windows(_walk_rtttl("Base", 5), BPM, melody_id="base")
    offsets = [w.offset for w in windows]
    assert len(windows) >= 4 and any(o > 0 for o in offsets)
    assert all(o % 16 == 0 and is_musical(w.notes) for o, w in zip(offsets, windows))


def test_pool_filters_garbage_and_caches(library, monkeypatch):
    pool = fragment_pool(library)
    assert len(pool) == 8  # beep (4 одинаковые ноты) отсеян по качеству
    calls = []
    original = club_fragments.iter_melodies
    monkeypatch.setattr(club_fragments, "iter_melodies", lambda *a, **k: (calls.append(1), original(*a, **k))[1])
    assert fragment_pool(library) == pool and not calls  # повторный вызов — из кэша


# ------------------------------------------------------------------ выбор + история
def test_ten_calls_give_ten_distinct_melody_offset_pairs(library, tmp_path):
    history = MusicHistory(str(tmp_path / "hist.db"))
    seen = []
    for _ in range(10):
        recent = recent_club_rows(history)
        frag = pick_fragment(library, BPM, seed=0, recent=recent)
        assert frag.hook.notes and is_musical(frag.hook.notes)
        history.record(style="club", melody_name=frag.melody, fragment_offset=frag.offset,
                       hook_fingerprint=frag.fingerprint)
        seen.append((frag.melody, frag.offset))
    assert len(set(seen)) == 10, seen
    assert any(off > 0 for _m, off in seen)


def test_restart_keeps_avoiding_history(library, tmp_path):
    path = str(tmp_path / "hist.db")
    first = MusicHistory(path)
    picked = []
    for _ in range(4):
        frag = pick_fragment(library, BPM, seed=5, recent=recent_club_rows(first))
        first.record(style="club", melody_name=frag.melody, fragment_offset=frag.offset,
                     hook_fingerprint=frag.fingerprint)
        picked.append((frag.melody, frag.offset))
    first.close()
    second = MusicHistory(path)  # «перезапуск»
    for _ in range(4):
        frag = pick_fragment(library, BPM, seed=5, recent=recent_club_rows(second))
        second.record(style="club", melody_name=frag.melody, fragment_offset=frag.offset,
                      hook_fingerprint=frag.fingerprint)
        picked.append((frag.melody, frag.offset))
    assert len(set(picked)) == 8, picked


def test_deterministic_for_same_seed_and_history(library):
    a = pick_fragment(library, BPM, seed=11, recent=[])
    b = pick_fragment(library, BPM, seed=11, recent=[])
    assert (a.melody, a.offset, a.fingerprint) == (b.melody, b.offset, b.fingerprint)


def test_recent_fingerprint_is_avoided(library):
    frag = pick_fragment(library, BPM, seed=1, recent=[])
    recent = [{"id": 1, "melody_name": "other", "fragment_offset": 0, "hook_fingerprint": frag.fingerprint}]
    for seed in range(20):
        assert pick_fragment(library, BPM, seed=seed, recent=recent).fingerprint != frag.fingerprint


def test_theme_search_wins_then_falls_back(library):
    hit = pick_fragment(library, BPM, seed=1, recent=[], theme="Fixture Tune 3")
    assert hit.source == "theme" and hit.melody == "fix3"
    miss = pick_fragment(library, BPM, seed=1, recent=[], theme="qqqzzz нет такой темы")
    assert miss.source == "pool"


def test_remember_club_writes_offset_and_fingerprint(library):
    history = MusicHistory(":memory:")
    hook, info = pick_club_hook(library, BPM, 0, [])
    assert hook is not None and info["source"] == "fragment"
    remember_club(history, {}, {"template": "t", "kick": "k", "hats": "h", "lead": "l", "bass": "b", "pad": "p"},
                  0, BPM, info, [])
    row = history.recent()[0]
    assert (row["melody_name"], row["fragment_offset"], row["hook_fingerprint"]) == (
        info["id"], info["offset"], info["fingerprint"])
    assert row["hook_fingerprint"]


# ------------------------------------------------------------------ фолбек
def test_fallback_without_library_warns(caplog):
    with caplog.at_level(logging.WARNING, logger=club_fragments.__name__):
        assert pick_club_hook(None, BPM, 0, []) == (None, None)
    assert any("pentatonic-fallback" in r.getMessage() and r.levelno == logging.WARNING for r in caplog.records)


def test_fallback_on_broken_library_warns_and_uses_custom_sink():
    class Broken:
        @property
        def _lock(self):
            raise RuntimeError("db locked")

    messages = []
    assert pick_club_hook(Broken(), BPM, 0, [], warn=messages.append) == (None, None)
    assert len(messages) == 1 and "db locked" in messages[0] and "pentatonic-fallback" in messages[0]


def test_empty_archive_raises_unavailable(tmp_path):
    archive = tmp_path / "empty.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        fh.write("")
    empty = RtttlLibrary(db_path=str(tmp_path / "e.db"), archive_path=str(archive))
    with pytest.raises(FragmentUnavailable):
        pick_fragment(empty, BPM)
    messages = []
    assert pick_club_hook(empty, BPM, 0, [], warn=messages.append) == (None, None) and messages


# ------------------------------------------------------------------ библиотека: добавление
def test_library_add_melody_with_source_and_tags(library):
    rtttl = _walk_rtttl("Ext", 500)
    assert add_melody(library, rtttl, "ext_one", title="External One", source="dj-web", tags=["web", "dendy"])
    assert not add_melody(library, rtttl, "ext_one")  # дубликат RTTTL не пишется
    rec = library.get("ext_one")
    assert rec["source"] == "dj-web" and rec["tags"] == ["web", "dendy"] and rec["rtttl"] == rtttl
