"""Тесты music_diversity (issue #3224, ADR-0146): память о сыгранном и штраф за недавнее."""

import hashlib
import logging
import random
import sqlite3
import time
from collections import Counter
from pathlib import Path

import pytest

from rob_box_mcp_tools.core import music_diversity
from rob_box_mcp_tools.core.club_arranger import (
    CLUB_TEMPLATES, HATS_PATTERNS, KICK_PATTERNS, PROGRESSIONS, REFERENCE_KIT, ROLE_SYNTHS,
    club_kit, club_progression, render_club,
)
from rob_box_mcp_tools.core.music_diversity import MusicHistory, weighted_pick

KIT_ROLES = ("template", "kick", "hats", "lead", "bass", "pad")


# ---------------------------------------------------------------- weighted_pick
def test_weighted_pick_deterministic_for_same_rng_state():
    options = ["a", "b", "c", "d"]
    recent = ["a", "b"]
    first = [weighted_pick(options, recent, random.Random(s)) for s in range(30)]
    second = [weighted_pick(options, recent, random.Random(s)) for s in range(30)]
    assert first == second


def test_weighted_pick_empty_history_is_uniform_enough():
    rng = random.Random(1)
    counts = Counter(weighted_pick(["a", "b", "c"], [], rng) for _ in range(3000))
    assert all(800 < counts[k] < 1200 for k in "abc"), counts


def test_weighted_pick_floor_keeps_choice_possible():
    """Все опции недавно играли — выбор всё равно возможен, все опции достижимы."""
    recent = ["a", "b", "a", "b", "a", "b"]
    rng = random.Random(2)
    picked = {weighted_pick(["a", "b"], recent, rng, floor=0.2) for _ in range(500)}
    assert picked == {"a", "b"}


def test_weighted_pick_fresh_repeat_penalised_more_than_old():
    recent = ["a", "x", "x", "x", "b"]  # a — самый свежий, b — самый старый
    rng = random.Random(3)
    counts = Counter(weighted_pick(["a", "b"], recent, rng) for _ in range(4000))
    assert counts["a"] < counts["b"] / 3, counts


def test_weighted_pick_rejects_bad_arguments():
    with pytest.raises(ValueError):
        weighted_pick([], [], random.Random(0))
    with pytest.raises(ValueError):
        weighted_pick(["a"], [], random.Random(0), floor=0.0)
    with pytest.raises(ValueError):
        weighted_pick(["a"], [], random.Random(0), decay=1.5)


# ---------------------------------------------------------------- MusicHistory
def test_history_memory_record_and_recent_newest_first():
    hist = MusicHistory(":memory:")
    assert hist.available
    for i, prog in enumerate(("p1", "p2", "p3")):
        assert hist.record(progression=prog, bpm=124, ts=1000 + i)
    rows = hist.recent(limit=2)
    assert [r["progression"] for r in rows] == ["p3", "p2"]
    assert rows[0]["bpm"] == 124 and rows[0]["set_id"] is None


def test_history_within_sec_filters_old_rows():
    hist = MusicHistory(":memory:")
    hist.record(progression="old", ts=time.time() - 3600)
    hist.record(progression="new")
    assert [r["progression"] for r in hist.recent(within_sec=60)] == ["new"]
    assert len(hist.recent()) == 2


def test_history_rejects_unknown_field():
    with pytest.raises(TypeError):
        MusicHistory(":memory:").record(progresion="typo")


def test_history_survives_restart_on_file(tmp_path):
    db = str(tmp_path / "voice_memory.db")
    first = MusicHistory(db)
    first.record(style="club", progression="VI-III-VII-i", template="dj_dave_32", melody_name="m1")
    first.close()
    second = MusicHistory(db)  # «перезапуск»: новый объект, тот же файл
    rows = second.recent()
    assert len(rows) == 1 and rows[0]["progression"] == "VI-III-VII-i" and rows[0]["melody_name"] == "m1"


def test_history_unavailable_db_warns_and_degrades_loudly(tmp_path, caplog):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    with caplog.at_level(logging.WARNING, logger=music_diversity.__name__):
        hist = MusicHistory(str(blocker / "sub" / "db.sqlite"))
    assert not hist.available
    assert any("недоступна" in r.message for r in caplog.records)
    assert hist.record(progression="p") is False
    assert hist.recent() == []


def test_history_coexists_with_other_tables_in_same_db(tmp_path):
    db = str(tmp_path / "voice_memory.db")
    conn = sqlite3.connect(db)
    conn.execute("CREATE TABLE rtttl_melodies (id INTEGER PRIMARY KEY)")
    conn.commit()
    conn.close()
    hist = MusicHistory(db)
    hist.record(progression="p")
    assert len(hist.recent()) == 1


# ---------------------------------------------------------------- club integration
def _play(hist_rows, seed):
    """Один «трек»: выбор каркаса и прогрессии с историей, запись в историю (свежие первыми)."""
    kit = club_kit(seed, recent=hist_rows)
    prog = club_progression(seed, hist_rows)
    hist_rows.insert(0, dict(kit, progression=prog))
    return kit, prog


@pytest.mark.parametrize("base", [1000, 6261502, 7690914])
def test_ten_tracks_with_shared_history_are_diverse(base):
    """Критерий #3224: ни одна прогрессия > 3 из 10, одинакового каркаса подряд нет."""
    rows = []
    played = [_play(rows, base + i) for i in range(10)]
    counts = Counter(prog for _, prog in played)
    assert max(counts.values()) <= 3, counts
    for (prev, _), (cur, _) in zip(played, played[1:]):
        assert prev != cur


def test_history_after_restart_penalises_recent_progression(tmp_path):
    db = str(tmp_path / "h.db")
    seed = 6261504
    first = MusicHistory(db)
    prog0 = club_progression(seed)
    first.record(progression=prog0, **club_kit(seed))
    first.close()
    rows = MusicHistory(db).recent()  # после «перезапуска»
    share = sum(club_progression(s, rows) == prog0 for s in range(2000, 2400)) / 400
    assert share < 1 / len(PROGRESSIONS)
    assert club_kit(seed, recent=rows) != club_kit(seed)


def test_same_seed_twice_in_a_row_gives_different_kit():
    rows = []
    a, _ = _play(rows, 6261504)
    b, _ = _play(rows, 6261504)
    assert a != b


def test_explicit_template_and_kick_beat_history():
    rows = [dict(zip(KIT_ROLES, ("dj_dave_32", "breakbeat", "offbeat", "pluck", "bass", "sinepad")))] * 5
    kit = club_kit(9, template="long_build_32", kick="four_on_floor", recent=rows)
    assert kit["template"] == "long_build_32" and kit["kick"] == "four_on_floor"


def test_seed_zero_ignores_history():
    rows = [dict(REFERENCE_KIT, progression=PROGRESSIONS[0][0])] * 5
    assert club_kit(0, recent=rows) == REFERENCE_KIT
    assert render_club(seed=0, recent=rows) == render_club(seed=0)


def test_seed0_snapshot_unchanged_with_history_argument():
    fixture = Path(__file__).parent / "fixtures" / "club_arranger_seed0.foxdot"
    expected = fixture.read_text(encoding="utf-8")
    assert render_club(seed=0).strip() == expected.strip()
    assert render_club(seed=0, recent=[dict(REFERENCE_KIT)]).strip() == expected.strip()


#: sha256[:16] render_club(seed=s) на develop ДО правки #3224 (снято со старого модуля).
_BASELINE = {
    0: "b6a32c04ab1c8c66", 1: "9356ed6c412cc5c9", 7: "405aa492cfd4d514",
    42: "8493f3b3560e071c", 6261504: "b0fa778a1de857dd",
}


@pytest.mark.parametrize("seed", sorted(_BASELINE))
@pytest.mark.parametrize("recent", [None, []])
def test_no_history_is_byte_identical_to_old_behaviour(seed, recent):
    code = render_club(seed=seed, recent=recent)
    assert hashlib.sha256(code.encode()).hexdigest()[:16] == _BASELINE[seed]


def test_no_history_kit_matches_old_rng_recipe():
    """club_kit без истории — прежний рецепт random.Random(f"club-kit:{seed}").choice."""
    for seed in (1, 5, 99, 6261505):
        rng = random.Random(f"club-kit:{seed}")
        old = {
            "template": rng.choice(CLUB_TEMPLATES), "kick": rng.choice(sorted(KICK_PATTERNS)),
            "hats": rng.choice(sorted(HATS_PATTERNS)), "lead": rng.choice(ROLE_SYNTHS["lead"]),
            "bass": rng.choice(ROLE_SYNTHS["bass"]), "pad": rng.choice(ROLE_SYNTHS["pad"]),
        }
        assert club_kit(seed) == old
        assert club_kit(seed, recent=[]) == old


def test_render_club_with_history_is_deterministic():
    rows = [dict(REFERENCE_KIT, progression=PROGRESSIONS[0][0])]
    a = render_club(seed=11, recent=rows)
    assert a == render_club(seed=11, recent=rows)
    assert "p1 >>" in a
