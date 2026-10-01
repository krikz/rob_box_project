"""PR-3a ADR-0149 §4.7: тема → профиль без LLM; хуки темы есть в локальной RTTTL-библиотеке и звучат.

Вторая половина теста открывает архив ``rob_box_mcp_tools/data/rtttl_melodies.jsonl.gz`` штатной
``RtttlLibrary`` (во временной SQLite) — как его откроет движок плеера. Пакет ставится отдельно от
монорепо — пропуск (как ``test_knowledge_legacy``).
"""

from __future__ import annotations

import pathlib
import sys

import pytest

from melodies import compose_p
from rob_box_music import knowledge as kn
from rob_box_music.theme import match_row, seeded_profile

#: Темы серии приёмки (ADR-0149 §7.1) и ещё одна → строка таблицы.
THEMES = {
    "космос": "space", "космическая дискотека": "space", "киберпанк": "cyber", "роботы будущего": "cyber",
    "детский праздник": "kids", "славянская вечеринка": "slavic", "русская народная": "slavic",
    "новый год": "winter", "зимняя сказка": "winter",
}


@pytest.mark.parametrize("text,row", THEMES.items())
def test_theme_words_pick_the_row(text, row):
    assert match_row(text) == row
    prof = seeded_profile(text)
    assert prof.row == row and prof.source == "theme" and prof.hook_ids == kn.THEMES[row].hooks
    lo, hi = kn.THEMES[row].bpm
    assert lo <= prof.bpm <= hi and 0 <= prof.root <= 11 and prof.mode == kn.THEMES[row].mode


def test_unknown_theme_is_pool_not_a_fake_theme():
    prof = seeded_profile("бухгалтерский отчёт")
    assert prof.row is None and prof.source == "pool" and prof.hook_ids == kn.DEFAULT_HOOKS
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    assert lo <= prof.bpm <= hi and prof.mode in kn.GENRE_WINDOWS["club"].scales


def test_profile_is_deterministic_and_case_insensitive():
    assert seeded_profile("Космос") == seeded_profile("  космос ")
    assert len({seeded_profile(t).bpm for t in ("космос", "космос 2", "космос 3", "космос 4", "космос 5")}) > 1


def test_theme_windows_inside_club_window():
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    for name, row in kn.THEMES.items():
        assert lo <= row.bpm[0] <= row.bpm[1] <= hi, name
        assert row.mode in kn.SCALES and len(row.hooks) >= 3, name


SRC = pathlib.Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    if not (SRC / "rob_box_mcp_tools").is_dir():
        pytest.skip("нет src/rob_box_mcp_tools рядом с пакетом")
    if str(SRC / "rob_box_mcp_tools") not in sys.path:
        sys.path.append(str(SRC / "rob_box_mcp_tools"))
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("rtttl") / "voice_memory.db"))


def _melodies(library, ids):
    out = {}
    for melody_id in ids:
        rec = library.get(melody_id)
        assert rec is not None and rec["name"] == melody_id, f"{melody_id} нет в архиве"
        out[melody_id] = rec["rtttl"]
    return out


@pytest.mark.parametrize("row", [*kn.THEMES, None])
def test_theme_hooks_exist_in_the_local_library_and_sound(library, row):
    """Хук темы — из локальной библиотеки на любой тонике сета (было 0/14), и не одна и та же мелодия подряд."""
    hooks_ids = kn.THEMES[row].hooks if row else kn.DEFAULT_HOOKS
    melodies = _melodies(library, hooks_ids)
    text = next((t for t, r in THEMES.items() if r == row), "без темы")
    base = seeded_profile(text)
    for root in range(12):
        prof = base.__class__(**{**base.__dict__, "root": root})
        first = compose_p(prof, 1, set_seed=root, melodies=melodies)
        assert first.hook is not None and first.hook.source in hooks_ids, (row, root)
        second = compose_p(prof, 2, set_seed=root, melodies=melodies, recent_hooks=(first.hook.source,))
        assert second.hook is not None and second.hook.source != first.hook.source, (row, root)
