"""Issue #2841 — жанровые каркасы ударных в ``core.harmonize``.

Живой сет 23.09.2026: у всех RTTTL-тем один бит, потому что
``_build_drums`` прибит намертво. Контракт:

* ``auto`` (по умолчанию) — байт-в-байт прежнее поведение;
* 6 жанровых каркасов + ``none``; у каждого опора на первой доле;
* неизвестный стиль — ValueError со списком допустимых;
* стиль проходит сквозь ``melody_to_compose_params`` в Harmonization.
"""

import pytest

from rob_box_mcp_tools.core import harmonize as hz
from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody

#: Плотная тема (восьмые) и редкая (половинки) — чтобы проверить оба варианта.
DENSE_RTTTL = "dense:d=8,o=5,b=120:c,d,e,f,g,a,b,c6,b,a,g,f,e,d,c,d"
SPARSE_RTTTL = "sparse:d=2,o=5,b=90:c,g,e,c"


def _params(rtttl, **kw):
    return melody_to_compose_params(rtttl_to_melody(rtttl), **kw)


def test_style_catalogue_has_at_least_four_genre_skeletons():
    genre = [s for s in hz.DRUM_STYLES if s not in ("auto", "none")]
    assert len(genre) >= 4
    assert hz.DEFAULT_DRUM_STYLE == "auto"


@pytest.mark.parametrize("rtttl", [DENSE_RTTTL, SPARSE_RTTTL])
def test_auto_is_the_old_behaviour(rtttl):
    """Без drum_style — ровно то же, что раньше строил фиксированный каркас."""
    old = _params(rtttl)["harmony"]
    auto = _params(rtttl, drum_style="auto")["harmony"]
    assert (old.drums, old.hats) == (auto.drums, auto.hats)
    assert old.drums.startswith("X...o...")
    assert old.drums[12] == "o"


@pytest.mark.parametrize("style", [s for s in hz.DRUM_STYLES if s not in ("auto", "none")])
@pytest.mark.parametrize("dense", [False, True])
def test_every_genre_skeleton_is_a_bar_with_downbeat_kick(style, dense):
    drums = hz._build_drums([0.0] * hz.STEPS_PER_BAR, dense, style)
    assert len(drums) == hz.STEPS_PER_BAR
    assert drums[0] == "X"
    assert set(drums) <= {"X", "o", "."}


def test_skeletons_differ_from_each_other():
    patterns = {
        style: hz._build_drums([0.0] * hz.STEPS_PER_BAR, False, style)
        for style in hz.DRUM_STYLES if style not in ("auto", "none")
    }
    assert len(set(patterns.values())) == len(patterns)


def test_four_on_floor_puts_kick_on_every_beat_and_offbeat_hats():
    h = _params(DENSE_RTTTL, drum_style="four_on_floor")["harmony"]
    assert [h.drums[i] for i in (0, 4, 8, 12)] == ["X"] * 4
    assert h.hats == "..-...-...-...-."


def test_halftime_snare_only_on_beat_three():
    h = _params(SPARSE_RTTTL, drum_style="halftime")["harmony"]
    assert h.drums.index("o") == 8
    assert h.drums.count("o") == 1


def test_dense_theme_picks_the_fuller_variant():
    sparse = hz._build_drums([0.0] * 16, False, "breakbeat")
    dense = hz._build_drums([0.0] * 16, True, "breakbeat")
    assert dense != sparse
    assert dense.count("X") + dense.count("o") > sparse.count("X") + sparse.count("o")


def test_backbeat_keeps_density_driven_hats():
    auto = _params(DENSE_RTTTL)["harmony"]
    backbeat = _params(DENSE_RTTTL, drum_style="backbeat")["harmony"]
    assert backbeat.hats == auto.hats


def test_none_removes_drums_and_hats():
    h = _params(DENSE_RTTTL, drum_style="none")["harmony"]
    assert h.drums == "" and h.hats == ""


def test_unknown_style_is_value_error_listing_styles():
    with pytest.raises(ValueError) as exc:
        _params(DENSE_RTTTL, drum_style="polka")
    assert "four_on_floor" in str(exc.value)


@pytest.mark.parametrize("raw, expected", [(None, "auto"), ("", "auto"), (" March ", "march")])
def test_check_drum_style_normalizes(raw, expected):
    assert hz.check_drum_style(raw) == expected


def test_style_patterns_auto_means_backbeat_for_free_tracks():
    assert hz.style_patterns("auto", dense=False) == hz.style_patterns("backbeat", dense=False)
    drums, hats = hz.style_patterns("four_on_floor", dense=False)
    assert drums == "X...X...X...X..." and hats == "..-...-...-...-."
    assert hz.style_patterns("none", dense=False) == ("", "")
