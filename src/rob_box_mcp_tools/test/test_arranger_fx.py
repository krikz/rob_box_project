"""Issue #2968 — одиночный FX-акцент ``fx`` в аранжировке ``compose_music``.

Контракт (зеркало test_arranger_groove_loop.py, #2841):

* FX встаёт в первый СВОБОДНЫЙ слот d1-d3 — общий пул с ``groove_loop``,
  порядок распределения детерминирован (луп первым, FX вторым);
* в отличие от лупа, FX звучит ТОЛЬКО в секциях-стыках формы
  (:data:`FX_BOUNDARY_SECTIONS`) — редкий акцент, не постоянный слой;
* ``beat_stretch`` не ставится — растяжка меняет высоту, а «выстрел» с
  уехавшей высотой звучит как брак, не как эффект;
* имя остаётся коротким: путь и флаг pack 1 — забота санитайзера
  (тот же ``loop(...)`` синт, что и жанровый луп — нет отдельного FX-синта).
"""

import re

import pytest

from rob_box_mcp_tools.core.arranger import (
    ArrangementError,
    FX_ROLE,
    render,
    spec_from_flat,
)
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

_FX_LINE_RE = re.compile(r"^(d\d) >> loop\('([^']+)', dur=([\d.]+), amp=(.+)\)$", re.M)


def _free_spec(**overrides):
    params = dict(
        bpm=120, root="A", scale="minor", form="arc",
        drums="X...o...X...o...", hats="-.-.-.-.-.-.-.-.",
        bass_synth="dub", bass_notes="0,0,4,0",
        lead_synth="pluck", lead_notes="0,2,4,7",
        pad_synth="warmpad", pad_notes="0,2,4",
    )
    params.update(overrides)
    return spec_from_flat(**params)


def test_no_fx_means_no_fx_layer():
    spec = _free_spec()
    assert all(layer.role != FX_ROLE for layer in spec.layers)
    assert "loop(" not in render(spec)


@pytest.mark.parametrize("value", [None, "", "none", "  "])
def test_empty_fx_is_ignored(value):
    spec = _free_spec(fx=value)
    assert all(layer.role != FX_ROLE for layer in spec.layers)


def test_unknown_fx_is_arrangement_error_listing_names():
    with pytest.raises(ArrangementError) as exc:
        _free_spec(fx="explosion_9")
    assert "gunshot_1" in str(exc.value)


def test_fx_takes_first_free_drum_slot():
    # drums → d1, hats → d2, перкуссии нет → fx в d3.
    code = render(_free_spec(fx="gunshot_1"))
    match = _FX_LINE_RE.search(code)
    assert match, code
    assert match.group(1) == "d3"
    assert match.group(2) == "gunshot_1"


def test_fx_and_groove_loop_do_not_collide_on_slots():
    """Оба претендуют на d1-d3 — не должны получить один и тот же слот.

    drums=None оставляет только hats на d1, поэтому и лупу, и FX хватает
    места (d2 и d3) — с обоими drums+hats, как в ``_free_spec`` по
    умолчанию, свободен только один слот d3, и на двоих его не хватит
    (см. отдельный ``test_no_free_drum_slot_is_honest_error``-подобный
    случай, честная ошибка, а не тихая коллизия слотов)."""
    code = render(_free_spec(drums=None, groove_loop="jungle_1", fx="gunshot_1"))
    loop_lines = re.findall(r"^(d\d) >> loop\('jungle_1'", code, re.M)
    fx_lines = re.findall(r"^(d\d) >> loop\('gunshot_1'", code, re.M)
    assert loop_lines and fx_lines
    assert loop_lines[0] != fx_lines[0]


def test_no_free_drum_slot_is_honest_error():
    """drums→d1, hats→d2, groove_loop→d3 (лупа кладут первым — см. модульный
    docstring) — для FX не остаётся ни одного слота d1-d3.

    Issue #2978 убрала ``perc`` из плана состава ``arc`` (бюджет ролей на
    секцию — не больше :data:`MAX_SIMULTANEOUS_ROLES`), поэтому прежний
    способ забить три слота (``drums``+``hats``+``perc``) больше не
    работает: ``perc`` в этой форме теперь всегда молчит и слота не
    занимает. Тот же сценарий («слотов не хватило») даёт связка с лупом.
    """
    with pytest.raises(ArrangementError) as exc:
        render(_free_spec(groove_loop="dnb_1", fx="siren_1"))
    assert "d1-d3" in str(exc.value)


def test_fx_has_no_beat_stretch():
    """Растяжка меняет высоту — недопустимо для одиночного акцента."""
    code = render(_free_spec(fx="siren_1"))
    assert "beat_stretch" not in code


def test_fx_is_silent_outside_boundary_sections():
    """arc: intro/build/main — не стыки, амп там 0; break/peak — стык."""
    code = render(_free_spec(fx="siren_1"))
    amp = _FX_LINE_RE.search(code).group(4)
    assert amp.startswith("var(["), amp
    assert "0, " in amp or amp.startswith("var([0"), amp


def test_fx_form_without_boundary_section_is_honest_error():
    """Форма без ни одной секции из FX_BOUNDARY_SECTIONS — честная ошибка,
    а не молча немой слой (тот же принцип, что #2837 «0 плееров»)."""
    from rob_box_mcp_tools.core import arranger

    original = dict(arranger.FORMS)
    arranger.FORMS["_no_boundary_test"] = [("only", 8, {"pad": 0.5})]
    try:
        with pytest.raises(ArrangementError) as exc:
            render(_free_spec(form="_no_boundary_test", fx="siren_1"))
        assert "стыка" in str(exc.value)
    finally:
        arranger.FORMS.clear()
        arranger.FORMS.update(original)


def test_derived_harmony_path_also_gets_fx():
    from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody

    params = melody_to_compose_params(
        rtttl_to_melody("test:d=4,o=5,b=100:c,e,g,c6,g,e,c,p")
    )
    spec = spec_from_flat(
        harmony=params["harmony"], bpm=params["bpm"], root=params["root"],
        scale=params["scale"], lead_synth="pluck", bass_synth="dub",
        pad_synth="warmpad", counter_synth="none", fx="scratch_1",
    )
    assert any(layer.role == FX_ROLE for layer in spec.layers)
    assert _FX_LINE_RE.search(render(spec))


def test_rendered_fx_passes_sanitizer_only_with_flag():
    code = render(_free_spec(fx="laser_1"))
    blocked = sanitize_renando(code, 0.7)
    assert any("не прослушан" in e for e in blocked.quality_errors)
    allowed = sanitize_renando(code, 0.7, pack1_loops_enabled=True)
    assert allowed.quality_errors == ()
    assert "1_pitchglitch_samples/_/backslash/003_SoundFX_LaserGun_KS.wav" in allowed.code


def test_levels_knob_covers_fx_role():
    from rob_box_mcp_tools.core.arranger import ArrangeOptions

    quiet = render(_free_spec(fx="gunshot_1", options=ArrangeOptions(levels={"fx": 0.5})))
    loud = render(_free_spec(fx="gunshot_1"))
    quiet_amp = _FX_LINE_RE.search(quiet).group(4)
    loud_amp = _FX_LINE_RE.search(loud).group(4)
    assert quiet_amp != loud_amp
