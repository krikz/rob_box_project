"""Issue #2841 — жанровый луп ``groove_loop`` в аранжировке ``compose_music``.

Контракт:

* луп встаёт в первый СВОБОДНЫЙ слот d1-d3 (на роботе есть только
  d1-d3/p1-p3), а если свободного нет — честная ArrangementError;
* длина лупа в битах — степень двойки под bpm, ``beat_stretch=1``
  растягивает файл ровно на неё;
* громкость идёт за бочкой формы, в ambient — за подкладом;
* имя остаётся коротким: путь и флаг pack 1 — забота санитайзера, и
  сгенерированный код проходит его только при включённом флаге.
"""

import re

import pytest

from rob_box_mcp_tools.core.arranger import (
    ArrangementError,
    LOOP_ROLE,
    render,
    spec_from_flat,
)
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

_LOOP_LINE_RE = re.compile(r"^(d\d) >> loop\('([^']+)', dur=([\d.]+), beat_stretch=1, amp=(.+)\)$", re.M)


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


def test_no_groove_loop_means_no_loop_layer():
    spec = _free_spec()
    assert all(layer.role != LOOP_ROLE for layer in spec.layers)
    assert "loop(" not in render(spec)


@pytest.mark.parametrize("value", [None, "", "none", "  "])
def test_empty_groove_loop_is_ignored(value):
    spec = _free_spec(groove_loop=value)
    assert all(layer.role != LOOP_ROLE for layer in spec.layers)


def test_unknown_groove_loop_is_arrangement_error_listing_names():
    with pytest.raises(ArrangementError) as exc:
        _free_spec(groove_loop="trance_9")
    assert "dnb_1" in str(exc.value)


def test_loop_takes_first_free_drum_slot():
    # drums → d1, hats → d2, перкуссии нет → луп в d3.
    code = render(_free_spec(groove_loop="jungle_1"))
    match = _LOOP_LINE_RE.search(code)
    assert match, code
    assert match.group(1) == "d3"
    assert match.group(2) == "jungle_1"


def test_loop_takes_d1_when_there_are_no_drums():
    code = render(_free_spec(drums=None, hats=None, groove_loop="break_1"))
    match = _LOOP_LINE_RE.search(code)
    assert match and match.group(1) == "d1", code


def test_no_free_drum_slot_is_honest_error():
    with pytest.raises(ArrangementError) as exc:
        render(_free_spec(perc="..n...n...n...n.", groove_loop="dnb_1"))
    assert "d1-d3" in str(exc.value)


def test_loop_length_follows_bpm():
    # jungle_1 = 2.0 c: 120 BPM → 4 бита; 60 BPM → 2 бита.
    fast = _LOOP_LINE_RE.search(render(_free_spec(bpm=120, groove_loop="jungle_1")))
    slow = _LOOP_LINE_RE.search(render(_free_spec(bpm=60, groove_loop="jungle_1")))
    assert fast.group(3) == "4"
    assert slow.group(3) == "2"


def test_loop_is_silent_where_form_drops_drums():
    """arc: intro (8 тактов) без бочки → первый сегмент amp лупа = 0."""
    amp = _LOOP_LINE_RE.search(render(_free_spec(groove_loop="funk_1"))).group(4)
    assert amp.startswith("var([0, "), amp


def test_loop_follows_pad_in_ambient_form():
    code = render(_free_spec(form="ambient", drums=None, hats=None, groove_loop="ambient_1"))
    amp = _LOOP_LINE_RE.search(code).group(4)
    head = amp[len("var(["):].split("]")[0] if amp.startswith("var(") else amp
    values = [float(v) for v in head.split(",")]
    assert values and all(v > 0 for v in values), amp  # звучит во всех секциях


def test_derived_harmony_path_also_gets_loop():
    from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody

    params = melody_to_compose_params(
        rtttl_to_melody("test:d=4,o=5,b=100:c,e,g,c6,g,e,c,p")
    )
    spec = spec_from_flat(
        harmony=params["harmony"], bpm=params["bpm"], root=params["root"],
        scale=params["scale"], lead_synth="pluck", bass_synth="dub",
        pad_synth="warmpad", counter_synth="none", groove_loop="hiphop_1",
    )
    assert any(layer.role == LOOP_ROLE for layer in spec.layers)
    assert _LOOP_LINE_RE.search(render(spec))


def test_rendered_loop_passes_sanitizer_only_with_flag():
    code = render(_free_spec(groove_loop="dnb_2"))
    blocked = sanitize_renando(code, 0.7)
    assert any("не прослушан" in e for e in blocked.quality_errors)
    allowed = sanitize_renando(code, 0.7, pack1_loops_enabled=True)
    assert allowed.quality_errors == ()
    assert "../../1_pitchglitch_samples/_loop_/dnb_2" in allowed.code


def test_pack0_loop_is_gated_like_pack1():
    code = render(_free_spec(groove_loop="foxdot"))
    assert sanitize_renando(code, 0.7).quality_errors
    result = sanitize_renando(code, 0.7, pack1_loops_enabled=True)
    assert result.quality_errors == ()
    assert result.slot_error is None
