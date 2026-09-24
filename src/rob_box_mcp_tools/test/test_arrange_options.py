"""Ручки ядра аранжировщика (ADR-0132, PR-3): каждая меняет только своё.

``HarmonizeOptions`` (ноты партий) и ``ArrangeOptions`` (сборка слоёв) —
по умолчанию все ``auto``, это сегодняшнее поведение байт-в-байт (его
стережёт ``test_arranger_golden``; здесь — что явные ``auto`` дают то же,
что отсутствие опций). Каждая не-``auto`` ручка проверяется на двух
вещах: она делает то, что обещает, и не трогает чужие партии. Плюс —
неверное значение даёт ``ValueError`` со списком, а итог ручки виден в
партитуре (строка «Решения по умолчанию»).

Темы берутся из замороженного входа golden-фикстуры (RTTTL-строка по
ключу архива), чтобы тест не зависел от ранжирования поиска библиотеки.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Dict, Optional

import pytest

from rob_box_mcp_tools.core.arranger import (
    PAD_STAB_SUS,
    ArrangementError,
    ArrangeOptions,
    render,
    spec_from_flat,
)
from rob_box_mcp_tools.core.harmonize import (
    BASS_MIDI_FLOOR,
    HarmonizeOptions,
    harmonize,
    parse_chord,
)
from rob_box_mcp_tools.core.rtttl_compose import (
    _pitch_weights,
    _profile_score,
    melody_to_compose_params,
    rtttl_to_melody,
)
from rob_box_mcp_tools.core.score_sheet import describe

_FIXTURE = Path(__file__).parent / "fixtures" / "arranger_golden.json"

#: Плотная тема (марш), редкая тема (Пантера), тема, где чистый профиль
#: спорит с тональным центром (#2873), тема с выбросами (#2876), тема,
#: которую автоматика переносит на октаву.
DENSE, SPARSE, KEY_DISPUTED, OUTLIERS, SHIFTED = (
    "imperial", "pinkpant", "hallofth_2", "stilldre_2", "tetris",
)

#: Тема из живого прогона issue #2962 (Терминатор, ``lead_octave='+1'``
#: с ``lead_synth='viola'``/``counter_synth='flute'`` уехал в MIDI 104-111).
TERMINATOR = "terminat"

_SYNTHS = dict(lead_synth="pluck", bass_synth="bass", pad_synth="strings")


def _rtttl(key: str) -> str:
    cases = json.loads(_FIXTURE.read_text(encoding="utf-8"))["cases"]
    return next(c["rtttl"] for c in cases if c["key"] == key)


def _params(key: str, options: Optional[HarmonizeOptions] = None) -> Dict[str, object]:
    return melody_to_compose_params(rtttl_to_melody(_rtttl(key)), options=options)


def _spec(params, options: Optional[ArrangeOptions] = None, **flat):
    kwargs = dict(_SYNTHS, form="arc", repeat=True)
    kwargs.update(flat)
    return spec_from_flat(
        harmony=params["harmony"],
        bpm=float(params["bpm"]),
        root=str(params["root"]),
        scale=str(params["scale"]),
        options=options,
        **kwargs,
    )


def _layer(spec, role):
    return next((layer for layer in spec.layers if layer.role == role), None)


def _player_line(code: str, player: str) -> str:
    return next(line for line in code.splitlines() if line.startswith(f"{player} >>"))


def _amps(line: str):
    match = re.search(r"amp=(var\(\[([^\]]*)\]|([0-9.]+))", line)
    raw = match.group(2) if match.group(2) is not None else match.group(3)
    return [float(v) for v in raw.split(",")]


def _decisions_text(params, spec) -> str:
    sheet = describe(
        spec=spec, code=render(spec), harmony=params["harmony"],
        prep_decisions=params["decisions"],
    )
    return sheet["text"]


# ---------------------------------------------------------------------------
# Умолчания и проверка ввода
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("key", [DENSE, SPARSE, OUTLIERS, SHIFTED])
def test_explicit_auto_options_equal_no_options(key):
    """Все ручки ``auto`` явно = без опций: те же ноты и тот же код."""
    base = _params(key)
    explicit = _params(key, HarmonizeOptions())
    assert explicit["harmony"] == base["harmony"]
    assert explicit["lead_midi"] == base["lead_midi"]
    assert render(_spec(explicit, ArrangeOptions())) == render(_spec(base))


@pytest.mark.parametrize("kwargs", [
    {"key_detection": "histogram"},
    {"lead_outliers": "drop"},
    {"harmonic_rhythm": "quarter"},
    {"density": "medium"},
    {"bass_style": "walking"},
    {"bass_approach": "yes"},
    {"pad_style": "arp"},
    {"pad_register": "top"},
    {"pad_register": (60, 66)},
    {"lead_octave": 3},
    {"lead_octave": True},
    {"chords": ("Am", "Hx")},
    {"chords": ()},
    {"drums": 5},
])
def test_harmonize_options_reject_unknown_values(kwargs):
    """Неизвестное значение — ошибка со списком, а не тихая замена."""
    with pytest.raises(ValueError):
        HarmonizeOptions(**kwargs)


@pytest.mark.parametrize("kwargs", [
    {"counter": "maybe"},
    {"theme_octaves": "double"},
    {"levels": {"vocals": 0.5}},
    {"levels": {"bass": 3.0}},
    {"levels": {"bass": "loud"}},
])
def test_arrange_options_reject_unknown_values(kwargs):
    with pytest.raises(ArrangementError):
        ArrangeOptions(**kwargs)


def test_parse_chord_names():
    assert parse_chord("Am") == (9, 0, 4)
    assert parse_chord("F") == (5, 9, 0)
    assert parse_chord("Bbm") == (10, 1, 5)
    assert parse_chord("C#") == (1, 5, 8)


# ---------------------------------------------------------------------------
# Подготовка темы: key_detection, lead_octave, lead_outliers
# ---------------------------------------------------------------------------


def test_key_detection_profile_is_pure_krumhansl():
    """``profile`` — лучший по чистой гистограмме; меняется только гармония."""
    auto = _params(KEY_DISPUTED)
    prof = _params(KEY_DISPUTED, HarmonizeOptions(key_detection="profile"))
    notes = [(int(m) % 12, float(d)) for m, d in auto["harmony"].lead if m is not None]
    weights = _pitch_weights(notes)
    best = max(
        ((r, s) for r in range(12) for s in ("major", "minor")),
        key=lambda rs: _profile_score(weights, rs[0], rs[1]),
    )
    ranked_top = prof["decisions"]["key_ranked"][0]
    assert ranked_top[2] == pytest.approx(_profile_score(weights, *best), abs=1e-3)
    assert (prof["root"], prof["scale"]) != (auto["root"], auto["scale"])
    assert prof["lead_midi"] == auto["lead_midi"]
    assert prof["decisions"]["key_detection"] == "profile"
    assert prof["harmony"].root == prof["root"]


def test_lead_octave_minus_one_shifts_from_normalized_register():
    """issue #2962: ручной сдвиг — от НОРМАЛИЗОВАННОГО регистра, не от сырого.

    ``SHIFTED`` (tetris) auto переносит на октаву вниз (``lead_shift ==
    -12``, см. следующий тест). ``lead_octave=-1`` должен лечь ещё на
    октаву ниже ЭТОГО же нормализованного регистра, а не octave-1 от
    сырой (незаписанной) темы.
    """
    auto = _params(SHIFTED)
    down = _params(SHIFTED, HarmonizeOptions(lead_octave=-1))
    for (a, da), (b, db) in zip(auto["harmony"].lead, down["harmony"].lead):
        assert da == db
        assert (a is None and b is None) or b == a - 12
    assert (
        down["decisions"]["lead_shift"]
        == auto["decisions"]["lead_shift"] - 12
    )
    assert down["decisions"]["lead_octave_mode"] == -1


def test_lead_octave_plus_one_errors_when_normalized_register_is_at_ceiling():
    """issue #2962: '+1' поверх нормализованного регистра выше потолка —
    честная ошибка.

    Раньше сдвиг считался от СЫРОЙ темы и потолок (``_LEAD_MAX_CEILING``)
    для ручного режима не проверялся вовсе — живой прогон ``terminat`` +
    ``lead_octave='+1'`` уехал в MIDI 104-111 (свист). ``SHIFTED`` (tetris)
    нормализован в 69-81; ``+1`` дал бы максимум 93 — выше потолка 88,
    поэтому тул обязан честно отказать, а не тихо сыграть выше рабочего
    регистра.
    """
    with pytest.raises(ValueError, match="потолка"):
        _params(SHIFTED, HarmonizeOptions(lead_octave=1))


def test_acceptance_issue_2962_terminator_lead_octave_plus_one():
    """Acceptance issue #2962: точный сценарий живого прогона 24.09.

    ``compose_music(name='terminat', lead_synth='viola', lead_octave='+1',
    counter_synth='flute')`` дал лид в MIDI 83-111 и контрмелодию флейты
    до 107 — ``_LEAD_MAX_CEILING=88`` соблюдался только в ``auto``. Тема
    ``terminat`` в рабочем регистре уже стоит у потолка (auto: 59-87),
    поэтому ``+1`` обязан честно отказать, а не тихо уйти в свист; ни у
    лида, ни у контрмелодии (которая всегда кладётся НИЖЕ ноты темы,
    см. ``harmonize._build_counter``) не должно быть ни одной ноты выше
    потолка ни в одном режиме.
    """
    from rob_box_mcp_tools.core.rtttl_compose import _LEAD_MAX_CEILING

    auto = _params(TERMINATOR)
    lead_notes = [m for m, _ in auto["harmony"].lead if m is not None]
    counter_notes = [m for m, _ in auto["harmony"].counter if m is not None]
    assert max(lead_notes) <= _LEAD_MAX_CEILING
    assert max(counter_notes) <= _LEAD_MAX_CEILING

    with pytest.raises(ValueError, match="потолка"):
        _params(TERMINATOR, HarmonizeOptions(lead_octave=1))


def test_lead_octave_keep_disables_register_normalization():
    """``keep`` — тема как записана (auto переносит эту тему на октаву вниз)."""
    auto = _params(SHIFTED)
    keep = _params(SHIFTED, HarmonizeOptions(lead_octave="keep"))
    assert auto["decisions"]["lead_shift"] == -12
    assert keep["decisions"]["lead_shift"] == 0
    for (a, _da), (k, _dk) in zip(auto["harmony"].lead, keep["harmony"].lead):
        assert (a is None and k is None) or k == a + 12


def test_lead_outliers_keep_leaves_outliers_in_place():
    fix = _params(OUTLIERS)
    keep = _params(OUTLIERS, HarmonizeOptions(lead_outliers="keep"))
    assert fix["decisions"]["outliers_moved"] > 0
    assert keep["decisions"]["outliers_moved"] == 0
    diffs = [
        (k - f) for (f, _a), (k, _b) in zip(fix["harmony"].lead, keep["harmony"].lead)
        if f is not None and f != k
    ]
    assert len(diffs) == fix["decisions"]["outliers_moved"]
    assert all(d % 12 == 0 for d in diffs)
    assert keep["decisions"]["lead_outliers_mode"] == "keep"


# ---------------------------------------------------------------------------
# Гармония: chords, harmonic_rhythm, density
# ---------------------------------------------------------------------------


def test_chords_override_drives_pad_and_bass():
    chords = ("Am", "F", "C", "G")
    auto = _params(DENSE)
    opts = HarmonizeOptions(chords=chords, bass_approach="off")
    got = _params(DENSE, opts)
    harmony = got["harmony"]
    want = [parse_chord(chords[bar % 4]) for bar in range(harmony.bars)]
    for chord in harmony.chords:
        assert chord.start % 4 == 0
        assert chord.pitch_classes == want[int(chord.start // 4)]
        assert sorted(t % 12 for t in chord.tones) == sorted(chord.pitch_classes)
    cursor = 0.0
    for note, dur in harmony.bass:
        bar_chord = want[int(cursor // 4)]
        assert note % 12 in bar_chord
        cursor += dur
    assert harmony.lead == auto["harmony"].lead
    assert harmony.drums == auto["harmony"].drums


def test_chords_longer_than_theme_is_an_error():
    too_many = tuple(["Am"] * 50)
    with pytest.raises(ValueError, match="chords"):
        _params(SPARSE, HarmonizeOptions(chords=too_many))


def test_harmonic_rhythm_bar_changes_only_on_barlines():
    auto = _params(DENSE)
    bar = _params(DENSE, HarmonizeOptions(harmonic_rhythm="bar"))
    assert any(c.start % 4 for c in auto["harmony"].chords)
    assert all(c.start % 4 == 0 for c in bar["harmony"].chords)
    assert bar["harmony"].lead == auto["harmony"].lead


def test_harmonic_rhythm_half_drops_inertia():
    auto = _params(DENSE)
    half = _params(DENSE, HarmonizeOptions(harmonic_rhythm="half"))
    assert len(half["harmony"].chords) > len(auto["harmony"].chords)
    assert all(c.start % 2 == 0 for c in half["harmony"].chords)


def test_density_sparse_on_dense_theme_gives_sparse_arrangement():
    auto = _params(DENSE)
    sparse = _params(DENSE, HarmonizeOptions(density="sparse"))
    assert auto["harmony"].dense and not sparse["harmony"].dense
    assert sparse["harmony"].density == auto["harmony"].density
    assert sparse["harmony"].chords == auto["harmony"].chords
    assert sparse["harmony"].decisions["bass_step"] == 2.0
    assert all(dur >= 0.5 for _n, dur in sparse["harmony"].pad)
    assert {dur for _n, dur in sparse["harmony"].pad} <= {2.0}
    spec_auto, spec_sparse = _spec(auto), _spec(sparse)
    assert _layer(spec_auto, "counter") is not None
    assert _layer(spec_sparse, "counter") is None
    assert any(isinstance(n, tuple) for n in _layer(spec_auto, "lead").midi)
    assert not any(isinstance(n, tuple) for n in _layer(spec_sparse, "lead").midi)


def test_density_dense_on_sparse_theme():
    dense = _params(SPARSE, HarmonizeOptions(density="dense"))
    assert dense["harmony"].dense
    assert _layer(_spec(dense), "counter") is not None


# ---------------------------------------------------------------------------
# Бас: bass_style, bass_approach
# ---------------------------------------------------------------------------


def _chord_at(harmony, beat):
    return next(
        c for c in harmony.chords if c.start <= beat < c.start + c.beats
    )


def _walk(part):
    cursor = 0.0
    for note, dur in part:
        yield cursor, note, dur
        cursor += dur


def test_bass_style_root_plays_chord_roots():
    auto = _params(DENSE)
    got = _params(DENSE, HarmonizeOptions(bass_style="root", bass_approach="off"))
    harmony = got["harmony"]
    for onset, note, _dur in _walk(harmony.bass):
        assert note == _chord_at(harmony, onset).root_midi
    assert harmony.pad == auto["harmony"].pad
    assert harmony.chords == auto["harmony"].chords
    assert sum(d for _n, d in harmony.bass) == sum(d for _n, d in auto["harmony"].bass)


def test_bass_style_root_fifth_uses_root_and_fifth_only():
    got = _params(DENSE, HarmonizeOptions(bass_style="root_fifth", bass_approach="off"))
    harmony = got["harmony"]
    for onset, note, _dur in _walk(harmony.bass):
        root = _chord_at(harmony, onset).root_midi
        assert note in (root, root + 7)


def test_bass_style_pedal_holds_tonic():
    got = _params(DENSE, HarmonizeOptions(bass_style="pedal"))
    harmony = got["harmony"]
    tonic = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"].index(harmony.root)
    assert {n for n, _d in harmony.bass} == {BASS_MIDI_FLOOR + (tonic - BASS_MIDI_FLOOR) % 12}
    assert [d for _n, d in harmony.bass] == [c.beats for c in harmony.chords]


def test_bass_style_off_removes_bass_layer():
    auto = _params(DENSE)
    got = _params(DENSE, HarmonizeOptions(bass_style="off"))
    assert got["harmony"].bass == ()
    spec = _spec(got)
    assert _layer(spec, "bass") is None
    code, code_auto = render(spec), render(_spec(auto))
    assert "p1 >>" not in code and "p1 >>" in code_auto
    assert _player_line(code, "p2") == _player_line(code_auto, "p2")
    assert _player_line(code, "p3") == _player_line(code_auto, "p3")


def test_bass_approach_off_and_on():
    auto = _params(DENSE)["harmony"]
    off = _params(DENSE, HarmonizeOptions(bass_approach="off"))["harmony"]
    on = _params(DENSE, HarmonizeOptions(bass_approach="on"))["harmony"]
    assert auto.decisions["bass_approaches"] > 0
    assert off.decisions["bass_approaches"] == 0
    assert on.decisions["bass_approaches"] >= auto.decisions["bass_approaches"]
    total = sum(d for _n, d in auto.bass)
    assert sum(d for _n, d in off.bass) == total == sum(d for _n, d in on.bass)
    # Без подходов каждая нота баса — тон своего аккорда.
    for onset, note, _dur in _walk(off.bass):
        assert note % 12 in _chord_at(off, onset).pitch_classes
    assert off.pad == auto.pad


def test_bass_approach_on_reaches_single_note_windows():
    """``on`` ставит подход и там, где ``auto`` его не ставит (окно из одной ноты)."""
    auto = _params(SPARSE)["harmony"]
    on = _params(SPARSE, HarmonizeOptions(bass_approach="on"))["harmony"]
    assert on.decisions["bass_approaches"] > auto.decisions["bass_approaches"]


# ---------------------------------------------------------------------------
# Пэд: pad_style, pad_register
# ---------------------------------------------------------------------------


def test_pad_style_off_removes_pad_layer():
    auto = _params(DENSE)
    got = _params(DENSE, HarmonizeOptions(pad_style="off"))
    assert got["harmony"].pad == ()
    assert got["harmony"].bass == auto["harmony"].bass
    spec = _spec(got)
    assert _layer(spec, "pad") is None
    assert "p3 >>" not in render(spec)


def test_pad_style_sustain_holds_each_chord():
    got = _params(DENSE, HarmonizeOptions(pad_style="sustain"))
    harmony = got["harmony"]
    assert [(t, d) for t, d in harmony.pad] == [(c.tones, c.beats) for c in harmony.chords]
    assert harmony.pad_sus is None
    pad = _layer(_spec(got), "pad")
    assert pad.sus is None
    assert "sus=" not in _player_line(render(_spec(got)), "p3")


def test_pad_style_stab_equals_auto():
    auto = _params(DENSE)
    stab = _params(DENSE, HarmonizeOptions(pad_style="stab"))
    assert stab["harmony"].pad == auto["harmony"].pad
    assert _layer(_spec(stab), "pad").sus == PAD_STAB_SUS


@pytest.mark.parametrize("register,bounds", [("mid", (55, 67)), ((60, 72), (60, 72))])
def test_pad_register_places_pad_tones(register, bounds):
    auto = _params(DENSE)
    got = _params(DENSE, HarmonizeOptions(pad_register=register))
    tones = [t for chord in got["harmony"].chords for t in chord.tones]
    assert all(bounds[0] <= t <= bounds[1] for t in tones)
    assert got["harmony"].bass == auto["harmony"].bass
    for mine, theirs in zip(got["harmony"].chords, auto["harmony"].chords):
        assert mine.pitch_classes == theirs.pitch_classes


# ---------------------------------------------------------------------------
# Ударные
# ---------------------------------------------------------------------------


def test_drums_and_hats_override_patterns():
    auto = _params(DENSE)
    got = _params(DENSE, HarmonizeOptions(drums="X.X.o...X.X.o...", hats="-.-.-.-.-.-.-.-."))
    assert got["harmony"].drums == "X.X.o...X.X.o..."
    assert got["harmony"].hats == "-.-.-.-.-.-.-.-."
    assert got["harmony"].bass == auto["harmony"].bass
    assert "play('X.X.o...X.X.o...'" in render(_spec(got))


# ---------------------------------------------------------------------------
# Сборка: counter, theme_octaves, levels
# ---------------------------------------------------------------------------


def test_counter_off_on_dense_theme():
    params = _params(DENSE)
    code_auto = render(_spec(params))
    code_off = render(_spec(params, ArrangeOptions(counter="off")))
    assert "d3 >>" in code_auto and "d3 >>" not in code_off
    assert _player_line(code_off, "p1") == _player_line(code_auto, "p1")


def test_counter_on_on_sparse_theme():
    params = _params(SPARSE)
    assert _layer(_spec(params), "counter") is None
    assert _layer(_spec(params, ArrangeOptions(counter="on")), "counter") is not None


def test_theme_octaves_off_and_on():
    dense = _params(DENSE)
    lead_off = _layer(_spec(dense, ArrangeOptions(theme_octaves="off")), "lead")
    assert not any(isinstance(n, tuple) for n in lead_off.midi)
    sparse = _params(SPARSE)
    lead_on = _layer(_spec(sparse, ArrangeOptions(theme_octaves="on")), "lead")
    plain = _layer(_spec(sparse), "lead")
    for doubled, note in zip(lead_on.midi, plain.midi):
        assert doubled == (None if note is None else (note - 12, note))


def test_levels_bass_half_halves_bass_amp():
    params = _params(DENSE)
    code_auto = render(_spec(params))
    code_half = render(_spec(params, ArrangeOptions(levels={"bass": 0.5})))
    amps_auto = _amps(_player_line(code_auto, "p1"))
    amps_half = _amps(_player_line(code_half, "p1"))
    assert amps_half == [pytest.approx(a / 2, abs=1e-4) for a in amps_auto]
    for player in ("p2", "p3", "d1", "d2", "d3"):
        assert _player_line(code_half, player) == _player_line(code_auto, player)


def test_levels_apply_to_composed_tracks_too():
    flat = dict(
        bpm=100.0, root="A", scale="minor", form="arc",
        drums="X...o...", bass_synth="bass", bass_notes="0, 4",
        lead_synth="pluck", lead_notes="0, 2, 4, 7",
    )
    base = render(spec_from_flat(**flat))
    quiet = render(spec_from_flat(options=ArrangeOptions(levels={"lead": 0.5}), **flat))
    assert _amps(_player_line(quiet, "p2")) == [
        pytest.approx(a / 2, abs=1e-4) for a in _amps(_player_line(base, "p2"))
    ]
    assert _player_line(quiet, "p1") == _player_line(base, "p1")


# ---------------------------------------------------------------------------
# Бюджет громкости (issue #2963): levels — только аттенюатор
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", [1.0, 0.0, 0.5])
def test_levels_boundary_values_up_to_one_are_accepted(value):
    ArrangeOptions(levels={"lead": value})  # не поднимает ArrangementError


@pytest.mark.parametrize("value", [1.0001, 1.3, 1.2, 2.0])
def test_levels_above_one_are_rejected_not_silently_clamped(value):
    """Issue #2963: потолок 1 — честная ошибка (ADR-0132), а не тихий кламп.

    Живой инцидент 24.09: ``levels='lead=1.3,bass=1.2,...'`` — раньше
    множитель до 2.0 разгонял роль ГРОМЧЕ базового баланса. С 24.09.2026
    множитель может только притушить роль (0..1); модель получает ошибку
    со списком и чинится следующим вызовом, а не молчаливой заменой.
    """
    with pytest.raises(ArrangementError, match="0..1"):
        ArrangeOptions(levels={"lead": value})


def test_levels_can_only_lower_the_mix_balance_never_raise_it():
    """Issue #2963, acceptance: суммарный amp конфига ≤ бюджета (баланс
    без явных ``levels`` — сегодняшний баланс :data:`ROLE_PROFILE`, ровно
    тот же бюджет, что аранжировщик и так собрал бы по умолчанию).

    Живой конфиг из issue (второй пример, 10:15:40): ``lead_synth=
    imperialbrass``, ``theme_octaves=on``, ``levels='lead=1.3,bass=1.2,
    drums=0.7,hats=0.5,pad=0.8'`` — после клампа ``lead``/``bass`` летят в
    ошибку (см. тест выше); здесь проверяем сам БЮДЖЕТ на клампнутых
    значениях (1.0 вместо 1.3/1.2) — то, что реально долетит до эфира при
    honest-retry модели.
    """
    from rob_box_mcp_tools.core.score_sheet import describe as _describe

    params = _params(DENSE)
    baseline = _spec(params, ArrangeOptions(theme_octaves="on"))
    levels = {
        "lead": 1.0, "bass": 1.0, "drums": 0.7, "hats": 0.5, "pad": 0.8,
    }
    overridden = _spec(
        params, ArrangeOptions(theme_octaves="on", levels=levels),
    )
    base_sheet = _describe(spec=baseline, code=render(baseline))
    over_sheet = _describe(spec=overridden, code=render(overridden))
    budget = base_sheet["mix_balance"]["total"]
    got = over_sheet["mix_balance"]["total"]
    assert got <= budget


# ---------------------------------------------------------------------------
# Партитура видит заданные ручки
# ---------------------------------------------------------------------------


def test_score_sheet_default_line_says_auto():
    params = _params(DENSE)
    text = _decisions_text(params, _spec(params))
    assert "bass_style=auto→" in text
    assert "pad_style=auto→" in text
    assert "counter=auto→" in text
    assert "levels=" not in text


def test_score_sheet_reflects_explicit_knobs():
    # issue #2962: KEY_DISPUTED нормализован в 71-83 (auto lead_shift=-12),
    # "+1" упёрся бы в потолок 88 (83+12=95) и дал бы ValueError — здесь
    # проверяется только текст партитуры, поэтому берём "-1" (59-71, ОК).
    opts = HarmonizeOptions(
        key_detection="profile", lead_octave=-1, bass_style="root",
        bass_approach="off", pad_style="off", harmonic_rhythm="bar",
        pad_register="mid", density="sparse",
    )
    params = _params(KEY_DISPUTED, opts)
    spec = _spec(params, ArrangeOptions(counter="off", theme_octaves="on", levels={"bass": 0.5}))
    text = _decisions_text(params, spec)
    for fragment in (
        "key=profile→", "lead_octave=-1→", "bass_style=root, шаг 2",
        "bass_approach=off→0", "pad_style=off", "harmonic_rhythm=bar",
        "pad_register=mid", "density=sparse(измерено", "counter=off (задано)",
        "theme_octaves=on (задано)", "levels=bass×0.5",
    ):
        assert fragment in text, fragment


def test_harmonize_accepts_options_directly():
    """Ядро без RTTTL-пути: ``harmonize(..., options=)`` тоже исполняет ручки."""
    notes = [(69, 1.0), (72, 1.0), (76, 1.0), (72, 1.0)] * 2
    plain = harmonize(notes, 100, "A", "minor")
    pedal = harmonize(notes, 100, "A", "minor", options=HarmonizeOptions(bass_style="pedal"))
    assert {n for n, _d in pedal.bass} == {45}
    assert pedal.lead == plain.lead
    assert pedal.decisions["knob_bass_style"] == "pedal"
