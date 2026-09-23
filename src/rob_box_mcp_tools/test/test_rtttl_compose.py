"""Tests for ``core.rtttl_compose`` — RTTTL → flat ``compose_music`` params."""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_compose import (
    detect_key,
    melody_to_compose_params,
    rtttl_to_melody,
)


def test_rtttl_to_melody_parses_bpm_and_notes():
    melody = rtttl_to_melody("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    assert melody.bpm == 63
    assert melody.notes == (
        (None, 0.5),
        (79, 0.5),
        (79, 0.5),
        (79, 0.5),
        (75, 2.0),
    )


def test_melody_to_compose_params_fills_bpm_midi_dur():
    melody = rtttl_to_melody("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    params = melody_to_compose_params(melody)
    assert params["bpm"] == 63
    assert params["lead_midi"] == "None, 79, 79, 79, 75"
    assert params["lead_dur"] == "0.5, 0.5, 0.5, 0.5, 2"
    # Синт мелодии конвертер НЕ выбирает — его даёт LLM.
    assert "lead_synth" not in params
    # root/scale определяются по нотам — валидные значения для аранжировщика.
    assert params["root"] in ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
    assert params["scale"]


def test_detect_key_picks_major_for_diatonic_c_major():
    # C, D, E, F, G, A, B — чистая C-dur гамма.
    midi = [60, 62, 64, 65, 67, 69, 71]
    assert detect_key(midi) == ("C", "major")


def test_detect_key_picks_harmonic_minor_when_leading_tone_present():
    # A, B, C, D, E, F, G# — гармонический ля-минор. Натуральный ля-минор и
    # C-dur неразличимы по набору нот (относительные тональности), поэтому
    # «минорность» здесь доказывает именно повышенная VII ступень (G#).
    midi = [69, 71, 72, 74, 76, 77, 80]
    root, scale = detect_key(midi)
    assert root == "A"
    assert scale == "harmonicMinor"


def test_detect_key_ignores_rests_and_defaults_without_notes():
    assert detect_key([None, None]) == ("C", "major")


def test_detect_key_weights_by_duration_for_chromatic_melody():
    """Долгая тоника перевешивает проходящие ноты, а повышенная VII ступень
    (C#) отличает гармонический минор от относительного мажора."""
    midi = [74, 74, 74, 70, 77, 73, 74]  # D D D Bb F C# D
    durs = [1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 2.0]
    root, scale = detect_key(midi, durs)
    assert root == "D"
    assert scale == "harmonicMinor"


def test_known_melody_roundtrip_has_matching_midi_and_dur_lengths():
    """lead_midi и lead_dur обязаны иметь одинаковую длину — аранжировщик
    проверяет это и играет ноту в ноту."""
    melody = rtttl_to_melody("imperial:d=4,o=5,b=100:8g5,8g5,8g5,8d#6,16a#5,8g5,8d#6,16a#5,8g5")
    params = melody_to_compose_params(melody)
    midi_tokens = params["lead_midi"].split(",")
    dur_tokens = params["lead_dur"].split(",")
    assert len(midi_tokens) == len(dur_tokens) == len(melody.notes)


def test_melody_snapped_to_bar_with_tail_rest():
    """Мелодия некратной такту длины доводится хвостовой паузой до целого
    числа тактов — иначе луп плывёт относительно ударной сетки (#live 11.09)."""
    melody = rtttl_to_melody("x:d=8,o=5,b=100:8g5,8g5")
    params = melody_to_compose_params(melody)
    # 2 восьмые = 1.0 доля, не кратно 4 → хвостовая пауза 3.0.
    assert params["lead_midi"] == "79, 79, None"
    assert params["lead_dur"] == "0.5, 0.5, 3"


# ---------------------------------------------------------------------------
# #2839: бас эталонных тем держится лада
# ---------------------------------------------------------------------------

#: Эталонные темы из встроенной библиотеки (ключи ``RtttlLibrary.get``).
#: Гимн — тот самый, на котором 23.09 бас играл C F# G G# A G# G C#.
_REFERENCE_THEMES = (
    "national_2", "hallofth", "stilldre", "terminat", "mariobro",
)


@pytest.fixture(scope="module")
def reference_harmonies(tmp_path_factory):
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary

    db = tmp_path_factory.mktemp("rtttl") / "lib.db"
    library = RtttlLibrary(db_path=str(db))
    out = {}
    for key in _REFERENCE_THEMES:
        entry = library.get(key)
        assert entry is not None, f"эталонной темы {key} нет в библиотеке"
        params = melody_to_compose_params(rtttl_to_melody(entry["rtttl"]))
        out[key] = params["harmony"]
    return out


def _scale_pcs(harmony):
    from rob_box_mcp_tools.core.arranger import SCALE_INTERVALS, VALID_ROOTS

    root = VALID_ROOTS.index(harmony.root)
    return {(root + i) % 12 for i in SCALE_INTERVALS[harmony.scale]}


def _chord_under(harmony, beat):
    for chord in harmony.chords:
        if chord.start <= beat < chord.start + chord.beats:
            return chord
    return harmony.chords[-1]


@pytest.mark.parametrize("key", _REFERENCE_THEMES)
def test_reference_bass_stays_in_key(reference_harmonies, key):
    """Внеладовых нот баса (по длительности) не больше 10%.

    До #2839 хроматический подход длиной в целый шаг баса давал 20-39%
    внеладового баса на этих темах — на слух «лютый мусор».
    """
    harmony = reference_harmonies[key]
    scale = _scale_pcs(harmony)
    total = sum(dur for _note, dur in harmony.bass)
    outside = sum(
        dur for note, dur in harmony.bass
        if note is not None and note % 12 not in scale
    )
    assert outside / total <= 0.10, f"{key}: вне лада {outside / total:.1%}"


@pytest.mark.parametrize("key", _REFERENCE_THEMES)
def test_reference_bass_length_matches_theme(reference_harmonies, key):
    """Инвариант: сумма длительностей баса равна длине темы нота в ноту."""
    harmony = reference_harmonies[key]
    assert sum(d for _n, d in harmony.bass) == sum(d for _n, d in harmony.lead)


@pytest.mark.parametrize("key", _REFERENCE_THEMES)
def test_reference_bass_long_notes_are_in_key_or_in_chord(
    reference_harmonies, key
):
    """Внеладовая нота баса длиннее полубита допустима только как тон аккорда.

    Короткий проход — краска; длинная внеладовая нота — фальшь. Исключение
    — тон заимствованного аккорда, выбранного гармонизацией: там бас
    обязан играть вместе с подкладом, иначе разойдётся уже с ним.
    """
    harmony = reference_harmonies[key]
    scale = _scale_pcs(harmony)
    cursor = 0.0
    for note, dur in harmony.bass:
        if note is not None and dur > 0.5 and note % 12 not in scale:
            chord = _chord_under(harmony, cursor)
            assert note % 12 in chord.pitch_classes, (
                f"{key}: длинная внеладовая нота {note} на бите {cursor}"
            )
        cursor += dur


def test_approach_is_chromatic_only_across_a_pentatonic_gap():
    """Хроматика в подходе — только там, где у лада нет ступени ближе тона.

    В ля-минорной пентатонике (A C D E G) под корнем C нет ступени на
    полутон или тон ниже (B и A# вне лада), поэтому подход снизу —
    хроматический B, разрешающийся в корень на полтона. Сверху D —
    ступень лада, и если она ближе к звучащей ноте, берётся она.
    """
    from rob_box_mcp_tools.core.harmonize import _approach_note

    a_minor_pentatonic = frozenset({9, 0, 2, 4, 7})
    c3 = 48
    # Звучит A2 — снизу ближе, ступени нет → хроматический B2 на полтона.
    assert _approach_note(45, c3, a_minor_pentatonic) == 47
    # Звучит E3 — сверху ближе, ступень D3 в ладу.
    assert _approach_note(52, c3, a_minor_pentatonic) == 50


def test_o7_garbage_style_theme_is_transposed_into_working_register():
    """issue #2840: тема в o=7 (как мусорная ``russiann``) визжала в C7-G7 —
    ``imperialbrass([100, 98, 96, ...])``. После нормализации регистра
    максимум лида обязан лежать не выше MIDI 88."""
    melody = rtttl_to_melody(
        "RussianN:d=4,o=7,b=125:"
        "2e,d,c,2d,c,d,2e,g,e,1d,2e,d,c,2d,c,d,2e,g,e,1d"
    )
    before = max(m for m, _ in melody.notes)
    assert before >= 96  # до нормализации — реально в o=7 (визг)

    params = melody_to_compose_params(melody)
    tokens = params["lead_midi"].split(", ")
    lead_pitches = [int(tok) for tok in tokens if tok != "None"]
    assert max(lead_pitches) <= 88


def test_theme_already_in_working_register_is_not_shifted():
    """Тема, уже стоящая в рабочем регистре лида, не должна транспонироваться
    — ``lead_midi`` обязан остаться нота в ноту, иначе существующие лупы
    поплывут по высоте без всякой на то причины."""
    melody = rtttl_to_melody("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    params = melody_to_compose_params(melody)
    assert params["lead_midi"] == "None, 79, 79, 79, 75"
