"""Tests for ``core.rtttl_compose`` — RTTTL → flat ``compose_music`` params."""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_compose import (
    detect_key,
    melody_to_compose_params,
    rtttl_to_melody,
)
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary


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
    "national_2", "hallofth", "hallofth_2", "stilldre", "terminat", "mariobro",
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


def test_real_archive_known_themes_capped_after_register_normalize(tmp_path):
    """Регрессия живого прогона 23.09 (issue #2840): нормализация по одной
    медиане пропускала ``terminat`` — median=80 (уже в рабочем регистре,
    сдвиг 0), но max=99 (несколько высоких проходящих нот тянут потолок
    за собой, медиана их не видит). Проверяем на РЕАЛЬНОМ архиве
    (``RtttlLibrary()`` без ``archive_path`` — настоящий бандл), не на
    реконструированных записях."""
    lib = RtttlLibrary(db_path=str(tmp_path / "real_register.db"))
    keys = [
        "national_2", "hallofth", "stilldre",
        "terminat", "mariobro", "russiann",
    ]
    for key in keys:
        rec = lib.get(key)
        assert rec is not None, key
        melody = rtttl_to_melody(rec["rtttl"])
        params = melody_to_compose_params(melody)
        tokens = params["lead_midi"].split(", ")
        lead_pitches = [int(tok) for tok in tokens if tok != "None"]
        assert max(lead_pitches) <= 88, (key, lead_pitches)


# ---------------------------------------------------------------------------
# #2876: пэд не тонет в басе, затакт не рвёт лид скачком в две октавы
# ---------------------------------------------------------------------------

#: Живой прогон 23.09.2026 («диджей Снупдог»): ``stilldre_2`` — затакт на
#: MIDI 60 против тела фразы на 75-77 (скачок 15-17 полутонов), плюс
#: подклад ``strings`` (41-58) сидел прямо на басе ``moogbass`` (звучащий
#: диапазон 41-48 после компенсации ``SYNTH_SEMITONE_SHIFT``). Остальные
#: три темы — тот же архив, без затакта: регрессия на то, что фикс не
#: портит обычные темы.
_PAD_REGISTER_THEMES = ("stilldre_2", "national_2", "hallofth_2", "nextepis")


@pytest.fixture(scope="module")
def pad_register_harmonies(tmp_path_factory):
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary as _Lib

    db = tmp_path_factory.mktemp("rtttl_2876") / "lib.db"
    library = _Lib(db_path=str(db))
    out = {}
    for key in _PAD_REGISTER_THEMES:
        entry = library.get(key)
        assert entry is not None, f"эталонной темы {key} нет в библиотеке"
        assert entry["name"] == key, (key, entry["name"])
        params = melody_to_compose_params(rtttl_to_melody(entry["rtttl"]))
        out[key] = params["harmony"]
    return out


@pytest.mark.parametrize("key", _PAD_REGISTER_THEMES)
def test_pad_never_sinks_below_c3(pad_register_harmonies, key):
    """Пэд не опускается ниже MIDI 48 (C3) ни на одной ноте ни одного тона."""
    harmony = pad_register_harmonies[key]
    pad_notes = [
        note for tones, _dur in harmony.pad if tones is not None for note in tones
    ]
    assert pad_notes, key
    assert min(pad_notes) >= 48, (key, min(pad_notes))


@pytest.mark.parametrize("key", _PAD_REGISTER_THEMES)
def test_pad_never_overlaps_bass_range(pad_register_harmonies, key):
    """Диапазон пэда и диапазон баса (звучащая высота) не пересекаются.

    До #2876 у ``stilldre_2`` пэд (41-58) и бас (36-48) делили полосу
    41-48 целиком; потолок пэда знал только о теме, не о басе.
    """
    harmony = pad_register_harmonies[key]
    pad_notes = [
        note for tones, _dur in harmony.pad if tones is not None for note in tones
    ]
    bass_notes = [note for note, _dur in harmony.bass if note is not None]
    assert pad_notes and bass_notes, key
    assert min(pad_notes) > max(bass_notes), (
        key, "пэд", min(pad_notes), "бас", max(bass_notes),
    )


@pytest.mark.parametrize("key", _PAD_REGISTER_THEMES)
def test_lead_has_no_pickup_driven_octave_leap(pad_register_harmonies, key):
    """Соседние ноты лида не расходятся больше чем на октаву из-за затакта.

    До #2876 у ``stilldre_2`` затакт на MIDI 60 стоял вплотную к телу
    фразы на 75-77 — скачок 15-17 полутонов на каждом из четырёх повторов.
    """
    harmony = pad_register_harmonies[key]
    pitches = [note for note, _dur in harmony.lead if note is not None]
    assert len(pitches) >= 2, key
    leaps = [abs(a - b) for a, b in zip(pitches, pitches[1:])]
    assert max(leaps) <= 12, (key, max(leaps))


# ---------------------------------------------------------------------------
# #2873: тональность эталонных тем реального архива
# ---------------------------------------------------------------------------

#: ``ключ архива → (тоника, "major"|"minor")``. Гармонический минор
#: засчитывается как минор: его выбирает отдельное правило (вводный тон),
#: а здесь проверяется тональный центр. Обоснование — по нотам записи.
_KEY_REFERENCE = {
    # Григ, «В пещере горного короля»: B C# D E F# D F# — 1-2-♭3-4-5 си-минора
    # от первой ноты; дальше хроматическая секвенция (F C# F · E C E), фраза
    # обрывается на A (VII). До #2873 — ля-мажор (живой прогон 23.09).
    "hallofth_2": ("B", "minor"),
    "mountainking": ("B", "minor"),  # та же тема, другая запись
    # Та же тема в ля-миноре: A B C D E C E, секвенция D# B D# · D A# D.
    "hallofth": ("A", "minor"),
    "hallofth_3": ("A", "minor"),
    # И в до-миноре: C D E♭ F G E♭ G.
    "inthehal": ("C", "minor"),
    # Гимн России: затакт G, сильная доля — C; F натуральный, конец на V.
    "national_2": ("C", "major"),
    # Бетховен, «К Элизе»: E D# E D# E B D C A — ля-минор, G# — вводный тон.
    "furelise": ("A", "minor"),
    # «Ода к радости»: A A B♭ C C B♭ A G F F G A — 3-3-4-5-5-4-3-2-1 фа-мажора.
    "odetojoy": ("F", "major"),
    # Та же ода в ре-мажоре: F# F# G A A G F# E D D E F#.
    "odetojoy_2": ("D", "major"),
    # Пятая Бетховена тоном ниже оригинала: F F F D♭ · E♭ E♭ E♭ C — си-♭-минор.
    "5thsymph": ("A#", "minor"),
    # Имперский марш квартой ниже: D D D B♭ F D B♭ F D — ре-минор (B♭ = ♭6).
    "imperial": ("D", "minor"),
    # «Тетрис» (Коробейники): E B C D C B A A C E D C B — ля-минор.
    "tetris": ("A", "minor"),
    "tetris_2": ("A#", "minor"),  # та же тема полутоном выше
    # Super Mario, главная тема: E E E C E G — до-мажор.
    "supermar_4": ("C", "major"),
    # Бах, «Иисус — моя радость»: G G A B D C C E D D G F# G — соль-мажор.
    "jesujoyo": ("G", "major"),
    # Моцарт, «Маленькая ночная серенада»: G…B A G, F# вместо F — соль-мажор.
    "eineklei": ("G", "major"),
    # Бах, «Шутка» (Badinerie): B D B F# B F# D F# D B — си-минор.
    "badineri": ("B", "minor"),
    # Моцарт, 40-я: B♭ A G G F E♭ E♭ D C — соль-минор.
    "mozart40": ("G", "minor"),
    # Вивальди, «Зима»: A♭ B♭ A♭ G F G F E♭ — фа-минор.
    "4seasons_5": ("F", "minor"),
    # Верди, «Сердце красавицы»: D# D# D# F# E C# · C# C# C# E D# B — си-мажор.
    "ladonnae": ("B", "major"),
    # Марш из «Индианы Джонса»: E F G C · D E F — до-мажор; и полутоном выше.
    "indianaj_4": ("C", "major"),
    "raidersm": ("C#", "major"),
    # Бонд-вамп: C D D D D C C C C E♭ E♭ E♭ E♭ D D D C — до-минор.
    "james_bond": ("C", "minor"),
    # «Миссия невыполнима»: E♭ C G · E♭ C F# — до-минор (C — тоника вампа).
    "missionimpossi": ("C", "minor"),
    # «Розовая пантера»: D# E F# G D# E F# G C B D# E — ми-минор.
    "pinkpant": ("E", "minor"),
    # Моцарт, «Турецкое рондо»: B A G# A C D C B C E — ля-минор.
    "rondoala": ("A", "minor"),
    # Still D.R.E.: A A A … G G G — ля-минор; «Next Episode»: D A A G A G F.
    "stilldre": ("A", "minor"),
    "nextepis": ("D", "minor"),
    # Terminator: педаль G#, A# B G# (1-2-♭3), F# — VII натурального минора.
    "terminat": ("G#", "minor"),
    # Мажорные темы, начатые с ТЕРЦИИ: первая нота указывает на минор от
    # терции, тоника держится гистограммой и затактом.
    # «Jingle Bells»: A A A · A A A · A C F G A — фа-мажор (живой фикс 14.09).
    "jinglebe_2": ("F", "major"),
    # «The First Noel»: E D C D E F G — до-мажор.
    "thefirst": ("C", "major"),
    # «White Christmas»: F# G F# F F# G G# A — ре-мажор.
    "whitechr": ("D", "major"),
    # «Rudolph»: G A G E C A G — 5-6-5-3-1-6-5 до-мажора.
    "rudolpht_2": ("C", "major"),
    "rudolpht_4": ("C", "major"),
}

#: Известные промахи — фиксируем честно, а не прячем. ``strict``: если
#: определение улучшат, тест упадёт и попросит перенести тему в таблицу.
_KEY_KNOWN_MISSES = {
    # Токката ре-минор: A G A(4 доли) G F D E C#(2) D — начинается с
    # выдержанной доминанты A, у ре-минора C# вне натурального лада.
    # Сейчас — ля-мажор (на origin/develop тоже ля-мажор).
    "toccata": ("D", "minor"),
    # «Jingle Bells» в соль-мажоре: B B B · B B B · B D G A B — вся тема на
    # терции B, тоника G звучит один раз; выигрывает си-минор. На
    # origin/develop было верно — регрессия #2873, принята сознательно.
    "jinglebe_5": ("G", "major"),
}


@pytest.fixture(scope="module")
def key_library(tmp_path_factory):
    db = tmp_path_factory.mktemp("rtttl_keys") / "lib.db"
    return RtttlLibrary(db_path=str(db))


def _detected_key(library, name):
    entry = library.get(name)
    assert entry is not None, f"темы {name} нет в архиве"
    params = melody_to_compose_params(rtttl_to_melody(entry["rtttl"]))
    family = "major" if params["scale"] == "major" else "minor"
    return params["root"], family


@pytest.mark.parametrize("name, expected", sorted(_KEY_REFERENCE.items()))
def test_reference_theme_key(key_library, name, expected):
    """issue #2873: тоника и лад эталонных тем реального архива."""
    assert _detected_key(key_library, name) == expected


@pytest.mark.parametrize("name, expected", sorted(_KEY_KNOWN_MISSES.items()))
@pytest.mark.xfail(strict=True, reason="известный промах #2873")
def test_known_key_misses(key_library, name, expected):
    assert _detected_key(key_library, name) == expected


def test_opening_on_tonic_beats_long_final_subtonic():
    """Позиционная опора (#2873) без архива: тема начинается с 1-2-♭3-4-5
    си-минора, а кончается долгим A (VII ступень). Первая нота и начало
    фразы держат си-минор против гистограммы."""
    b, cs, d, e, fs, a = 71, 73, 74, 76, 78, 81
    midi = [b, cs, d, e, fs, d, fs, b, a, fs, d, fs, a]
    durs = [0.25] * 12 + [2.0]
    root, scale = detect_key(midi, durs)
    assert root == "B"
    assert scale in ("minor", "harmonicMinor")


def test_pickup_on_dominant_does_not_become_tonic():
    """Затакт (короткая нота перед долгой) не утверждает тонику: гимн
    начинается с G/8 перед C/4, тональность — до-мажор, не соль."""
    g4, c5, e5, f5 = 67, 72, 76, 77
    midi = [g4, c5, g4, e5, f5, e5, c5, g4]
    durs = [0.5, 1.0, 0.5, 1.0, 0.5, 0.5, 1.0, 1.0]
    assert detect_key(midi, durs) == ("C", "major")
