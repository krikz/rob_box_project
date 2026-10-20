"""Годность партитуры к эталону темы: выравнивание нот с пропусками (#3573, ADR-0157 §1.2 п.6).

Два слоя. Синтетика: сравнение терпит лишнюю/потерянную ноту и перенос октавой там, где прежнее точное окно
интервалов ломалось. Размеченный набор: 15 партитур «Токкаты» с робота (7 из них — BWV 565 по названию и нотам,
остальные — чужие пьесы; ``data/toccata_gate.json``, melody/bass обрезаны до окон совпадений) против RTTTL-эталонов
хуков toccata/toccata_2/toccataa/toccataa_2 с робота (``/data/voice_memory.db``, 12.10.2026) — печатается таблица
«старый балл — новый балл — правда». Чистого разделения на этом наборе НЕТ (см. ``test_overlap_is_pinned``).
"""

import json
import os
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange import theme_align as ta
from rob_box_music.material import ScoreMaterial
from rob_box_music.model import Key, PitchEvent

_DATA = os.path.join(os.path.dirname(__file__), "data", "toccata_gate.json")

#: Опознавательные ноты BWV 565 (хук toccata): мотив с мордентом ``A–G–A–G–F–D–E–C♯–D–A–G–A``.
_MOTIF = [81, 79, 81, 79, 77, 74, 76, 73, 74, 81, 79, 81]


def _contour(pitches):
    return [b - a for a, b in zip(pitches, pitches[1:])]


def _fit(voice, motif=_MOTIF):
    got = ta.best_match({"melody": voice}, [ta.pitches(_contour(motif))])
    return got[0] if got else None


def _padded(notes):
    """Голос: чужая музыка до и после мотива (без его высотных классов) — окно ищется, а не лежит с начала."""
    return [60, 63, 68, 70, 71, 60, 63, 68, 70, 71, 60, 63] + notes + [59, 56, 60, 63, 68, 70, 71, 60, 63, 68]


# ── синтетика ────────────────────────────────────────────────────────────────────────────────────────────────

def test_identical_motif_scores_one_at_any_transposition():
    assert _fit(_padded(_MOTIF)) == 1.0
    assert _fit(_padded([m - 7 for m in _MOTIF])) == 1.0


def test_a_dropped_note_does_not_shift_the_rest_of_the_window():
    """Потерянная нота (мордент записан без второго G): остаток окна не считается промахом. Точное окно интервалов
    давало тут 4 из 11 — пьеса неотличима от чужой."""
    dropped = _MOTIF[:3] + _MOTIF[4:]
    assert _fit(_padded(dropped)) >= 0.85


def test_an_extra_note_does_not_shift_the_rest_of_the_window():
    extra = _MOTIF[:4] + [81] + _MOTIF[4:]
    assert _fit(_padded(extra)) >= 0.85


def test_a_repeated_note_is_an_extra_note():
    """«A–A–G» вместо «A–G»: повтор ноты в начале — вставка, а не сдвиг всего мотива."""
    assert _fit(_padded([_MOTIF[0]] + _MOTIF)) >= 0.85


def test_an_octave_transfer_inside_the_motif_keeps_the_match():
    """Каскад ``A5 → A4 → A3``: вторая половина мотива октавой ниже — совпадение по высотному классу."""
    cascade = _MOTIF[:6] + [m - 12 for m in _MOTIF[6:]]
    assert _fit(_padded(cascade)) >= 0.85


def test_unrelated_music_does_not_match():
    scale = [60 + (2 * i) % 12 + 12 * (i // 6 % 2) for i in range(200)]
    assert _fit(scale) < 0.7


def test_a_voice_shorter_than_the_motif_is_not_compared():
    assert _fit(_MOTIF[:-1]) is None
    assert ta.best_match({}, [ta.pitches(_contour(_MOTIF))]) is None
    assert ta.best_match({"melody": _MOTIF}, []) is None


def test_the_score_is_deterministic_and_inside_the_unit_interval():
    voice = _padded(_MOTIF[:3] + [66, 67] + _MOTIF[5:])
    assert _fit(voice) == _fit(list(voice))
    assert 0.0 <= _fit(voice) <= 1.0


def test_contour_fit_compares_two_ringtone_versions_of_one_theme():
    """``theme_version`` сравнивает контуры двух RTTTL одной темы тем же выравниванием."""
    a = _contour(_MOTIF)
    b = _contour(_MOTIF[:3] + _MOTIF[4:] + [79])  # другая запись: потеряна нота, добавлена в конце
    other = _contour([60, 64, 67, 72, 67, 64, 60, 62, 64, 65, 67, 69])
    assert ta.contour_fit(a, a) == 1.0
    assert ta.contour_fit(a, b) > ta.contour_fit(a, other)


# ── контракт theme_match ──────────────────────────────────────────────────────────────────────────────────────

def _material(melody, bass=(), mid="fixture:x"):
    ev = lambda rows: tuple(PitchEvent(int(m), float(b), float(d)) for m, b, d in rows)  # noqa: E731
    return ScoreMaterial(material_id=mid, title=mid, composer="", source="fixture", license="PD", meter=(4, 4),
                         bpm=100, key=Key(2, "minor"), melody=ev(melody), bass=ev(bass))


def test_theme_match_returns_score_voice_and_beat_of_the_start():
    rows = [(m, i * 0.5, 0.5) for i, m in enumerate(_padded(_MOTIF[:3] + _MOTIF[4:]))]
    refs = hooks.reference_contours(["toccata:d=16,o=5,b=140:" + ",".join(
        {81: "a", 79: "g", 77: "f", 74: "d", 76: "e", 73: "c#"}[m] for m in _MOTIF)])
    score, voice, beat = hooks.theme_match(_material(rows), refs)
    assert voice == "melody" and score >= 0.85
    assert beat == rows[12][1]  # мотив начинается после 12 нот чужой музыки


def test_theme_match_prefers_the_melody_on_a_tie_and_finds_the_theme_in_the_bass():
    rows = [(m, i * 0.5, 0.5) for i, m in enumerate(_padded(_MOTIF))]
    noise = [(60 + (i * 5) % 7, i * 0.5, 0.5) for i in range(60)]
    refs = hooks.reference_contours(["toccata:d=16,o=5,b=140:a,g,a,g,f,d,e,c#,d,a,g,a"])
    assert hooks.theme_match(_material(rows, rows), refs)[1] == "melody"  # ничья — мелодия
    assert hooks.theme_match(_material(noise, rows), refs)[1] == "bass"
    assert hooks.theme_match(_material(rows[:5]), refs) is None  # голос короче эталона


def test_for_theme_threshold_still_decides():
    rows = [(m, i * 0.5, 0.5) for i, m in enumerate(_padded(_MOTIF[:3] + _MOTIF[4:]))]
    m = _material(rows)
    ref = "toccata:d=16,o=5,b=140:a,g,a,g,f,d,e,c#,d,a,g,a"
    assert hooks.for_theme(m, [ref])[1] is not None  # потерянная нота — мотив узнан
    with pytest.raises(hooks.HookError, match="главного мотива"):
        hooks.for_theme(m, ["toccata:d=16,o=5,b=140:c,e,g,c6,g,e,c,e,g,c6,g,e"], match_min=kn.THEME_REF_MATCH_MIN)


# ── размеченный набор «Токкаты» ───────────────────────────────────────────────────────────────────────────────

def _load():
    with open(_DATA, encoding="utf-8") as fh:
        data = json.load(fh)
    rows = []
    for item in data["materials"]:
        ev = lambda voice: tuple(PitchEvent(int(m), float(b), float(d)) for m, b, d in item[voice])  # noqa: E731
        mat = ScoreMaterial(material_id=item["material_id"], title=item["title"], composer=item["composer"],
                            source="fixture", license="PD", meter=tuple(item["meter"]), bpm=item["bpm"],
                            key=Key(*item["key"]), melody=ev("melody"), bass=ev("bass"))
        rows.append((item, mat))
    return list(data["refs"].values()), rows


REFS, ROWS = _load()


def _score(mat):
    return hooks.theme_match(mat, hooks.reference_contours(REFS))[0]


def _split():
    pos = {i["id"]: _score(m) for i, m in ROWS if i["label"] == "bwv565"}
    neg = {i["id"]: _score(m) for i, m in ROWS if i["label"] == "other"}
    return pos, neg


def test_the_table_old_vs_new(capsys):
    """Печатает «материал — старый балл — новый балл — правда» (старый записан прежним кодом на полных партитурах)."""
    lines = ["", f"{'материал':10} {'правда':10} {'старый':>7} {'новый':>7}  название"]
    for item, mat in ROWS:
        lines.append(f"{item['id']:10} {item['label']:10} {item['old_score']:7.2f} {_score(mat):7.2f}  "
                     f"{item['title'][:50]}")
    with capsys.disabled():
        print("\n".join(lines))
    assert len(ROWS) == 15


def test_the_fixture_scores_equal_the_full_score_values():
    """Выжимка окон даёт тот же новый балл, что полная партитура (записан генератором на полных файлах)."""
    for item, mat in ROWS:
        assert _score(mat) == pytest.approx(item["new_score_full"], abs=1e-5), item["id"]


def test_every_strict_bwv565_passes_the_set_threshold():
    pos, _neg = _split()
    assert len(pos) == 6
    assert min(pos.values()) >= kn.THEME_REF_MATCH_MIN, pos


def test_the_exact_trill_theme_scores_one():
    """8Y5dGnPp и ecVWgjKR содержат трель toccataa целиком — 1.0 (как и до замены)."""
    pos, _neg = _split()
    assert pos["8Y5dGnPp"] == 1.0 and pos["ecVWgjKR"] == 1.0


def test_the_aligned_score_beats_the_exact_window_on_the_ornamented_arrangements():
    """Пять партитур одной аранжировки (мордент записан иначе, чем в эталоне): старый балл — 0.64, новый — выше."""
    gained = [(i["id"], i["old_score"], _score(m)) for i, m in ROWS if i["id"] in
              ("ahe4m23Z", "Abwz9NsB", "kKXwmG2h", "CnUiATx5", "epGSe9mH")]
    assert all(new > old + 0.15 for _id, old, new in gained), gained


def test_overlap_is_pinned():
    """ЧЕСТНО (#3573): при одном пороге разделения нет. Чужие пьесы не ниже худшей BWV 565 — только фуга BWV 564 до
    мажор (``jNAdByZX``, тот же 0.83 на эталоне-трели ``toccataa``); ``i1hJ7CuY`` (BWV 540) — 0.79, на волосок ниже.
    Тест фиксирует состояние: улучшат сравнение — он упадёт, и порог пересчитывается по таблице."""
    pos, neg = _split()
    floor = min(pos.values())
    assert sorted(k for k, v in neg.items() if v >= floor - 1e-9) == ["jNAdByZX"], (floor, neg)
    assert max(v for k, v in neg.items() if k != "jNAdByZX") < floor


@pytest.mark.xfail(strict=True, reason="#3573: BWV 564 (jNAdByZX) не отличить от BWV 565 одним порогом — 0.83 и там, "
                                       "и там; нужен другой признак (скелет в орнаментике, см. PR)")
def test_one_threshold_separates_bwv565_from_the_rest():
    pos, neg = _split()
    assert min(pos.values()) > max(neg.values()), (pos, neg)


def test_equal_materials_with_replaced_pitches_do_not_hit_a_stale_cache():
    """Кэш совпадений ключуется нотами: тот же id с другой мелодией — другой балл."""
    item, mat = next((i, m) for i, m in ROWS if i["id"] == "kKXwmG2h")
    scrambled = replace(mat, melody=tuple(replace(e, midi=60 + (k * 5) % 7) for k, e in enumerate(mat.melody)),
                        bass=())
    assert _score(scrambled) < _score(mat)
