"""#3542 (утро 08.10): тема целиком включается и у RTTTL-хука, а материал не отвергается узким коридором лида.

(а) «клубный сет на тему в пещере горного короля» — хук ``hallofth_2`` (ринг-тон в 4 такта, тема не длиннее хука):
тема целиком — из самой длинной записи того же произведения среди найденных (``hook.theme_version``: начало
совпадает контуром, как эталон у материала). (б) «лоуфай сет на тему Моцарт» — материал отвергнут «7 из 20 нот не
помещаются в коридор»: октава своя у двух тактов, затем у такта (``hook.fit_register``), отказ — только если и это не
помогает; критерий один у хука, темы и годности материала. (в) материал без главного мотива эталона уступает трек
следующему годному в плане (``hook.material_unfit`` с эталонами), а не RTTTL-хуку. Мелодии — свои, синтетические.
"""

from __future__ import annotations

from dataclasses import replace

from rob_box_music import knowledge as kn
from rob_box_music.arrange import compose as cp
from rob_box_music.arrange import hook as hooks
from rob_box_music.diversity import track_composition
from rob_box_music.model import BEATS_PER_BAR, PitchEvent, validate
from rob_box_music.rtttl import parse_rtttl
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile
from test_harmony_material import synthetic

STYLE = kn.STYLES[kn.DEFAULT_STYLE]
REGISTER = cp.hook_register(STYLE)
_NAMES = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")
#: Тема на 16 тактов восьмыми — четыре 4-тактовые фразы по трезвучиям до мажора (как ``test_theme_whole``).
_ARPS = ("c,e,g,e", "d,f,a,f", "e,g,b,g", "c,e,g,c6", "a4,c,e,c", "f,a,c6,a", "g,b,d6,b", "c6,g,e,c")
LONG = "long:d=8,o=5,b=132:" + ",".join(a + "," + a for a in _ARPS + _ARPS[::-1])
_LONG_PITCHES = [m for m, _d in parse_rtttl(LONG)[2] if m is not None]


def _tune(name: str, pitches, dur: int = 8, bpm: int = 132) -> str:
    return f"{name}:d={dur},o=5,b={bpm}:" + ",".join(f"{_NAMES[m % 12]}{m // 12 - 1}" for m in pitches)


#: Ринг-тон той же темы — только первые 8 тактов, на тон выше, шестнадцатыми в темпе вдвое ниже (та же скорость нот):
#: так в архиве ``hallofth_2`` (4 такта, b=140) против ``hallofth`` (вся тема, b=260).
SHORT = _tune("short", [m + 2 for m in _LONG_PITCHES[:64]], dur=16, bpm=66)
#: Длинная мелодия другого произведения (гаммы) — не версия темы.
SCALES = "scales:d=8,o=5,b=132:" + ",".join(["c,d,e,f,g,a,b,c6,b,a,g,f,e,d,c,d"] * 16)


def _profile(hook_ids, materials=(), mode="major") -> ThemeProfile:
    return ThemeProfile("тест", kn.DEFAULT_STYLE, 132, 0, mode, tuple(hook_ids), None, tuple(hook_ids),
                        materials=tuple(materials))


# ── (а) RTTTL-хук: тема целиком из длинной записи того же произведения ───────────────────────────────────────────

def test_theme_version_is_the_longest_record_of_the_same_tune():
    melodies = {"short": SHORT, "long": LONG, "scales": SCALES}
    assert hooks.theme_version("short", melodies, 132) == "long"
    assert hooks.theme_version("long", melodies, 132) == "long"
    assert hooks.theme_version("short", {"short": SHORT, "scales": SCALES}, 132) == "short"  # гаммы — не та тема


def test_short_ringtone_alone_has_no_theme_beyond_the_hook():
    """Причина утра 08.10: ринг-тон хука — 8 тактов = хук, «вся мелодия» не длиннее хука — темы нет."""
    hook, _key = hooks.from_rtttl(SHORT, "short", 132, 0, "major", REGISTER, 32)
    assert hook.bars == 8 and hook.theme == ()


def test_short_ringtone_plays_the_whole_theme_of_the_longer_record_in_the_track():
    """Трек 1 на хуке ``short`` (хук №1 темы) — в первом дропе вся тема из ``long``: 16 тактов, все 128 нот, высоты
    — те же ступени в тонике трека, что у хука (первая нота темы — первая нота хука), гармония на всю тему."""
    plan = seeded_plan(_profile(("short", "long")), 7)
    track = cp.compose(plan, 1, melodies={"short": SHORT, "long": LONG})
    validate(track)
    hook = track.hook
    assert hook.source == "short" and hook.bars == 8 and hook.theme_bars == 16
    assert len(hook.theme) == len(_LONG_PITCHES)
    shift = (hook.notes[0].midi - _LONG_PITCHES[0]) % 12
    assert [e.midi % 12 for e in hook.theme] == [(m + shift) % 12 for m in _LONG_PITCHES]
    assert [e.midi % 12 for e in hook.theme[:len(hook.notes)]] == [e.midi % 12 for e in hook.notes]
    assert [e.beat for e in hook.theme[:4]] == [0.0, 0.5, 1.0, 1.5]  # восьмые, как у хука (свой time_scale записи)
    assert len(track.harmony.progression[kn.THEME_SECTION]) >= 16
    assert track_composition(track)["theme_bars"] == 16  # лог ``started`` на роботе: тема целиком видна


# ── (б) широкий материал: октава по такту вместо отказа ───────────────────────────────────────────────────────

def _wide(material_id="local:wide"):
    """Материал 8 тактов, нечётные такты — на две октавы выше: целиком (и по два такта) в коридор лида не встаёт,
    половина нот — вне коридора; каждый такт — встаёт."""
    m = synthetic((0, 0, 5, 5, 3, 3, 4, 4))
    melody = tuple(replace(e, midi=e.midi + (24 if int(e.beat // BEATS_PER_BAR) % 2 else 0)) for e in m.melody)
    return replace(m, melody=melody, material_id=material_id)


def test_wide_material_motif_gets_an_octave_per_bar_instead_of_refusal():
    m = _wide()
    assert hooks.material_unfit(m, 132, 0, "major", REGISTER) is None
    hook, _key = hooks.from_material(m, 132, 0, "major", REGISTER)
    got = list(hook.notes)
    assert all(REGISTER[0] <= e.midi <= REGISTER[1] for e in got)
    for bar in range(hook.bars):  # внутри такта — интервалы автора, ноты по порядку
        a = [e.midi for e in got if int(e.beat // BEATS_PER_BAR) == bar]
        b = [e.midi for e in m.melody if int(e.beat // BEATS_PER_BAR) == bar]
        assert [y - x for x, y in zip(a, a[1:])] == [y - x for x, y in zip(b, b[1:])], bar


def test_fit_register_takes_the_coarsest_layout_that_folds_few_notes():
    notes = [(float(i), 60 + 24 * (i // 4 % 2) + i % 4) for i in range(16)]  # 4 такта, такт через такт +24
    whole, folded, span = hooks.fit_register(notes, [16.0], 0, REGISTER)
    assert span == 1 and folded <= hooks.MAX_FOLDED_SHARE * len(whole)
    near = [(float(i), 64 + i % 5) for i in range(16)]
    assert hooks.fit_register(near, [16.0], 0, REGISTER)[1:] == (0, 0)  # помещается целиком — как до #3542


def test_short_phrase_answer_is_folded_into_the_corridor_instead_of_refusal():
    """Короткая мелодия (фраза + ответ на IV/V): фраза во весь коридор, ответ ни на одной ступени целиком не встаёт —
    ответ с переносом октавой не больше ``hook.MAX_FOLDED_SHARE`` нот, а не отказ «ответ не помещается»."""
    wide = "wide:d=8,o=5,b=132:d,g,c6,e6,g6,c7,g6,e6,c6,g,d,g"  # ре5..до7 — весь коридор (62, 84)
    hook, _key = hooks.from_rtttl(wide, "wide", 132, 0, "major", REGISTER)
    assert hook.bars == 4 and len(hook.notes) == 24
    assert all(REGISTER[0] <= e.midi <= REGISTER[1] for e in hook.notes)


def test_compose_plays_the_wide_material_not_the_ringtone():
    m = _wide()
    plan = seeded_plan(_profile(("long",), (m.material_id,)), 7, materials={m.material_id: m}, references=[LONG])
    assert plan.track(1).material == m.material_id
    track = cp.compose(plan, 1, melodies={"long": LONG}, materials={m.material_id: m})
    validate(track)
    assert track.hook.source == m.material_id


# ── (в) материал без главного мотива — следующий годный, а не RTTTL ───────────────────────────────────────────

def _scales_material():
    """Материал без мотива эталона: гаммы четвертями (эталон — арпеджио материала ``local:arps``)."""
    m = synthetic((0, 0, 5, 5, 3, 3, 4, 4))
    steps = (0, 2, 4, 5, 7, 9, 11, 12, 11, 9, 7, 5, 4, 2, 0, 2)
    melody = tuple(PitchEvent(60 + steps[i % len(steps)], float(i), 1.0, 2) for i in range(32))
    return replace(m, melody=melody, material_id="local:scales")


def test_plan_gives_track_one_to_the_next_material_with_the_main_motif():
    first, second = _scales_material(), replace(synthetic((0, 0, 5, 5, 3, 3, 4, 4)), material_id="local:arps")
    ref = _tune("ref", [e.midi for e in second.melody[:13]], dur=4, bpm=120)
    materials = {first.material_id: first, second.material_id: second}
    assert "главного мотива" in hooks.material_unfit(first, 132, 0, "major", REGISTER, [ref])
    assert hooks.material_unfit(second, 132, 0, "major", REGISTER, [ref]) is None
    profile = _profile(("ref",), (first.material_id, second.material_id))
    rejected = {}
    plan = seeded_plan(profile, 7, 2, materials=materials, rejected=rejected, references=[ref])
    assert plan.track(1).material == second.material_id and "главного мотива" in rejected[first.material_id]
    track = cp.compose(plan, 1, melodies={"ref": ref}, materials=materials)
    assert track.hook.source == second.material_id
