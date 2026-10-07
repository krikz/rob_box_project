"""ADR-0154 PR-7: тема целиком в DJ-треке (запрос Шифу 07.10 «хотел бы услышать всю тему горного короля»).

В секции ``knowledge.THEME_SECTION`` (первый дроп) у трека с материалом звучит вся тематическая секция материала фраза
за фразой (``hook.theme_span``/``theme_cuts``), а не хук 4–8 тактов; секция растёт на длину темы, трек — не длиннее
``knowledge.TRACK_MAX_BARS``; гармония и бас — из материала на всю тему. RTTTL — вся мелодия до потолка, гармония —
Витерби по мелодии. Материал — синтетический (``test_harmony_material.synthetic``): ноты и аккорды известны заранее.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange import compose as cp
from rob_box_music.arrange import hook as hooks
from rob_box_music.model import BARS_TOTAL, BEATS_PER_BAR, blend_bars, validate
from test_harmony_material import _plan, synthetic

STYLE = kn.STYLES[kn.DEFAULT_STYLE]
#: Четыре фразы по 4 такта, каждая — своя ступень на 2 такта (аккорд слота ``CHORD_BARS`` известен).
THEME = (0, 0, 5, 5, 3, 3, 4, 4, 5, 5, 3, 3, 0, 0, 4, 4)


def material(degrees=THEME, phrase_bars=4, **kw) -> mt.ScoreMaterial:
    m = synthetic(degrees, **kw)
    phrases = tuple(mt.Phrase(b, phrase_bars, "new", 2 if b == 0 else 1) for b in range(0, len(degrees), phrase_bars))
    return replace(m, phrases=phrases)


def _section(track, name):
    start = 0
    for sec in track.form.sections:
        if sec.name == name:
            return start, sec.bars
        start += sec.bars
    raise KeyError(name)


def _compose(m, template=None, seed=7):
    plan = _plan(m.material_id, seed)
    if template:
        plan = replace(plan, tracks=(replace(plan.track(1), template=template),))
    track = cp.compose(plan, 1, materials={m.material_id: m})
    validate(track)
    return track


def _drop_lead(track):
    start, bars = _section(track, kn.THEME_SECTION)
    lo, hi = start * BEATS_PER_BAR, (start + bars) * BEATS_PER_BAR
    return [replace(e, beat=e.beat - lo) for e in track.parts["lead"].pitches if lo <= e.beat < hi]


def test_theme_of_four_phrases_plays_whole_in_the_first_drop():
    """16 тактов материала — 4 фразы — звучат в первом дропе нота в ноту: доли материала (4/4, темп трека), ступени
    лада — те же после переноса в тонику трека, фразы по порядку; хук — только первые 4 такта."""
    m = material()
    track = _compose(m)
    assert track.hook.bars == 4 and track.hook.theme_bars == 16
    start, bars = _section(track, kn.THEME_SECTION)
    assert bars >= 16 and track.form.bars_total in BARS_TOTAL
    got = [e for e in _drop_lead(track) if e.beat < 16 * BEATS_PER_BAR]
    want = [e for e in m.melody if e.beat < 16 * BEATS_PER_BAR]
    assert [e.beat for e in got] == [e.beat for e in want]
    shift = (track.key.root - m.key.root) % 12
    assert [e.midi % 12 for e in got] == [(e.midi + shift) % 12 for e in want]
    # внутри фразы интервалы — авторские (октава — своя у фразы, контур — нет)
    for p in m.phrases:
        a = [e.midi for e in got if p.bar * 4 <= e.beat < (p.bar + p.bars) * 4]
        b = [e.midi for e in want if p.bar * 4 <= e.beat < (p.bar + p.bars) * 4]
        assert [y - x for x, y in zip(a, a[1:])] == [y - x for x, y in zip(b, b[1:])]


def test_theme_harmony_and_bass_cover_the_whole_theme():
    """Ступени слотов темы — аккорды автора на всю тему (не петля хука); бас на тактах темы — прима аккорда темы."""
    m = material()
    track = _compose(m)
    slots = track.hook.theme_bars // cp.CHORD_BARS
    theme_degrees = [c.degree for c in track.harmony.progression[kn.THEME_SECTION][:slots]]
    assert theme_degrees == list(THEME[::cp.CHORD_BARS])
    loop = track.harmony.progression["drop2"]
    assert track.harmony.progression[kn.THEME_SECTION][slots:] == loop  # остаток секции — петля хука
    start, _bars = _section(track, kn.THEME_SECTION)
    scale = kn.SCALES[track.key.mode]
    for bar in range(16):
        notes = [e.midi % 12 for e in track.parts["bass"].pitches if int(e.beat // 4) == start + bar]
        root = (track.key.root + scale[THEME[bar]]) % 12
        assert notes and notes[0] == root, bar


def test_theme_follows_the_tempo_scale():
    """Материал вдвое быстрее трека (``material_scale`` ½): 32 такта материала — тема 16 тактов клуба, доли темы —
    доли материала × ½ (тот же множитель, что у хука, гармонии и баса)."""
    m = material(THEME + THEME, phrase_bars=8, bpm=264)
    assert hooks.material_scale(m, 132) == 0.5
    hook, _key = hooks.from_material(m, 132, 0, "major", cp.hook_register(STYLE), 32)
    assert hook.bars == 4 and hook.theme_bars == 16
    assert [e.beat for e in hook.theme] == [e.beat * 0.5 for e in m.melody]


def test_theme_is_cut_by_phrase_at_the_limit_and_track_stays_bounded():
    """40 тактов материала: тема режется по концу фразы на потолке (32), трек — 80 тактов; форма ``long64`` —
    потолок 24 (трек ≤ ``TRACK_MAX_BARS``)."""
    m = material(THEME + THEME + THEME[:8])
    track = _compose(m, "club48")
    assert track.hook.theme_bars == kn.THEME_MAX_BARS
    assert track.form.bars_total == kn.TRACK_MAX_BARS
    long = _compose(m, "long64")
    assert long.hook.theme_bars == cp.theme_limit(STYLE.forms["long64"]) == 24
    assert long.form.bars_total <= kn.TRACK_MAX_BARS and long.form.bars_total in BARS_TOTAL
    by8 = material(THEME + THEME + THEME[:8], phrase_bars=8)  # концы фраз 8, 16, 24 … — в потолке 24 ровно три
    assert _compose(by8, "long64").hook.theme_bars == 24


@pytest.mark.parametrize("template", sorted(STYLE.forms))
def test_theme_form_keeps_blend_and_outro(template):
    """Растянутая форма — тот же блэнд со следующим треком и тот же outro (валидатор A1/A4)."""
    m = material(THEME + THEME)
    track = _compose(m, template)
    plain = cp.compose(_plan(None), 1)
    assert track.hook.theme_bars > track.hook.bars
    assert blend_bars(track, plain) == blend_bars(plain, track) == STYLE.blend[0]


@pytest.mark.parametrize("theme_bars", range(0, kn.THEME_MAX_BARS + 1, 4))
def test_theme_form_lengths(theme_bars):
    for template, spec in STYLE.forms.items():
        limit = cp.theme_limit(spec)
        if theme_bars > limit:
            continue
        form = cp.theme_form(spec, theme_bars)
        total = sum(b for _n, b, _e, _r in form)
        drop = dict((n, b) for n, b, _e, _r in form)[kn.THEME_SECTION]
        assert total in BARS_TOTAL and drop >= theme_bars and drop % 4 == 0, (template, theme_bars)


def test_no_theme_without_limit_or_when_theme_is_the_hook():
    """Без потолка (годность материала в плане) — хук как до PR-7; материал на 8 тактов — тема не длиннее хука."""
    m = material()
    plain, _k = hooks.from_material(m, 132, 0, "major", cp.hook_register(STYLE))
    assert plain.theme == () and plain.theme_bars == 0
    short, _k = hooks.from_material(material(THEME[:8], 8), 132, 0, "major", cp.hook_register(STYLE), 32)
    assert short.theme == ()


#: RTTTL на 16 тактов восьмыми: четыре 4-тактовые фразы по трезвучиям до мажора (хук — первые 8 тактов).
_ARPS = ("c,e,g,e", "d,f,a,f", "e,g,b,g", "c,e,g,c6", "a4,c,e,c", "f,a,c6,a", "g,b,d6,b", "c6,g,e,c")
RTTTL_LONG = "long16:d=8,o=5,b=132:" + ",".join(a + "," + a for a in _ARPS + _ARPS[::-1])


def test_rtttl_theme_is_the_whole_melody_with_viterbi_harmony():
    """RTTTL на 16 тактов: хук — 8 тактов, тема — вся мелодия; трек с ней валиден, ступени темы — на всю тему."""
    hook, key = hooks.from_rtttl(RTTTL_LONG, "long16", 132, 0, "major", cp.hook_register(STYLE), 32)
    assert hook.bars == 8 and hook.theme_bars == 16
    plan = _plan(None)
    profile = replace(plan.profile, hook_ids=("long16",), theme_hooks=("long16",))
    track = cp.compose(replace(plan, profile=profile), 1, melodies={"long16": RTTTL_LONG})
    validate(track)
    assert track.hook.source == "long16" and track.hook.theme_bars == 16
    assert len(track.harmony.progression[kn.THEME_SECTION]) >= 16 // cp.CHORD_BARS
