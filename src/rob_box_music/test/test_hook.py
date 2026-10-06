"""PR-3a ADR-0149: хук из RTTTL — узнаётся, развивается по секциям, детерминирован; на событиях рендера."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import replace

import pytest

from melodies import CHROMATIC, LONG, MELODIES, PASSING, SHORT, SLOW, SPREAD, WIDE, compose_p, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange.compose import SECTION_BARS, hook_register
from rob_box_music.diversity import track_history
from rob_box_music.model import BEATS_PER_BAR, Key, TrackError, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.rtttl import parse_rtttl
from rob_box_music.tonality import key_fit

HOOK_REGISTER = hook_register(kn.STYLES["club"])


def _intervals(midis):
    return [b - a for a, b in zip(midis, midis[1:])]


def _lead_events(track):
    program = render(track, "A")
    _parsed, events = program_events(program.code, program.form_beats)
    slot = program.slots["lead"]
    # Лид в два голоса (``knowledge.SYNTH_STEREO``, #3465): второй голос — та же нота с задержкой Хааса меньше
    # 1/16 доли. Мелодию считаем по сетке 16-х, одна нота на (16-я, высота).
    seen, out = set(), []
    for e in events:
        key = (round(e.beat * 4), e.midi)
        if e.slot == slot and key not in seen:
            seen.add(key)
            out.append(e)
    return out


def _bars(events, first_bar, n_bars):
    """Ноты такт за тактом: [(доля в такте, MIDI), ...] для тактов ``first_bar … first_bar+n_bars-1``."""
    out = defaultdict(list)
    for e in events:
        bar = int(e.beat // BEATS_PER_BAR)
        if first_bar <= bar < first_bar + n_bars:
            out[bar - first_bar].append((round(e.beat % BEATS_PER_BAR, 3), e.midi))
    return [sorted(out[b]) for b in range(n_bars)]


@pytest.mark.parametrize("root", range(12))
@pytest.mark.parametrize("name", ["long", "slow"])
def test_hook_is_the_melody_start_transposed_with_intervals_kept(name, root):
    """Хук узнаётся: первые ноты мелодии в том же порядке и с теми же интервалами на любой тонике."""
    hook, key = hooks.from_rtttl(MELODIES[name], name, 132, root, "minor")
    _n, _bpm, notes = parse_rtttl(MELODIES[name])
    melody = [m for m, _d in notes if m is not None][:len(hook.notes)]
    assert _intervals([e.midi for e in hook.notes]) == _intervals(melody)
    assert hook.source == name and hook.bars in hooks.HOOK_BARS
    assert {e.midi % 12 for e in hook.notes} <= kn.scale_pitch_classes(key.root, key.mode)
    lo, hi = kn.REGISTERS["lead"]
    assert lo <= min(e.midi for e in hook.notes) and max(e.midi for e in hook.notes) <= hi


def test_major_theme_in_minor_plan_goes_to_the_relative_major():
    _hook, key = hooks.from_rtttl(LONG, "long", 132, 9, "minor")  # ля минор → до мажор
    assert (key.root, key.mode) == (0, "major")
    _hook, key = hooks.from_rtttl(SHORT, "short", 132, 9, "minor")
    assert (key.root, key.mode) == (9, "minor")


def test_slow_ringtone_is_scaled_to_the_track_tempo():
    """b=65 при треке 132: доли ×2, ноты на сетке 16-х, ритм тот же, что у LONG (там вдвое длиннее запись)."""
    assert hooks.time_scale(65, 132) == 2.0 and hooks.time_scale(130, 132) == 1.0 and hooks.time_scale(300, 132) == 0.5
    slow, _k = hooks.from_rtttl(SLOW, "slow", 132, 0, "minor")
    long_, _k = hooks.from_rtttl(LONG, "long", 132, 0, "minor")
    n = len(slow.notes)
    assert [(e.beat, e.dur_beats) for e in slow.notes] == [(e.beat, e.dur_beats) for e in long_.notes[:n]]
    assert all((e.beat * 4).is_integer() for e in slow.notes)


def test_short_melody_becomes_phrase_and_answer_in_the_scale():
    hook, key = hooks.from_rtttl(SHORT, "short", 132, 4, "minor")
    phrase = [e for e in hook.notes if e.beat < hooks.PHRASE_BARS * BEATS_PER_BAR]
    answer = [e for e in hook.notes if e.beat >= hooks.PHRASE_BARS * BEATS_PER_BAR]
    assert hook.bars == 4 and len(phrase) == len(answer)
    assert [e.beat + hooks.PHRASE_BARS * BEATS_PER_BAR for e in phrase] == [e.beat for e in answer]
    assert [e.midi for e in phrase] != [e.midi for e in answer], "ответ — на другой ступени, не повтор"
    assert {e.midi % 12 for e in hook.notes} <= kn.scale_pitch_classes(key.root, key.mode)


def test_out_of_scale_melody_is_refused_not_repaired():
    with pytest.raises(hooks.HookError, match="вне лада.*key_fit 0.58"):
        hooks.from_rtttl(CHROMATIC, "chroma", 132, 0, "minor")
    with pytest.raises(hooks.HookError, match="RTTTL"):
        hooks.from_rtttl("мусор", "x", 132, 0, "minor")


@pytest.mark.parametrize("root", range(12))
def test_passing_chromatic_notes_stay_in_the_hook(root):
    """I12 (#3409): проходящие полутона темы не повод отказа — key_fit ≥ 0.6, ноты и интервалы те же."""
    hook, key = hooks.from_rtttl(PASSING, "passing", 132, root, "minor", HOOK_REGISTER)
    _n, _bpm, notes = parse_rtttl(PASSING)
    melody = [m for m, _d in notes if m is not None][:len(hook.notes)]
    assert _intervals([e.midi for e in hook.notes]) == _intervals(melody)
    assert kn.HOOK_KEY_FIT_MIN <= hook.key_fit < 1.0
    assert hook.key_fit == key_fit([(e.midi, e.dur_beats) for e in hook.notes], kn.ROOTS[key.root], key.mode)
    assert {e.midi % 12 for e in hook.notes} - kn.scale_pitch_classes(key.root, key.mode), "хроматика не вычищена"


@pytest.mark.parametrize("root", range(12))
def test_wide_melody_is_folded_into_the_lead_corridor_by_octaves(root):
    """§3.3 (#3409): тема шире коридора — переносы октавой отдельных нот, не отказ; остальные интервалы целы."""
    hook, key = hooks.from_rtttl(WIDE, "wide", 132, root, "minor", HOOK_REGISTER)
    _n, _bpm, notes = parse_rtttl(WIDE)
    melody = [m for m, _d in notes if m is not None][:len(hook.notes)]
    got = [e.midi for e in hook.notes]
    assert all(HOOK_REGISTER[0] <= m <= HOOK_REGISTER[1] for m in got)
    assert len({(g - m) % 12 for g, m in zip(got, melody)}) == 1, "те же ноты, сдвиг одной транспозицией"
    shift = max(set(g - m for g, m in zip(got, melody)), key=[g - m for g, m in zip(got, melody)].count)
    folded = sum(g - m != shift for g, m in zip(got, melody))
    assert 0 < folded <= hooks.MAX_FOLDED_SHARE * len(got)
    kept = sum(a == b for a, b in zip(_intervals(got), _intervals(melody)))
    assert kept >= len(got) - 1 - 2 * folded, "перенос задевает только интервалы к перенесённой ноте"


def test_motif_that_folding_would_break_is_refused():
    with pytest.raises(hooks.HookError, match="мотив ломается"):
        hooks.from_rtttl(SPREAD, "spread", 132, 0, "minor", HOOK_REGISTER)


def test_diatonic_moves_a_chromatic_note_with_its_scale_neighbour():
    c_major = Key(0, "major")
    assert hooks.diatonic(61, c_major, 2) == 65 and hooks.diatonic(66, c_major, -2) == 63
    assert hooks.diatonic(64, c_major, 2) == 67


def test_lead_key_fit_is_checked_by_duration_not_note_by_note():
    """Валидатор (I12): лид с проходящими проходит, лид в чужом ладу (всё на полутон ниже) — нет."""
    track = compose_p(profile(hooks=("passing",)), 1, melodies={"passing": PASSING})
    assert track.hook is not None and track.hook.source == "passing"
    validate(track)
    lead = track.parts["lead"]
    off = tuple(replace(e, midi=e.midi - 1) for e in lead.pitches)
    with pytest.raises(TrackError) as err:
        validate(replace(track, parts={**track.parts, "lead": replace(lead, pitches=off)}))
    assert err.value.path == "parts.lead.pitches" and "в ладу" in err.value.reason


def test_hook_outcome_is_logged_with_key_fit(caplog):
    """I12: key_fit принятого хука и причина отказа — в логе (замер приёмки по логу, не по слуху)."""
    caplog.set_level("INFO", logger="rob_box_music.arrange.hook")
    hook, _key = hooks.from_rtttl(PASSING, "passing", 132, 0, "minor", HOOK_REGISTER)
    with pytest.raises(hooks.HookError):
        hooks.from_rtttl(CHROMATIC, "chroma", 132, 0, "minor")
    lines = [r.getMessage() for r in caplog.records]
    assert any("melody=passing" in m and f"key_fit={hook.key_fit:.2f}" in m for m in lines), lines
    assert any("melody=chroma" in m and "отказ" in m and "key_fit" in m for m in lines), lines


def test_a_theme_melody_becomes_the_hook_on_most_roots():
    """Тема отвергается только честно (не ложится в коридор лида/под ней нет места пэду), на большинстве тоник — хук."""
    for name in ("long", "short", "slow"):
        got = [compose_p(profile(root=r, hooks=(name,)), 1, melodies=MELODIES).hook for r in range(12)]
        assert sum(1 for h in got if h and h.source == name) >= 8, name


HOOKED_ROOTS = [r for r in range(12) if compose_p(profile(root=r, hooks=("long",)), 1, melodies=MELODIES).hook]


@pytest.mark.parametrize("track_no", [1, 2])  # первый трек — ``Style.opening_form`` (#3427), остальные — ``club48``
@pytest.mark.parametrize("root", HOOKED_ROOTS)
def test_track_hook_heard_in_drop_and_developed_by_sections(root, track_no):
    """drop = хук; build = только начало хука и пауза перед дропом; break — медленнее; drop2 — в терциях."""
    set_root = (root - 7 * (track_no - 1)) % 12  # тоника трека ``track_no`` = ``root`` (ход по квинтам)
    template = kn.STYLES[kn.DEFAULT_STYLE].opening_form if track_no == 1 else "club48"
    track = compose_p(profile(root=set_root, hooks=("long",)), track_no, set_seed=root, melodies=MELODIES,
                      template=template)
    validate(track)
    assert track.hook is not None and track.hook.source == "long"
    lead = _lead_events(track)
    sections = [(s.name, s.bars, s.energy, s.roles) for s in track.form.sections]
    starts = [sum(bars for _n, bars, _e, _r in sections[:i]) for i in range(len(sections))]
    start = {name: at for at, (name, _b, _e, _r) in zip(starts, sections)}
    hook_bars = [sorted((round(e.beat % 4, 3), e.midi) for e in track.hook.notes if int(e.beat // 4) == b)
                 for b in range(track.hook.bars)]
    assert _bars(lead, start["drop"], track.hook.bars) == hook_bars
    build = _bars(lead, start["build"], SECTION_BARS)
    assert build[-1] == [], "последний такт build — пауза перед дропом"
    assert build[:hooks.BUILD_BARS] == hook_bars[:hooks.BUILD_BARS] and build[2:4] == hook_bars[:2]
    brk = _bars(lead, start["break"], SECTION_BARS)
    assert sum(map(len, brk)) < sum(map(len, hook_bars)), "break реже drop"
    drop2 = _bars(lead, start["drop2"], SECTION_BARS)
    assert drop2 != _bars(lead, start["drop"], SECTION_BARS)
    assert {m for bar in drop2 for _b, m in bar} >= {m for bar in hook_bars for _b, m in bar}
    for name, bars, _e, _r in sections:
        if name.startswith(("intro", "outro")):
            assert all(not bar for bar in _bars(lead, start[name], bars)), name


def test_sections_are_not_identical_bars_of_one_loop():
    """Хук не повтор ×16: среди тактов лида с хуком больше разных, чем тактов в мотиве."""
    track = compose_p(profile(hooks=("long",)), 1, melodies=MELODIES)
    lead = _lead_events(track)
    bars = [tuple(b) for b in _bars(lead, 0, track.form.bars_total) if b]
    assert len(set(bars)) > track.hook.bars


@pytest.mark.parametrize("seed", range(20))
def test_hook_track_keeps_bass_off_kick_and_pad_under_lead(seed):
    track = compose_p(profile(root=seed % 12), seed % 3 + 1, set_seed=seed, melodies=MELODIES)
    program = render(track, "A")
    _parsed, events = program_events(program.code, program.form_beats)
    by = defaultdict(list)
    for e in events:
        by[{s: r for r, s in program.slots.items()}[e.slot]].append(e)
    kicks = {e.beat for e in by["kick"]}
    assert by["bass"] and not kicks & {e.beat for e in by["bass"]}, "бас не на бочке"
    assert max(e.midi for e in by["pad"]) <= min(e.midi for e in by["lead"]) - 3
    assert HOOK_REGISTER[0] <= min(e.midi for e in by["lead"]) or track.hook is None


def test_compose_is_deterministic_by_seed():
    a = compose_p(profile(), 2, set_seed=5, melodies=MELODIES)
    assert a == compose_p(profile(), 2, set_seed=5, melodies=MELODIES)
    assert render(a, "A") == render(compose_p(profile(), 2, set_seed=5, melodies=MELODIES), "A")


def test_last_played_hook_is_not_repeated_while_another_fits():
    for seed in range(10):
        first = compose_p(profile(), 1, set_seed=seed, melodies=MELODIES)
        second = compose_p(profile(), 1, set_seed=seed, melodies=MELODIES, history=[track_history(first)])
        assert second.hook.source != first.hook.source


def test_no_usable_melody_falls_back_to_the_lead_motif():
    track = compose_p(profile(hooks=("chroma",)), 1, melodies=MELODIES)
    assert track.hook is None and track.history_key.hook is None
    assert track.parts["lead"].pitches, "мотив лида звучит, тишины нет"
    assert compose_p(profile(), 1, melodies=None).hook is None


def test_n_seeds_give_different_tracks():
    """Разнообразие внутри одной темы: хук, тональность, прогрессия меняются от трека к треку."""
    tracks = [compose_p(profile(root=s % 12), s, set_seed=s, melodies=MELODIES) for s in range(1, 13)]
    assert len({t.hook.source for t in tracks}) >= 3
    assert len({(t.key.root, t.key.mode) for t in tracks}) >= 6
    assert len({t.history_key.progression for t in tracks}) >= 2
    assert len({render(t, "A").code for t in tracks}) == len(tracks)


def test_recent_hooks_of_past_sets_go_last():
    """I17 (#3399): хук недавних сетов (история в БД) — в конце очереди, давний раньше свежего."""
    import random

    from melodies import MELODIES, profile
    from rob_box_music.arrange.compose import hook_candidates

    prof = profile(hooks=("long", "short", "slow"))
    history = [{"melody_name": "other"}, {"melody_name": "long"}, {"melody_name": "slow"}]
    for seed in range(8):
        order = [h.source for h, _k in hook_candidates(prof, MELODIES, random.Random(seed), history)]
        assert order == ["short", "slow", "long"], seed
