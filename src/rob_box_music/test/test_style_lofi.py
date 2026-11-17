"""Стиль ``lofi`` — запись ``knowledge.STYLES`` (ADR-0153 S4, переделка #3557 после «шляпы» 13.10): грув — замедленный
брейк-луп пака и бочка, свинг 16-х нот 0.58–0.65, Rhodes держит септаккорд такта, бас — прима/квинта нотами 1–2 доли,
лид дышит, шипение пластинки каждый такт, петли по 4 такта без подъёма и дропа. Поведение — на
``SetPlan``/``Track``/событиях программы Renardo (симулятор ``render.events``). Клуб побайтно тот же —
``test_style_same_tracks``.
"""

from __future__ import annotations

import dataclasses

import pytest

from melodies import MELODIES
from rob_box_music import knowledge as kn
from rob_box_music.arrange import harmony
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import track_composition
from rob_box_music.model import BEATS_PER_BAR, STEPS_PER_BAR, Stereo, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import RenderError, render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import match_style_text, seeded_profile

LOFI = kn.STYLES["lofi"]
#: Свинг 16-х (MPC): нечётная 16-я стоит на этой доле восьмой (0.5 — ровно, 0.67 — триоль).
SWING16 = (0.58, 0.65)


def _plan(theme: str, seed: int, hooks: bool = False):
    profile = seeded_profile(theme, "lofi")
    profile = dataclasses.replace(profile, hook_ids=tuple(MELODIES)) if hooks else profile
    return seeded_plan(profile, seed, set_id=f"lo{seed}")


@pytest.fixture(scope="module")
def tracks():
    """3 сида × 2 трека (сид 0 и 2 — хуки RTTTL темы, сид 1 — мотив лида): (план, трек, программа, события по ролям)."""
    out = []
    for seed in range(3):
        plan = _plan("дождливый вечер", seed, hooks=seed != 1)
        history: list = []
        for no in (1, 2):
            track = compose(plan, no, history=history, melodies=MELODIES if seed != 1 else None)
            validate(track)
            program = render(track, "A")
            _p, events = program_events(program.code, program.form_beats, loops=True)
            role_of = {slot: role for role, slot in program.slots.items()}
            by_role: dict = {}
            for ev in events:
                if ev.amp > 0:
                    by_role.setdefault(role_of[ev.slot], []).append(ev)
            out.append((plan, track, program, by_role))
            history.insert(0, track_composition(track))
    return out


def _late_beats(plan) -> float:
    return round(plan.swing * 30000 / plan.bpm) * plan.bpm / 60000  # offset_ms — целые мс


def test_tempo_window_and_sixteenth_swing_of_the_set():
    """Темп сета — в окне стиля (72–92), свинг — нечётные 16-е на 0.58–0.65 восьмой (а не оффбит-восьмые)."""
    bpms = [w.bpm for w in LOFI.genre_windows.values()]
    assert (min(b for b, _ in bpms), max(b for _, b in bpms)) == (72, 92)
    assert kn.genre_style(LOFI, next(iter(LOFI.genre_windows))) == LOFI, "поля стиля — первое окно"
    assert LOFI.swing_steps == tuple(range(1, STEPS_PER_BAR, 2)), "качаются нечётные 16-е"
    for seed in range(20):
        plan = _plan("дождливый вечер", seed)
        window = LOFI.genre_windows[plan.genre]
        assert window.bpm[0] <= plan.bpm <= window.bpm[1], (seed, plan.genre, plan.bpm)
        assert SWING16[0] <= 0.5 + plan.swing <= SWING16[1], (seed, plan.swing)


def test_groove_is_a_slowed_pack_break_with_a_kick_and_no_hat_loop(tracks):
    """Грув петель — брейк-луп пака (роль ``loop``, каталог ``break``): событие на каждой восьмой его секций, ``tempo=``
    — темп брейка (замедлен под сет); бочка в петлях с бочкой; клэпа нет (малый — в брейке); хэты — только в окнах
    блэнда, восьмыми, с «руками» (сдвиги разные, ни один не раньше сетки)."""
    for plan, track, program, by_role in tracks:
        loop = track.parts["loop"]
        info = kn.SAMPLE_CATALOG[loop.synth_or_sample]
        assert info.role == "break" and info.bpm > plan.bpm, (info.name, plan.bpm)
        assert f"tempo={info.bpm}" in program.code
        bars = [sec.bars for sec in track.form.sections if "loop" in sec.roles]
        assert bars and len(by_role["loop"]) == sum(bars) * 8, "кусок брейка — каждая восьмая"
        assert "clap" not in track.parts and all("clap" not in sec.roles for sec in track.form.sections)
        hat_sections = {sec.name for sec in track.form.sections if "hats" in sec.roles}
        assert hat_sections <= {"intro", "outro_tail"}, hat_sections
        grid = track.parts["hats"].grid.steps
        assert all(i % 2 == 0 for i, st in enumerate(grid) if st.on), "хэт — восьмыми"
        assert len({st.offset_ms for st in grid if st.on}) > 3 and all(st.offset_ms >= 0 for st in grid)


def test_tonal_roles_share_the_sixteenth_groove(tracks):
    """Свинг — грув ролей ``Style.swing_roles``: ноты баса, пэда и лида — на сетке 16-х, нечётные 16-е — позже на
    свинг; на прямой нечётной 16-й — ни одной."""
    for plan, track, _program, by_role in tracks:
        late = _late_beats(plan)
        allowed = (0.0, 0.25 + late, 0.5, 0.75 + late, 1.0)
        for role in ("bass", "pad", "lead"):
            assert by_role.get(role), (role, track.track_id)
            for e in by_role[role]:
                assert min(abs(e.beat % 1 - f) for f in allowed) < 1e-3, (role, e.beat % 1, late)


def test_keys_hold_seventh_chords_one_per_bar(tracks):
    """Клавиши (Rhodes) держат аккорд: ни одного аккорда короче такта, смена не чаще такта; септаккорд ступени."""
    for _plan_, track, _program, _by in tracks:
        assert track.history_key.pad_figure == "held"
        pad = track.parts["pad"]
        assert pad.synth_or_sample == "rhpiano"
        assert all(e.dur_beats >= BEATS_PER_BAR for e in pad.pitches), "аккорд держится ≥ такта"
        assert all(e.beat % BEATS_PER_BAR == 0 for e in pad.pitches), "смена — на первой доле такта"
        for chords in track.harmony.progression.values():
            for chord in chords:
                pcs = harmony.chord_pcs(LOFI, track.key, chord.degree, chord.quality or None)
                assert len(pcs) == 4 and {m % 12 for m in chord.voicing} == set(pcs), (chord, pcs)


def test_bass_plays_roots_and_fifths_in_long_notes(tracks):
    """Бас — рифф стиля: ноты в 1–2 доли на восьмых, тон — прима или квинта аккорда такта, регистр читаемый на
    колонках робота (``registers["bass"]`` от 40)."""
    for _plan_, track, _program, _by in tracks:
        assert track.history_key.bass_figure == "riff"
        chords = {}
        bar = 0
        for sec in track.form.sections:
            for k, chord in enumerate(track.harmony.progression[sec.name]):
                chords[bar + k] = chord
            bar += sec.bars
        for e in track.parts["bass"].pitches:
            assert e.dur_beats >= 1.0, e
            assert (e.beat * 2) % 1 == 0 and LOFI.registers["bass"][0] <= e.midi <= LOFI.registers["bass"][1], e
            chord = chords[int(e.beat // BEATS_PER_BAR)]
            pcs = harmony.chord_pcs(LOFI, track.key, chord.degree, chord.quality or None)
            assert e.midi % 12 in (pcs[0], pcs[2]), (e, pcs)
        assert all(e.dur_beats <= 2.0 for e in track.parts["bass"].pitches), "не дольше половинной"


def test_vinyl_hiss_plays_every_bar_of_the_form(tracks):
    """Шипение пластинки — роль ``fx`` из подложек пака (``texture``) во всех секциях, удар каждого такта
    (``Style.fx_bars``), звучит до следующего (файл длиннее такта)."""
    for _plan_, track, program, by_role in tracks:
        name = track.parts["fx"].synth_or_sample
        assert name in kn.PACK_TEXTURES and kn.SAMPLE_CATALOG[name].role == "texture"
        assert all("fx" in sec.roles for sec in track.form.sections)
        bars = track.form.bars_total
        assert sorted(round(e.beat) for e in by_role["fx"]) == [b * BEATS_PER_BAR for b in range(bars)]
        assert all(e.sus_beats == BEATS_PER_BAR for e in by_role["fx"])
        assert kn.SAMPLE_CATALOG[name].path in program.sample_files


def test_lead_breathes_outside_the_theme(tracks):
    """Лид молчит в тактах ``.`` маски ``Style.lead_mask`` (по кругу от начала секции; тема целиком — без пауз)."""
    mask = LOFI.lead_mask
    assert "." in mask and "x" in mask
    for _plan_, track, _program, _by in tracks:
        onset_bars = {int(e.beat // BEATS_PER_BAR) for e in track.parts["lead"].pitches}
        bar = 0
        for sec in track.form.sections:
            theme = track.hook.theme_bars if (track.hook and track.hook.theme
                                              and kn.section_kind(sec.name) == kn.THEME_SECTION) else 0
            if "lead" in sec.roles:
                silent = {bar + k for k in range(theme, sec.bars) if mask[(k - theme) % len(mask)] == "."}
                assert not onset_bars & silent, (track.track_id, sec.name)
            bar += sec.bars


def test_form_is_eight_bar_loops_changing_one_element(tracks):
    """Форма — интро/хвост блэнда по 4 и петли по 4 такта (тема — 8); соседние петли различаются составом (меняется
    элемент: брейк, бочка, мелодия), ни подъёма (``build``), ни ролла малого."""
    for _plan_, track, _program, _by in tracks:
        middle = [s for s in track.form.sections if not s.name.startswith(("intro", "outro"))]
        assert middle and all(s.bars % 4 == 0 for s in middle)
        assert not any(kn.section_kind(s.name).startswith("build") for s in track.form.sections)
        for a, b in zip(middle, middle[1:]):
            assert a.roles != b.roles, (a.name, b.name)
    assert LOFI.roll_bars == 0


def test_drums_are_pack_files_and_their_events_are_the_model(tracks):
    for _plan_, track, program, by_role in tracks:
        kick = track.parts["kick"]
        assert kick.synth_or_sample in {kn.KICK_SOUNDS[k].pack for k in LOFI.kick_pool}
        assert track.parts["hats"].synth_or_sample in LOFI.drum_files["hats"]
        files = {kn.SAMPLE_CATALOG[track.parts[r].synth_or_sample].path for r in ("kick", "hats")}
        assert files <= program.sample_files and not program.samples, "удары — файлы паков, не play()"
        assert track_composition(track)["kick"] in LOFI.kick_pool
        assert track.mix.a9_model >= LOFI.a9_model_low, track.mix.a9_model
        assert track.mix.duck_roles == {"bass"}, "мягкий насос баса (ADR-0157 H7), клавиши держат без сайдчейна"
        model = sorted(round(i / 4 + st.offset_ms * track.bpm / 60000.0, 3)
                       for i, st in enumerate(kick.grid.steps) if st.on and _sounds(track, "kick", i / 4))
        assert sorted(round(e.beat, 3) for e in by_role["kick"]) == model, "события бочки = модель со свингом"


def _sounds(track, role: str, beat: float) -> bool:
    start = 0
    for sec in track.form.sections:
        if start * 4 <= beat < (start + sec.bars) * 4:
            return role in sec.roles
        start += sec.bars
    return False


def test_haas_voice_and_note_swing_cannot_share_the_delay_key(tracks):
    _plan_, track, _program, _by = tracks[0]
    stereo = {**track.mix.stereo, "lead": Stereo(pan=1.0, detune=0.125, haas_ms=15.0)}
    with pytest.raises(RenderError, match="Хаас"):
        render(dataclasses.replace(track, mix=dataclasses.replace(track.mix, stereo=stereo)), "A")


@pytest.mark.parametrize("text", ["лоуфай", "включи лоуфай сет", "лоу-фай про дождь", "лоу фай", "lo-fi", "lo fi",
                                  "lofi beats", "лофай", "чилл", "чиллаут сет", "chill"])
def test_lofi_words_pick_the_style(text):
    assert match_style_text(text) == "lofi"


@pytest.mark.parametrize("text", ["чили", "лофт", "логистика", "фай", "лоток"])
def test_words_close_to_lofi_words_are_not_a_style(text):
    assert match_style_text(text) is None
