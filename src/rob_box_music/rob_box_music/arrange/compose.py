"""``compose(plan, track_no, ...)`` — трек сета по плану (ADR-0149 §3.3, §3.4, §4.4–§4.7; PR-3a, PR-3b).

Форма 48 тактов: intro (хэт, пэд; 4 такта) → intro_low (+бочка, бас; 4) → build (+клэп, начало хука) → drop (хук)
→ break (без бочки и баса, хук вдвое медленнее) → drop2 (хук в параллельных терциях) → outro (без лида; 4) →
outro_tail (хэт, пэд; 4). Интро и аутро поделены под блэнд двух дек (PR-8, ``model.blend_bars``): хвост уходящего
и начало входящего звучат вместе 8 тактов, бочка и бас меняются на такте свопа. Хук — начало
мелодии темы из локальной RTTTL-библиотеки (``arrange.hook``); тональность трека — тоника профиля и лад
хука. Нет годной мелодии темы — лид-мотив «вопрос/ответ» (PR-2) с тем же развитием, ``track.hook = None``.
Прогрессия подбирается под хук (``harmony.fit_progression``), бас в оффбит, пэд с голосоведением.

Из плана (``set_plan``): темп сета, тоника трека (ход по квинтам), энергия трека — сдвиг энергии секций и
состав ролей (``knowledge.ENERGY_THIN_ROLES``), свинг хэтов. Секция перед дропом кончается fill-ом
(``arrange.rhythm``): ролл клэпа, бочка снята на последней доле. Клэп-бэкбит — только в дропах, в build
и break клэп звучит одним роллом fill-а.
Так рисунки ударных складываются в период 16 тактов — степень двойки, которую санитайзер v1 не трогает
(``_fix_pattern_length``; на роботе программа v2 пока идёт через ``execute_music_code``).

Микс (PR-3c, ``arrange.mix``): уровни ролей по модели громкости, тембры по семье темы и сиду трека, бочка жанра
с настоящим низом (``knowledge.KICK_SOUNDS``), сайдчейн-огибающая от рисунка бочки на басе и пэде. Пэд поэтому
звучит аккордом на каждой 16-й (``sus`` — шаг): огибающая ``amplify`` живёт только на событиях.

Разнообразие (PR-3d, ADR-0149 I17, A12, A13): ``history`` — строки ``music_history`` (свежие первыми). Каркас
ударных (``knowledge.DRUM_KITS``) не повторяет прошлый трек; прогрессия — не больше 3 раз за 10 треков; хук-фрагмент
(отпечаток без транспозиции) не повторяется подряд; сэмплы DJ_Dave — слой ``sample`` и ``fx`` по роли каталога
(``arrange.samples``). Выбор — ``diversity.weighted_pick`` со своим ГСЧ сида на каждую ось: сид меняет материал,
темп сета — нет. Что писать в историю — ``diversity.track_history(track)``.
"""

from __future__ import annotations

import hashlib
import random
from dataclasses import replace
from typing import Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import (
    BEATS_PER_BAR, Form, Grid, Harmony, HistoryKey, Hook, Key, Part, PitchEvent, Section, Track, Transition,
)
from ..diversity import fingerprint, recent_values, weighted_pick
from ..set_plan import SetPlan, seeded_plan
from ..theme import ThemeProfile
from . import bass, harmony, hook as hooks, lead, mix, rhythm, samples

_DRUMS = frozenset({"kick", "hats"})
_FULL = _DRUMS | {"clap", "bass", "pad", "lead"}
#: Переход трека (ADR-0149 §3.12, PR-8): блэнд 8 тактов, своп баса и бочки на 4-м — у входа и у выхода один.
TRANSITION = Transition(8, 4, True)
_SWAP = TRANSITION.bass_swap_bar
_TAIL = TRANSITION.phrase_bars - _SWAP
#: (имя, такты, энергия 0..10, роли). Имена секций — ключи развития хука ``arrange.hook.DEVELOPMENT``.
#: Интро и аутро поделены под блэнд (``model.blend_bars``): входящий трек начинает хэтами и пэдом под хвостом
#: уходящего (хэты + пэд, без лида), бочка и бас входят ровно на такте свопа — там, где их снимает уходящий.
#: Энергия секции — ось вида (``knowledge.LOOKS``, PR-7): интро/аутро (блэнд) при любой энергии трека ≤ 4 — ровная
#: прямая бочка, build 4..7, дроп ≥ 7 — полный «насос».
SECTIONS: Tuple[Tuple[str, int, int, frozenset], ...] = (
    ("intro", _SWAP, 2, frozenset({"hats", "pad"})),
    ("intro_low", TRANSITION.phrase_bars - _SWAP, 2, _DRUMS | {"bass", "pad"}),
    ("build", 8, 5, _FULL),
    ("drop", 8, 8, _FULL),
    ("break", 8, 4, frozenset({"hats", "pad", "lead"})),
    ("drop2", 8, 9, _FULL),
    ("outro", 8 - _TAIL, 2, _DRUMS | {"bass", "pad"}),
    ("outro_tail", _TAIL, 1, frozenset({"hats", "pad"})),
)
#: Длина секций с лидом (развитие хука считается от начала каждой).
SECTION_BARS = 8
CHORD_BARS = 2
#: Уровень партии до ``mix.mix_parts`` (он ставит уровень роли из ``knowledge.ROLE_LEVEL_DB``).
_UNLEVELED = 0.0
STEP_BEATS = BEATS_PER_BAR / 16
PAD_GAP = 3  # верх пэда ниже лида на ≥ 3 полутона (ADR-0149 §3.6)
LOOP_BEATS = CHORD_BARS * BEATS_PER_BAR * 4  # петля прогрессии — 4 аккорда
#: Коридор хука: низ лида ≥ низ пэда + 9 (любое обращение трезвучия) + ``PAD_GAP`` — пэду есть где звучать.
HOOK_REGISTER = (kn.REGISTERS["pad"][0] + 9 + PAD_GAP, kn.REGISTERS["lead"][1])


def _before_drop(i: int) -> bool:
    return i + 1 < len(SECTIONS) and SECTIONS[i + 1][0].startswith("drop")


def _form(energy: int) -> Form:
    """Секции трека энергии ``energy``: энергия секций сдвинута от средней (3), тонкие роли сняты. Fill — перед
    дропом (клэп-ролл, если энергия не сняла клэп) и в конце трека."""
    thin = frozenset(kn.ENERGY_THIN_ROLES.get(energy, ()))
    out = []
    for i, (name, bars, base, roles) in enumerate(SECTIONS):
        layers = {role for role, names in samples.SECTIONS.items() if name in names}
        roles = (roles | layers | ({"clap"} if _before_drop(i) else set())) - thin
        fill = _before_drop(i) or i == len(SECTIONS) - 1
        out.append(Section(name, bars, min(10, max(0, base + energy - 3)), frozenset(roles), fill))
    return Form(tuple(out))


def _sections() -> Iterator[Tuple[int, int, str, frozenset]]:
    """(первый такт, такты, имя, роли) по форме."""
    start = 0
    for name, bars, _energy, roles in SECTIONS:
        yield start, bars, name, roles
        start += bars


def _bars_with(role: str) -> Iterator[int]:
    for start, bars, _name, roles in _sections():
        if role in roles:
            yield from range(start, start + bars)


def _pad(chords, register, synth: str) -> Part:
    """Аккорд на каждой 16-й (``sus`` — шаг): сайдчейн-огибающая рендера ложится на каждое событие."""
    loop = len(chords) * CHORD_BARS
    events = tuple(
        PitchEvent(m, bar * BEATS_PER_BAR + step * STEP_BEATS, STEP_BEATS, 3)
        for bar in _bars_with("pad") for step in range(16)
        for m in chords[(bar % loop) // CHORD_BARS].voicing
    )
    return Part("pad", synth, rhythm.grid(range(16)), events, _UNLEVELED, register)


def _bass(key: Key, chords, synth: str) -> Part:
    loop = len(chords) * CHORD_BARS
    bars = [(bar, harmony.triad_pcs(key, chords[(bar % loop) // CHORD_BARS].degree)) for bar in _bars_with("bass")]
    events = bass.offbeat_bass(bars, kn.REGISTERS["bass"])
    return Part("bass", synth, rhythm.grid(rhythm.OFFBEAT_STEPS), events, _UNLEVELED, kn.REGISTERS["bass"])


def _lead(motif: Hook, key: Key, synth: str) -> Part:
    """Хук (или мотив) по секциям с лидом: развитие ``arrange.hook.develop`` от начала каждой секции."""
    events: List[PitchEvent] = []
    for start, bars, name, roles in _sections():
        if "lead" in roles:
            offset = start * BEATS_PER_BAR
            events += [PitchEvent(e.midi, offset + e.beat, e.dur_beats, e.accent)
                       for e in hooks.develop(motif, name, bars, key)]
    total = sum(bars for _n, bars, _e, _r in SECTIONS) * 16
    grid = rhythm.grid({int(e.beat * 4) for e in events}, total)
    return Part("lead", synth, grid, tuple(events), _UNLEVELED, kn.REGISTERS["lead"])


def _drums(form: Form, swing_ms: int, kit: str) -> Dict[str, Part]:
    """Бочка и клэп — на всю форму (fill-ы), хэты каркаса ``kit`` — такт со свингом. Бочка секции — рисунок её вида
    (``mix.look``: build ↔ drop). Клэп-бэкбит — в дропах; в остальных секциях клэп — только ролл fill-а.
    Бочка — сэмпл жанра с настоящим низом (``mix.kick_sound``)."""
    clap_bar = rhythm.clap_grid()
    silent = rhythm.grid(())

    def kick(sec: Section, fill: bool) -> Grid:
        bar = rhythm.kick_grid(mix.look(sec.energy).kick)
        return rhythm.kick_fill(bar) if fill else bar

    def clap(sec: Section, fill: bool) -> Grid:
        bar = clap_bar if sec.name.startswith("drop") else silent
        return rhythm.clap_fill(bar) if fill else bar

    grids = {"kick": rhythm.form_grid(form.sections, kick), "hats": rhythm.hats_grid(swing_ms, kit)}
    if any("clap" in sec.roles for sec in form.sections):
        grids["clap"] = rhythm.form_grid(form.sections, clap)
    kick = mix.kick_sound("club")
    return {r: Part(r, kn.PLAY_SYNTH, g, None, _UNLEVELED, (0, 0), kick.sample if r == "kick" else 0)
            for r, g in grids.items()}


def hook_candidates(profile: ThemeProfile, melodies: Mapping[str, str], rng: random.Random,
                    history: Sequence[Mapping] = ()) -> Iterator[Tuple[Hook, Key]]:
    """Годные мелодии темы в порядке сида; хук прошлого трека (мелодия или фрагмент) подряд не повторяется."""
    last = history[0] if history else {}
    ids = [i for i in profile.hook_ids if i in melodies and i != last.get("melody_name")]
    for melody_id in rng.sample(ids, len(ids)):
        try:
            hook, key = hooks.from_rtttl(melodies[melody_id], melody_id, profile.bpm, profile.root, profile.mode,
                                         HOOK_REGISTER)
        except hooks.HookError:
            continue
        if fingerprint(hook.notes) != last.get("hook_fingerprint"):
            yield hook, key


def _motif(key: Key, rng: random.Random, last_fp: Optional[str]) -> Hook:
    """Мотив лида (PR-2), не совпадающий фрагментом с хуком прошлого трека."""
    for _ in range(8):
        motif = Hook(lead.motif(key, rng, kn.REGISTERS["lead"]), lead.MOTIF_BARS, None)
        if fingerprint(motif.notes) != last_fp:
            break
    return motif


def _arrange(motif: Hook, key: Key, rng: random.Random, lead_synth: str,
             history: Sequence[Mapping] = ()):
    """Лид, прогрессия под мотив и пэд под лидом; пэд не помещается под лидом — ``ValueError``."""
    lead_part = _lead(motif, key, lead_synth)
    drop = [e for e in hooks.develop(motif, "drop", SECTION_BARS, key) if e.beat < LOOP_BEATS]
    recent = recent_values(history, "progression")
    degrees = harmony.fit_progression(key, drop, CHORD_BARS * BEATS_PER_BAR, rng, recent)
    pad_top = min(kn.REGISTERS["pad"][1], min(e.midi for e in lead_part.pitches) - PAD_GAP)
    pad_register = (kn.REGISTERS["pad"][0], pad_top)
    return lead_part, degrees, pad_register, harmony.pad_chords(key, degrees, pad_register)


def _kit(history: Sequence[Mapping], rng: random.Random) -> str:
    """Каркас ударных: не прошлого трека, со штрафом за недавние."""
    recent = recent_values(history, "kit")
    options = [k for k in kn.DRUM_KITS if not recent or k != recent[0]]
    return weighted_pick(options, recent, rng)


def compose(plan: SetPlan, track_no: int, *, melodies: Optional[Mapping[str, str]] = None,
            history: Sequence[Mapping] = (), deck: str = "A") -> Track:
    """Трек ``track_no`` сета по плану. ``melodies`` — ``{id: rtttl}`` для ``plan.profile.hook_ids``; ``history`` —
    строки ``music_history`` (свежие первыми, ``MusicHistory.recent``).

    Темп — сета, тоника — ``plan.root(track_no)``, энергия — ``plan.track(track_no).energy``. Хук — первая
    мелодия темы, под которой складываются гармония и пэд; ни одной — мотив лида (PR-2).
    """
    step = plan.track(track_no)
    profile = replace(plan.profile, bpm=plan.bpm, root=plan.root(track_no))
    rng = random.Random(f"{plan.seed}:{track_no}")
    synths = mix.timbres(profile.row, random.Random(f"timbre:{plan.seed}:{track_no}"))
    track_hook: Optional[Hook] = None
    for candidate, key in hook_candidates(profile, melodies or {}, rng, history):
        try:
            lead_part, degrees, pad_register, chords = _arrange(candidate, key, rng, synths["lead"], history)
        except ValueError:
            continue
        track_hook = motif = candidate
        break
    if track_hook is None:
        key = Key(profile.root, profile.mode)
        motif = _motif(key, rng, history[0].get("hook_fingerprint") if history else None)
        lead_part, degrees, pad_register, chords = _arrange(motif, key, rng, synths["lead"], history)
    form = _form(step.energy)
    axis = {name: random.Random(f"{plan.seed}:{track_no}:{name}") for name in ("kit", "sample", "loop", "fx")}
    kit = _kit(history, axis["kit"])
    perc = samples.perc_pool(key, history, axis["sample"])
    loop = samples.pick(samples.LOOP_ROLES, key, history, "sample", axis["loop"])
    fx = samples.pick(samples.FX_ROLES, key, history, "fx", axis["fx"])
    drums = _drums(form, rhythm.swing_offset_ms(plan.swing, plan.bpm), kit)
    parts, track_mix = mix.mix_parts(
        {**drums, "bass": _bass(key, chords, synths["bass"]), "pad": _pad(chords, pad_register, synths["pad"]),
         "lead": lead_part, "sample": samples.perc_part(perc, kit, axis["sample"]),
         "loop": samples.loop_part(loop), "fx": samples.fx_part(fx, SECTION_BARS)}, form)
    prog = harmony.progression_name(degrees)
    sha = hashlib.sha256(repr((plan.bpm, key, step, sorted(parts.items()), chords)).encode()).hexdigest()[:8]
    return Track(
        track_id=f"{plan.set_id}:{track_no:02d}:{deck}:{sha}", seed=plan.seed, bpm=plan.bpm, key=key, form=form,
        parts=parts, harmony=Harmony({n: chords for n, _b, _e, _r in SECTIONS}), hook=track_hook,
        mix=track_mix,
        energy=step.energy, transition_in=TRANSITION, transition_out=TRANSITION,
        history_key=HistoryKey(kit, prog, track_hook.source if track_hook else None, loop, key.root,
                               fingerprint(motif.notes), fx, ",".join(perc)),
    )


def club_track(seed: int, *, set_id: str = "v2", deck: str = "A", track_no: int = 1) -> Track:
    """Трек без темы и без RTTTL-библиотеки (мотив лида) — для проверок плеера (PR-4) одним сидом."""
    rng = random.Random(seed)
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    profile = ThemeProfile("", "club", rng.randint(lo, hi), rng.randrange(12), "minor", (), None)
    return compose(seeded_plan(profile, seed, set_id=set_id), track_no, deck=deck)


__all__ = ["HOOK_REGISTER", "SECTIONS", "SECTION_BARS", "TRANSITION", "club_track", "compose", "hook_candidates"]
