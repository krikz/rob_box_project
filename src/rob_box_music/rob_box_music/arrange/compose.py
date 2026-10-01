"""``compose(plan, track_no, ...)`` — трек сета по плану (ADR-0149 §3.3, §3.4, §4.4–§4.7; PR-3a, PR-3b).

Форма 48 тактов по 8: intro (бочка, хэт, пэд) → build (+клэп, бас, начало хука) → drop (хук) → break (без
бочки и баса, хук вдвое медленнее) → drop2 (хук в параллельных терциях) → outro (без лида). Хук — начало
мелодии темы из локальной RTTTL-библиотеки (``arrange.hook``); тональность трека — тоника профиля и лад
хука. Нет годной мелодии темы — лид-мотив «вопрос/ответ» (PR-2) с тем же развитием, ``track.hook = None``.
Прогрессия подбирается под хук (``harmony.fit_progression``), бас в оффбит, пэд с голосоведением.

Из плана (``set_plan``): темп сета, тоника трека (ход по квинтам), энергия трека — сдвиг энергии секций и
состав ролей (``knowledge.ENERGY_THIN_ROLES``), свинг хэтов. Секция перед дропом кончается fill-ом
(``arrange.rhythm``): ролл клэпа, бочка снята на последней доле. Клэп-бэкбит — только в дропах, в build
и break клэп звучит одним роллом fill-а; последний такт outro тоже без последней бочки (конец фразы трека).
Так рисунки ударных складываются в период 16 тактов — степень двойки, которую санитайзер v1 не трогает
(``_fix_pattern_length``; на роботе программа v2 пока идёт через ``execute_music_code``).

Не сделано здесь (PR-3c/3d): сайдчейн, mix, тембры, сэмплы и разнообразие через ``music_history``.
"""

from __future__ import annotations

import hashlib
import random
from dataclasses import replace
from typing import Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import (
    BEATS_PER_BAR, Form, Grid, Harmony, HistoryKey, Hook, Key, Mix, Part, PitchEvent, Section, Track, Transition,
)
from ..set_plan import SetPlan, seeded_plan
from ..theme import ThemeProfile
from . import bass, harmony, hook as hooks, lead, rhythm

_DRUMS = frozenset({"kick", "hats"})
_FULL = _DRUMS | {"clap", "bass", "pad", "lead"}
#: (имя, энергия 0..10, роли). Имена секций — ключи развития хука ``arrange.hook.DEVELOPMENT``.
SECTIONS: Tuple[Tuple[str, int, frozenset], ...] = (
    ("intro", 3, _DRUMS | {"pad"}),
    ("build", 5, _FULL),
    ("drop", 8, _FULL),
    ("break", 4, frozenset({"hats", "pad", "lead"})),
    ("drop2", 9, _FULL),
    ("outro", 3, _DRUMS | {"bass", "pad"}),
)
SECTION_BARS = 8
CHORD_BARS = 2
#: Уровни ролей, дБ пика (≤ потолков ``knowledge.LEVEL_CEILINGS``; сумма drop ≈ −3.2 дБ).
LEVELS_DB: Dict[str, float] = {"kick": -6.0, "hats": -16.0, "clap": -13.0, "bass": -10.0, "pad": -18.0, "lead": -14.0}
PAN: Dict[str, float] = {"hats": 0.25, "clap": -0.2}
SYNTHS: Dict[str, str] = {"bass": "bass", "pad": "sinepad", "lead": "pluck"}
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
    for i, (name, base, roles) in enumerate(SECTIONS):
        roles = (roles | ({"clap"} if _before_drop(i) else set())) - thin
        fill = _before_drop(i) or i == len(SECTIONS) - 1
        out.append(Section(name, SECTION_BARS, min(10, max(0, base + energy - 3)), frozenset(roles), fill))
    return Form(tuple(out))


def _sections() -> Iterator[Tuple[int, str, frozenset]]:
    """(первый такт, имя, роли) по форме."""
    for i, (name, _energy, roles) in enumerate(SECTIONS):
        yield i * SECTION_BARS, name, roles


def _bars_with(role: str) -> Iterator[int]:
    for start, _name, roles in _sections():
        if role in roles:
            yield from range(start, start + SECTION_BARS)


def _pad(chords, register) -> Part:
    loop = len(chords) * CHORD_BARS
    events = tuple(
        PitchEvent(m, bar * BEATS_PER_BAR, CHORD_BARS * BEATS_PER_BAR - 0.5, 2)
        for bar in _bars_with("pad") if bar % CHORD_BARS == 0
        for m in chords[(bar % loop) // CHORD_BARS].voicing
    )
    grid = rhythm.grid(range(0, loop * 16, CHORD_BARS * 16), loop * 16)
    return Part("pad", SYNTHS["pad"], grid, events, LEVELS_DB["pad"], register)


def _bass(key: Key, chords) -> Part:
    loop = len(chords) * CHORD_BARS
    bars = [(bar, harmony.triad_pcs(key, chords[(bar % loop) // CHORD_BARS].degree)) for bar in _bars_with("bass")]
    events = bass.offbeat_bass(bars, kn.REGISTERS["bass"])
    return Part("bass", SYNTHS["bass"], rhythm.grid(rhythm.OFFBEAT_STEPS), events, LEVELS_DB["bass"],
                kn.REGISTERS["bass"])


def _lead(motif: Hook, key: Key) -> Part:
    """Хук (или мотив) по секциям с лидом: развитие ``arrange.hook.develop`` от начала каждой секции."""
    events: List[PitchEvent] = []
    for start, name, roles in _sections():
        if "lead" in roles:
            offset = start * BEATS_PER_BAR
            events += [PitchEvent(e.midi, offset + e.beat, e.dur_beats, e.accent)
                       for e in hooks.develop(motif, name, SECTION_BARS, key)]
    total = len(SECTIONS) * SECTION_BARS * 16
    grid = rhythm.grid({int(e.beat * 4) for e in events}, total)
    return Part("lead", SYNTHS["lead"], grid, tuple(events), LEVELS_DB["lead"], kn.REGISTERS["lead"])


def _drums(form: Form, swing_ms: int) -> Dict[str, Part]:
    """Бочка и клэп — на всю форму (fill-ы), хэты — такт со свингом. Клэп-бэкбит — в дропах; в остальных
    секциях клэп — только ролл fill-а."""
    kick_bar, clap_bar = rhythm.kick_grid(kn.GENRE_WINDOWS["club"].kick), rhythm.clap_grid()
    silent = rhythm.grid(())

    def kick(_sec: Section, fill: bool) -> Grid:
        return rhythm.kick_fill(kick_bar) if fill else kick_bar

    def clap(sec: Section, fill: bool) -> Grid:
        bar = clap_bar if sec.name.startswith("drop") else silent
        return rhythm.clap_fill(bar) if fill else bar

    grids = {"kick": rhythm.form_grid(form.sections, kick), "hats": rhythm.hats_grid(swing_ms)}
    if any("clap" in sec.roles for sec in form.sections):
        grids["clap"] = rhythm.form_grid(form.sections, clap)
    return {r: Part(r, kn.PLAY_SYNTH, g, None, LEVELS_DB[r], (0, 0)) for r, g in grids.items()}


def hook_candidates(profile: ThemeProfile, melodies: Mapping[str, str], rng: random.Random,
                    recent_hooks: Sequence[str] = ()) -> Iterator[Tuple[Hook, Key]]:
    """Годные мелодии темы в порядке сида; только что игравшая (``recent_hooks[0]``) — последней."""
    ids = [i for i in profile.hook_ids if i in melodies]
    order = rng.sample(ids, len(ids))
    order.sort(key=lambda i: bool(recent_hooks) and i == recent_hooks[0])
    for melody_id in order:
        try:
            yield hooks.from_rtttl(melodies[melody_id], melody_id, profile.bpm, profile.root, profile.mode,
                                   HOOK_REGISTER)
        except hooks.HookError:
            continue


def _arrange(motif: Hook, key: Key, rng: random.Random):
    """Лид, прогрессия под мотив и пэд под лидом; пэд не помещается под лидом — ``ValueError``."""
    lead_part = _lead(motif, key)
    drop = [e for e in hooks.develop(motif, "drop", SECTION_BARS, key) if e.beat < LOOP_BEATS]
    degrees = harmony.fit_progression(key, drop, CHORD_BARS * BEATS_PER_BAR, rng)
    pad_top = min(kn.REGISTERS["pad"][1], min(e.midi for e in lead_part.pitches) - PAD_GAP)
    pad_register = (kn.REGISTERS["pad"][0], pad_top)
    return lead_part, degrees, pad_register, harmony.pad_chords(key, degrees, pad_register)


def compose(plan: SetPlan, track_no: int, *, melodies: Optional[Mapping[str, str]] = None,
            recent_hooks: Sequence[str] = (), deck: str = "A") -> Track:
    """Трек ``track_no`` сета по плану. ``melodies`` — ``{id: rtttl}`` для ``plan.profile.hook_ids``.

    Темп — сета, тоника — ``plan.root(track_no)``, энергия — ``plan.track(track_no).energy``. Хук — первая
    мелодия темы, под которой складываются гармония и пэд; ни одной — мотив лида (PR-2).
    """
    step = plan.track(track_no)
    profile = replace(plan.profile, bpm=plan.bpm, root=plan.root(track_no))
    rng = random.Random(f"{plan.seed}:{track_no}")
    track_hook: Optional[Hook] = None
    for candidate, key in hook_candidates(profile, melodies or {}, rng, recent_hooks):
        try:
            lead_part, degrees, pad_register, chords = _arrange(candidate, key, rng)
        except ValueError:
            continue
        track_hook = candidate
        break
    if track_hook is None:
        key = Key(profile.root, profile.mode)
        motif = Hook(lead.motif(key, rng, kn.REGISTERS["lead"]), lead.MOTIF_BARS, None)
        lead_part, degrees, pad_register, chords = _arrange(motif, key, rng)
    form = _form(step.energy)
    drums = _drums(form, rhythm.swing_offset_ms(plan.swing, plan.bpm))
    parts = {**drums, "bass": _bass(key, chords), "pad": _pad(chords, pad_register), "lead": lead_part}
    prog = "-".join(str(d) for d in degrees)
    sha = hashlib.sha256(repr((plan.bpm, key, step, sorted(parts.items()), chords)).encode()).hexdigest()[:8]
    return Track(
        track_id=f"{plan.set_id}:{track_no:02d}:{deck}:{sha}", seed=plan.seed, bpm=plan.bpm, key=key, form=form,
        parts=parts, harmony=Harmony({n: chords for n, _e, _r in SECTIONS}), hook=track_hook,
        mix=Mix({r: p.level_db for r, p in parts.items()}, {r: PAN.get(r, 0.0) for r in parts}, 0.0),
        energy=step.energy, transition_in=Transition(8, 4, True), transition_out=Transition(8, 4, True),
        history_key=HistoryKey("club_v2", prog, track_hook.source if track_hook else None, None, key.root),
    )


def club_track(seed: int, *, set_id: str = "v2", deck: str = "A", track_no: int = 1) -> Track:
    """Трек без темы и без RTTTL-библиотеки (мотив лида) — для проверок плеера (PR-4) одним сидом."""
    rng = random.Random(seed)
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    profile = ThemeProfile("", "club", rng.randint(lo, hi), rng.randrange(12), "minor", (), None)
    return compose(seeded_plan(profile, seed, set_id=set_id), track_no, deck=deck)


__all__ = ["HOOK_REGISTER", "LEVELS_DB", "SECTIONS", "club_track", "compose", "hook_candidates"]
