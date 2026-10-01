"""``compose(profile, track_no, ...)`` — трек сета из профиля темы (ADR-0149 §3.3, §4.7; PR-3a).

Форма 48 тактов по 8: intro (бочка, хэт, пэд) → build (+клэп, бас, начало хука) → drop (хук) → break (без
бочки и баса, хук вдвое медленнее) → drop2 (хук в параллельных терциях) → outro (без лида). Хук — начало
мелодии темы из локальной RTTTL-библиотеки (``arrange.hook``); тональность трека — тоника профиля и лад
хука. Нет годной мелодии темы — лид-мотив «вопрос/ответ» (PR-2) с тем же развитием, ``track.hook = None``.
Прогрессия подбирается под хук (``harmony.fit_progression``), бас в оффбит, пэд с голосоведением.

Не сделано здесь (PR-3b/3c): ``set_plan`` (дуга энергии, ход тоники), fill-ы/акценты/свинг/сайдчейн,
сэмплы и разнообразие по всем осям через ``music_history``.
"""

from __future__ import annotations

import hashlib
import random
from typing import Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import (
    BEATS_PER_BAR, Form, Harmony, HistoryKey, Hook, Key, Mix, Part, PitchEvent, Section, Track, Transition,
)
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


def _drums() -> Dict[str, Part]:
    grids = {"kick": rhythm.kick_grid(kn.GENRE_WINDOWS["club"].kick), "hats": rhythm.hats_grid(),
             "clap": rhythm.clap_grid()}
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


def compose(profile: ThemeProfile, track_no: int, *, set_seed: int = 0, melodies: Optional[Mapping[str, str]] = None,
            recent_hooks: Sequence[str] = (), set_id: str = "v2", deck: str = "A") -> Track:
    """Трек ``track_no`` сета. ``melodies`` — ``{id: rtttl}`` для ``profile.hook_ids`` из RTTTL-библиотеки.

    Хук — первая мелодия темы, под которой складываются гармония и пэд; ни одной — мотив лида (PR-2).
    """
    rng = random.Random(f"{set_seed}:{track_no}")
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
    parts = {**_drums(), "bass": _bass(key, chords), "pad": _pad(chords, pad_register), "lead": lead_part}
    form = Form(tuple(Section(n, SECTION_BARS, e, roles) for n, e, roles in SECTIONS))
    prog = "-".join(str(d) for d in degrees)
    sha = hashlib.sha256(repr((profile.bpm, key, sorted(parts.items()), chords)).encode()).hexdigest()[:8]
    return Track(
        track_id=f"{set_id}:{track_no:02d}:{deck}:{sha}", seed=set_seed, bpm=profile.bpm, key=key, form=form,
        parts=parts, harmony=Harmony({n: chords for n, _e, _r in SECTIONS}), hook=track_hook,
        mix=Mix({r: p.level_db for r, p in parts.items()}, {r: PAN.get(r, 0.0) for r in parts}, 0.0),
        energy=3, transition_in=Transition(8, 4, True), transition_out=Transition(8, 4, True),
        history_key=HistoryKey("club_v2", prog, track_hook.source if track_hook else None, None, key.root),
    )


def club_track(seed: int, *, set_id: str = "v2", deck: str = "A") -> Track:
    """Трек без темы и без RTTTL-библиотеки (мотив лида) — для проверок плеера (PR-4) одним сидом."""
    rng = random.Random(seed)
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    profile = ThemeProfile("", "club", rng.randint(lo, hi), rng.randrange(12), "minor", (), None)
    return compose(profile, 1, set_seed=seed, set_id=set_id, deck=deck)


__all__ = ["HOOK_REGISTER", "LEVELS_DB", "SECTIONS", "club_track", "compose", "hook_candidates"]
