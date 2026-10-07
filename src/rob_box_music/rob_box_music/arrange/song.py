"""Classic-песня в модели v2: мелодия целиком по куплетам, аккомпанемент — выход ``harmonize`` (ADR-0149 §9 PR-11).

«Поставь Калинку»: хук — ВСЯ мелодия, куплет = один её проход, куплеты отличаются составом
(``knowledge.SONG_VERSES``: тема поверх пэда и баса → входят ударные → полный состав). Темп и тональность — из
мелодии (RTTTL), а не из club-окна. Ноты аккомпанемента здесь не сочиняются: бас, пэд и рисунки ударных — это
:class:`SongMaterial`, который собирает ``rob_box_mcp_tools.engine.classic`` из ``core.harmonize`` (библиотека с
10+ вшитыми фиксами, не переписывается; пакет без ROS её не импортирует). Здесь — только раскладка по форме,
тембры и уровни (``arrange.mix``).

Песня из материала партитуры (ADR-0154 PR-7B, :func:`score_song`): «сыграй <произведение>», когда произведение есть в
индексе партитур, — мелодия автора один раз, куплеты = секции материала (:func:`score_verses`), аккомпанемент — аккорды
автора арпеджио (фактура ``arp``, ``pad.arp_events``) и басовый голос автора, а не ``core/harmonize``.
"""

from __future__ import annotations

import hashlib
import random
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..material import MeterMap, Phrase, ScoreMaterial, ScoreSection, bar_beats, meter_map, validate_material
from ..model import (
    BEATS_PER_BAR, STEPS_PER_BAR, Form, Harmony, HistoryKey, Hook, Key, Part, PitchEvent, Section, Track, Transition,
    validate,
)
from . import harmony, hook as hooks, mix, pad, rhythm

#: Песня не стыкуется в DJ-сете; переход — только чтобы трек прошёл валидатор модели.
SONG_TRANSITION = Transition(8, 4, False)
SONG_ENERGY = 3
#: Стиль песни: бочка и уровни ролей (ADR-0153 §4.3: песенная форма стилей — второй шаг; пока стиль по умолчанию).
SONG_STYLE = kn.STYLES[kn.DEFAULT_STYLE]
#: Ноты партии ``harmonize``: ``(midi | None, доли)``; у пэда вместо midi — кортеж аккорда.
Line = Sequence[Tuple[object, float]]


@dataclass(frozen=True)
class SongMaterial:
    """Мелодия и аккомпанемент одного прохода (куплета) — абсолютные MIDI и доли, как отдаёт ``harmonize``.

    ``drums``/``hats`` — рисунки на такт (16 шагов, ``knowledge.SONG_DRUM_SYMBOLS``), пустая строка — без них.
    ``pad_sus`` — длина удара аккорда пэда в долях; ``None`` — аккорд держится всю свою длительность.
    """

    melody_id: str
    title: str
    bpm: int
    root: int
    mode: str
    lead: Line
    bass: Line
    pad: Line
    pad_sus: Optional[float]
    drums: str
    hats: str

    @property
    def beats(self) -> float:
        return float(sum(d for _n, d in self.lead))


def verse_count(theme_bars: int) -> int:
    """Проходов мелодии в песне: около ``SONG_TARGET_BARS`` тактов, не больше строк ``SONG_VERSES``."""
    most = max(kn.SONG_VERSES)
    return max(1, min(most, round(kn.SONG_TARGET_BARS / max(theme_bars, 1))))


def _events(line: Line, offset: float, sus: Optional[float] = None) -> List[PitchEvent]:
    out: List[PitchEvent] = []
    beat = 0.0
    for notes, dur in line:
        chord = notes if isinstance(notes, tuple) else (notes,)
        length = min(float(sus), float(dur)) if sus else float(dur)
        out += [PitchEvent(int(m), offset + beat, length, 2) for m in chord if m is not None]
        beat += float(dur)
    return out


def _tonal(role: str, synth: str, line: Line, form: Form, theme_beats: float,
           sus: Optional[float] = None) -> Optional[Part]:
    events: List[PitchEvent] = []
    start = 0.0
    for sec in form.sections:
        if role in sec.roles:
            events += _events(line, start, sus)
        start += theme_beats
    if not events:
        return None
    pitches = [e.midi for e in events]
    return Part(role, synth, rhythm.grid(()), tuple(events), 0.0, (min(pitches), max(pitches)))


def _drums(drums: str, hats: str) -> Dict[str, Part]:
    """Рисунки ``harmonize`` → сетки ролей (такт); бочка — сэмпл стиля с настоящим низом (``mix.kick_sound``)."""
    steps: Dict[str, List[int]] = {}
    for pattern in (drums, hats):
        if pattern and len(pattern) != STEPS_PER_BAR:
            raise ValueError(f"рисунок ударных {pattern!r} не 16 шагов")
        for i, symbol in enumerate(pattern):
            role = kn.SONG_DRUM_SYMBOLS.get(symbol)
            if role is not None:
                steps.setdefault(role, []).append(i)
    kick = mix.kick_sound(SONG_STYLE)
    return {role: Part(role, kn.PLAY_SYNTH, rhythm.grid(on, accents={0: 3}), None, 0.0, (0, 0),
                       kick.sample if role == "kick" else 0)
            for role, on in steps.items()}


def _form(theme_bars: int, roles: frozenset) -> Form:
    plan = kn.SONG_VERSES[verse_count(theme_bars)]
    return Form(tuple(Section(name, theme_bars, energy, frozenset(sec & roles), False)
                      for name, energy, sec in plan), kind=kn.FORM_SONG)


def song_track(material: SongMaterial, *, seed: int, deck: str = "A") -> Track:
    """Трек-песня по материалу ``harmonize``; ``ValueError``/``TrackError`` — материал не ложится в модель."""
    beats = material.beats
    theme_bars = int(round(beats / BEATS_PER_BAR))
    for name, line in (("bass", material.bass), ("pad", material.pad)):
        if abs(sum(d for _n, d in line) - beats) > 1e-6:
            raise ValueError(f"{name}: длина {sum(d for _n, d in line)} долей ≠ мелодии {beats}")
    if theme_bars * BEATS_PER_BAR != beats:
        raise ValueError(f"мелодия {beats} долей — не целое число тактов")
    synths = {role: random.Random(f"song:{seed}:{role}").choice(kn.SONG_TIMBRES[role]) for role in kn.TONAL_ROLES}
    drums = _drums(material.drums, material.hats)
    form = _form(theme_bars, frozenset(drums) | set(kn.TONAL_ROLES))
    tonal = {"lead": _tonal("lead", synths["lead"], material.lead, form, beats),
             "bass": _tonal("bass", synths["bass"], material.bass, form, beats),
             "pad": _tonal("pad", synths["pad"], material.pad, form, beats, material.pad_sus)}
    # песня без «качания» и без клубного LPF-свипа: mix_parts по форме песни (PR-7)
    parts, track_mix = mix.mix_parts(SONG_STYLE, {**drums, **{r: p for r, p in tonal.items() if p is not None}}, form)
    form = replace(form, sections=tuple(replace(s, roles=frozenset(s.roles & set(parts))) for s in form.sections))
    key = Key(material.root, material.mode)
    hook = Hook(tuple(_events(material.lead, 0.0)), theme_bars, material.melody_id)
    sha = hashlib.sha256(repr((material.melody_id, material.bpm, key, sorted(parts.items()))).encode()).hexdigest()[:8]
    track = Track(
        track_id=f"classic:{material.melody_id}:{deck}:{sha}", seed=seed, bpm=material.bpm, key=key, form=form,
        parts=parts, harmony=Harmony({}), hook=hook, mix=track_mix, energy=SONG_ENERGY,
        transition_in=SONG_TRANSITION, transition_out=SONG_TRANSITION,
        history_key=HistoryKey("classic_v2", "harmonize", material.melody_id, None, key.root),
    )
    validate(track)
    return track


# ── Песня из материала партитуры (ADR-0154 PR-7B) ──────────────────────────────────────────────────────────────

def _meter(material: ScoreMaterial) -> MeterMap:
    mm = meter_map(material.meter)
    if mm is None:
        raise ValueError(f"размер {material.meter[0]}/{material.meter[1]} в 4/4 не переводится (ADR-0154 §3.6)")
    return mm


def _piece_bars(material: ScoreMaterial, mm: MeterMap) -> int:
    end = max(e.beat + e.dur_beats for e in material.melody)
    return max(1, int(-(-end // mm.bar)))


def score_verses(material: ScoreMaterial) -> List[Tuple[str, int, int]]:
    """Куплеты песни ``(имя, первый такт, тактов)`` в тактах материала: секции партитуры по порядку (такты до первой
    — «intro»); без секций — фразы подряд, пока куплет не наберёт ``knowledge.SONG_SCORE_VERSE_BARS`` тактов (без
    фраз — по столько тактов). Покрывают пьесу от первого такта до последней ноты."""
    end = _piece_bars(material, _meter(material))
    marks = {s.bar: s.name for s in sorted(material.sections, key=lambda s: s.bar) if s.bar < end}
    if not marks:
        step = kn.SONG_SCORE_VERSE_BARS
        cuts = [p.bar for p in material.phrases if 0 < p.bar < end] or list(range(step, end, step))
        bounds = [0]
        for cut in cuts:
            if cut - bounds[-1] >= step:
                bounds.append(cut)
        marks = {b: "verse" for b in bounds}
    marks.setdefault(0, "intro")
    starts = sorted(marks)
    return [(marks[b], b, e - b) for b, e in zip(starts, starts[1:] + [end])]


def _club(material: ScoreMaterial, mm: MeterMap, events) -> List[PitchEvent]:
    return [PitchEvent(e.midi, *mm.note(e.beat, e.dur_beats), e.accent) for e in events]


def _octave_into(midi: int, register: Tuple[int, int]) -> int:
    while midi < register[0]:
        midi += 12
    while midi > register[1]:
        midi -= 12
    return midi


def _chord_spans(material: ScoreMaterial, mm: MeterMap, register: Tuple[int, int]
                 ) -> List[Tuple[float, float, Tuple[int, ...]]]:
    """Аккорды автора → ``(доля клуба, долей, обращение)`` на сетке 16-х: обращение — ближайшее к предыдущему
    (голосоведение), аккорд без тонов (``other``) — прима."""
    out: List[Tuple[float, float, Tuple[int, ...]]] = []
    step = BEATS_PER_BAR / STEPS_PER_BAR
    for c in material.chords:
        start, dur = mm.note(c.beat, c.dur_beats)
        start, beats = round(start / step) * step, round(dur / step) * step
        pcs = tuple((c.root_pc + i) % 12 for i in kn.CHORD_INTERVALS[c.quality])[:3]
        options = harmony.voicings(pcs, register)
        if beats <= 0 or not options:
            continue
        prev = out[-1][2] if out else None
        mid = sum(register) / 2

        def cost(v: Tuple[int, ...]) -> float:
            return sum(abs(a - b) for a, b in zip(v, prev)) if prev else abs(sum(v) / len(v) - mid)
        out.append((start, beats, min(options, key=lambda v: (cost(v), v))))
    return out


def _score_bass(material: ScoreMaterial, mm: MeterMap, spans, register: Tuple[int, int]) -> List[PitchEvent]:
    """Басовый голос автора в регистре баса песни; голоса нет — прима аккорда автора на его длину."""
    if material.bass:
        return [replace(e, midi=_octave_into(e.midi, register)) for e in _club(material, mm, material.bass)]
    return [PitchEvent(_octave_into(min(v), register), b, d, 2) for b, d, v in spans]


def _sliced(events: Sequence[PitchEvent], form: Form, role: str) -> List[PitchEvent]:
    """События роли только в секциях, где роль звучит; нота не переходит за конец формы."""
    out, start = [], 0.0
    total = form.bars_total * BEATS_PER_BAR
    for sec in form.sections:
        end = start + sec.bars * BEATS_PER_BAR
        if role in sec.roles:
            out += [replace(e, dur_beats=min(e.dur_beats, total - e.beat)) for e in events if start <= e.beat < end]
        start = end
    return out


def _score_form(verses: List[Tuple[str, int, int]], mm: MeterMap) -> Form:
    """Секции песни по куплетам материала; состав — строки ``knowledge.SONG_VERSES``: первый куплет — тема над
    аккомпанементом, последний — полный состав, средние — со вторым составом."""
    rows = kn.SONG_VERSES[min(len(verses), max(kn.SONG_VERSES))]
    out, total = [], 0
    for i, (name, bar, bars) in enumerate(verses):
        row = rows[0] if i == 0 else rows[-1] if i == len(verses) - 1 else rows[min(1, len(rows) - 1)]
        end = int(-(-(bar + bars) * mm.club // BEATS_PER_BAR))
        club = end - total
        if club <= 0:
            continue
        if total + club > kn.SONG_MAX_BARS:
            break
        out.append(Section(name, club, row[1], row[2], False))
        total += club
    return Form(tuple(out), kind=kn.FORM_SONG)


def from_bar(material: ScoreMaterial, bar: int) -> ScoreMaterial:
    """Материал с такта ``bar`` (такты исходного размера): ноты, аккорды, фразы и секции раньше него отброшены,
    остальное сдвинуто к началу — песня начинается с главного мотива."""
    if bar <= 0:
        return material
    shift = bar * bar_beats(material.meter)

    def notes(events):
        return tuple(replace(e, beat=e.beat - shift) for e in events if e.beat >= shift)
    return replace(material, melody=notes(material.melody), bass=notes(material.bass),
                   chords=tuple(replace(c, beat=c.beat - shift) for c in material.chords if c.beat >= shift),
                   phrases=tuple(Phrase(p.bar - bar, p.bars, p.relation, p.repeats)
                                 for p in material.phrases if p.bar >= bar),
                   sections=tuple(ScoreSection(s.name, s.bar - bar, s.bars, s.origin)
                                  for s in material.sections if s.bar >= bar))


def score_song(material: ScoreMaterial, *, seed: int, deck: str = "A", references: Sequence[str] = ()) -> Track:
    """Песня по материалу партитуры (ADR-0154 PR-7B): мелодия автора один раз (``knowledge.SONG_MAX_BARS``),
    куплеты — :func:`score_verses`, пэд — аккорды автора арпеджио (``pad.arp_events``, порядок ``Style.arp_order``
    стиля песни), бас — голос автора; темп и тональность — материала. Размер не переводится в 4/4, нет аккордов или
    трек не прошёл валидатор — ``ValueError``/``TrackError`` (вызывающий берёт RTTTL-песню).

    ``references`` — RTTTL того же произведения: узнаваемость — тот же ``hook.for_theme``, что у DJ-трека (голос
    главного мотива становится мелодией, песня — с его такта); мотива эталона в голосах нет — ``HookError``
    (тоже ``ValueError``): материал не годится."""
    validate_material(material)
    material, anchor = hooks.for_theme(material, references)
    material = from_bar(material, anchor or 0)
    mm = _meter(material)
    spans = _chord_spans(material, mm, kn.SONG_SCORE_PAD)
    if not spans:
        raise ValueError("у материала нет аккордов — аккомпанемент не из чего строить")
    form = _score_form(score_verses(material), mm)
    synths = {role: random.Random(f"song:{seed}:{role}").choice(kn.SONG_TIMBRES[role]) for role in kn.TONAL_ROLES}
    pad_events = pad.arp_events(SONG_STYLE.arp_order, spans)
    lines = {"lead": _club(material, mm, material.melody), "pad": list(pad_events),
             "bass": _score_bass(material, mm, spans, kn.SONG_REGISTERS["bass"])}
    tonal = {}
    for role, events in lines.items():
        notes = _sliced(events, form, role)
        if notes:
            pitches = [e.midi for e in notes]
            grid = rhythm.grid(range(STEPS_PER_BAR)) if role == "pad" else rhythm.grid(())
            tonal[role] = Part(role, synths[role], grid, tuple(notes), 0.0, (min(pitches), max(pitches)))
    drums = _drums(kn.SONG_SCORE_DRUMS, kn.SONG_SCORE_HATS)
    parts, track_mix = mix.mix_parts(SONG_STYLE, {**drums, **tonal}, form)
    form = replace(form, sections=tuple(replace(s, roles=frozenset(s.roles & set(parts))) for s in form.sections))
    key = material.key
    bpm = max(kn.BPM_RANGE[0], min(kn.BPM_RANGE[1], int(round((material.bpm or kn.SONG_SCORE_BPM) * mm.tempo_ratio))))
    hook = Hook(tuple(tonal["lead"].pitches), form.bars_total, material.material_id)
    sha = hashlib.sha256(repr((material.material_id, bpm, key, sorted(parts.items()))).encode()).hexdigest()[:8]
    track = Track(
        track_id=f"classic:{material.material_id}:{deck}:{sha}", seed=seed, bpm=bpm, key=key, form=form,
        parts=parts, harmony=Harmony({}), hook=hook, mix=track_mix, energy=SONG_ENERGY,
        transition_in=SONG_TRANSITION, transition_out=SONG_TRANSITION,
        history_key=HistoryKey("classic_v2", "score", material.material_id, None, key.root),
    )
    validate(track)
    return track


__all__ = ["SONG_TRANSITION", "SongMaterial", "from_bar", "score_song", "score_verses", "song_track", "verse_count"]
