"""Гармония: аккорды лада и голосоведение пэда (ADR-0149 §3.6, research 4.2); пул прогрессий и размер аккорда —
из стиля (``knowledge.Style``, ADR-0153 §2.2). Есть материал партитуры — ступени из его аккордов
(:func:`from_material`, ADR-0154 §3.3); пробелы материала и мелодия без аккордов автора — Витерби по выученной
таблице переходов (``knowledge.PROGRESSION_TRANSITIONS``, :func:`viterbi`) с эмиссией «доля звучащей мелодии в тонах
аккорда» (аудит 07.10 Ф1, числа — ``knowledge.HOOK_HARMONY``). ``fit_progression`` (шаблоны стиля) — база сравнения
исследовательских скриптов M3, в треке не звучит."""

from __future__ import annotations

import bisect
import math
import random
from typing import Callable, Dict, Iterable, List, Mapping, NamedTuple, Optional, Sequence, Tuple, Union

from .. import knowledge as kn
from ..diversity import weighted_pick
from ..material import ChordSpan, Phrase, ScoreMaterial, meter_map
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, PitchEvent

#: Одна прогрессия — не больше ``PROGRESSION_CAP`` раз за ``PROGRESSION_WINDOW`` треков подряд (ADR-0149 A13).
PROGRESSION_CAP, PROGRESSION_WINDOW = 3, 10
#: Сколько скомпонованных, но не сыгранных треков может затесаться в окно (#3460, A13: 4/10 на случайной серии): N+1
#: компонуется заранее и в ``history`` попадает, а сет остановили/сменили до его старта — в записи его нет, и окно из
#: 9 прошлых записей по ``history`` покрывает меньше сыгранных треков. По одному на сет, окно из 10 — до трёх сетов.
PROGRESSION_SKIPPED = 3
#: Сколько прошлых треков ``history`` видит подбор прогрессии (и сколько хранит ``SetMemory``).
PROGRESSION_LOOKBACK = PROGRESSION_WINDOW - 1 + PROGRESSION_SKIPPED


class ChordSym(NamedTuple):
    """Аккорд гармонии до обращения (аудит 07.10 Ф2, #3530): ступень лада и качество — ключ
    ``knowledge.CHORD_INTERVALS``; обращение пэда — :func:`voice_chain` (``model.Chord``)."""

    degree: int
    quality: str


#: Аккорд в функциях гармонии: ступень (диатоническое качество) или :class:`ChordSym`.
AnyChord = Union[int, ChordSym]


def sym(key: Key, chord: AnyChord) -> ChordSym:
    """Аккорд как :class:`ChordSym`: ступень без качества — диатоническое трезвучие лада ``key`` (лад не из семи
    ступеней — качество пусто: терции лада)."""
    if isinstance(chord, ChordSym):
        return chord
    return ChordSym(chord, kn.diatonic_quality(key.mode, chord) if len(kn.SCALES[key.mode]) == 7 else "")


def qualities(key: Key, degree: int) -> Tuple[str, ...]:
    """Качества ступени ``degree`` без аккорда автора: диатоническое первым, затем ``knowledge.DEGREE_QUALITIES``
    лада (в миноре V — и мажорная)."""
    return (kn.diatonic_quality(key.mode, degree), *kn.DEGREE_QUALITIES.get(key.mode, {}).get(degree, ()))


def progression_name(degrees: Sequence[int]) -> str:
    """Имя прогрессии в ``music_history.progression``: ступени через дефис."""
    return "-".join(str(d) for d in degrees)


def chord_pcs(style: kn.Style, key: Key, degree: int, quality: Optional[str] = None) -> Tuple[int, ...]:
    """Тоны аккорда ступени ``degree`` из ``style.chord_size`` звуков: (прима, терция, квинта, …). ``quality`` —
    ключ ``knowledge.CHORD_INTERVALS`` (аккорд автора, гармонический V), ``None`` — терции лада
    (``knowledge.chord_pitch_classes``)."""
    return kn.chord_pitch_classes(key.root, key.mode, degree, quality or None, style.chord_size)


def voicings(pcs: Sequence[int], register: Tuple[int, int]) -> List[Tuple[int, ...]]:
    """Все тесные расположения (обращения) аккорда внутри регистра."""
    lo, hi = register
    out = []
    for inv in range(len(pcs)):
        order = list(pcs[inv:]) + list(pcs[:inv])
        for base in (m for m in range(lo, hi + 1) if m % 12 == order[0]):
            notes = [base]
            for pc in order[1:]:
                notes.append(notes[-1] + (pc - notes[-1]) % 12)
            if notes[-1] <= hi:
                out.append(tuple(notes))
    return out


def _movement(a: Sequence[int], b: Sequence[int]) -> int:
    return sum(abs(x - y) for x, y in zip(a, b))


def pad_chords(style: kn.Style, key: Key, degrees: Sequence[AnyChord], register: Tuple[int, int]
               ) -> Tuple[Chord, ...]:
    """Петля прогрессии → обращения с минимальным движением голосов, включая стык «последний → первый»
    (:func:`voice_chain` по кругу): цепочка «каждый от предыдущего» уплывает, и на повторе петли пэд прыгает (до 17
    полутонов у тестов PR-2)."""
    return voice_chain(style, key, degrees, register, ring=True)


def fit_progression(style: kn.Style, key: Key, notes: Sequence[PitchEvent], chord_beats: float, rng: random.Random,
                    recent: Sequence[str] = ()) -> Tuple[int, ...]:
    """Прогрессия из ``style.progressions``, аккорды которой покрывают больше всего звучания хука.

    ``notes`` — хук в долях от начала петли, аккорд держится ``chord_beats`` долей. ``recent`` — прогрессии
    прошлых треков (свежие первыми): сыгранная ``PROGRESSION_CAP`` раз за ``PROGRESSION_LOOKBACK`` не берётся, при ничьей —
    выбор сидом со штрафом за недавнее (``diversity.weighted_pick``).
    """
    def score(degrees: Tuple[int, ...]) -> float:
        triads = [set(chord_pcs(style, key, d)) for d in degrees]
        return sum(e.dur_beats for e in notes if e.midi % 12 in triads[int(e.beat // chord_beats) % len(triads)])

    window = list(recent)[:PROGRESSION_LOOKBACK]
    pool = style.progressions
    allowed = [d for d in pool if window.count(progression_name(d)) < PROGRESSION_CAP] or list(pool)
    scores = {degrees: score(degrees) for degrees in allowed}
    best = max(scores.values())
    top = {progression_name(d): d for d in allowed if scores[d] == best}
    return top[weighted_pick(list(top), window, rng)]


# ── Гармония под звучащую мелодию: Витерби (ADR-0154 §3.3, §3.4, Н6; аудит 07.10 Ф1, #3529) ───────────────────

#: Петля из ≥ 2 разных аккордов: переход «последний → первый» реже этого — последний слот становится каденцией
#: по таблице (ступень, лучше всех ведущая из предпоследнего в первый).
CADENCE_MIN_P = 0.05
#: Недиатонический аккорд материала становится диатоническим, если у них столько общих звуков (2 из 3).
COMMON_TONES_MIN = 2
_SAMPLE_BEATS = BEATS_PER_BAR / STEPS_PER_BAR  # аккорд слота — по длительности на сетке 16-х трека
_STRONG_BEATS = 2.0  # сильные доли — 1 и 3 доли такта (каждые 2 доли от начала такта)
Table = Mapping[str, tuple]


def table_mode(mode: str) -> str:
    """Лад таблиц корпуса (переходы, тоны баса): семиступенные с большой терцией — ``major``, остальные — ``minor``;
    лад не из семи ступеней (пентатоника, хроматика) — ``ValueError`` (ступени 0..6 там не определены)."""
    scale = kn.SCALES.get(mode, ())
    if len(scale) != 7:
        raise ValueError(f"лад {mode!r} не семиступенный — таблицы переходов нет")
    return "major" if scale[2] == 4 else "minor"


def transition_table(mode: str) -> Table:
    """Таблица переходов лада трека (:func:`table_mode`)."""
    return kn.PROGRESSION_TRANSITIONS[table_mode(mode)]


def _triad(key: Key, chord: AnyChord) -> set:
    """Трезвучие аккорда (ступень — диатоническое, :class:`ChordSym` — своего качества)."""
    c = sym(key, chord)
    return set(kn.chord_pitch_classes(key.root, key.mode, c.degree, c.quality))


def diminished(key: Key) -> frozenset:
    """Ступени лада ``key`` с уменьшённым трезвучием (две малые терции)."""
    scale = kn.SCALES[key.mode]
    return frozenset(d for d in range(7)
                     if (scale[(d + 2) % 7] - scale[d]) % 12 == 3 and (scale[(d + 4) % 7] - scale[(d + 2) % 7]) % 12 == 3)


def unresolved_dims(key: Key) -> frozenset:
    """Уменьшённые ступени, которых гармония под мелодию не берёт: все, кроме вводного трезвучия
    (``knowledge.DIM_ROOT``; аудит П6)."""
    return frozenset(d for d in diminished(key) if kn.SCALES[key.mode][d] != kn.DIM_ROOT)


def _weight(e: PitchEvent, hh: kn.HookHarmony) -> float:
    return e.dur_beats * (hh.strong_weight if _strong(e) else 1.0)


def _strong(e: PitchEvent) -> bool:
    return abs(e.beat % _STRONG_BEATS) < 1e-6


def _b9(pc: int, triad: set) -> bool:
    """Нота на полутон выше тона аккорда и сама не тон аккорда — малая нона/секунда."""
    return pc not in triad and (pc - 1) % 12 in triad


Pins = Sequence[Optional[ChordSym]]


def _emissions(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int,
               clash: Optional[Sequence[PitchEvent]] = None, pins: Pins = ()) -> List[List[Tuple[float, str]]]:
    """[слот][ступень] — (оценка, качество): доля мелодии слота в трезвучии аккорда минус штраф за b9 на сильных
    долях, лучшее из качеств ступени (:func:`qualities`, ничья — диатоническое; закреплённый аккорд ``pins`` — его
    качество). Вес ноты — длительность, на сильной доле ×``HOOK_HARMONY.strong_weight`` (0, если слот пуст).
    ``clash`` — все звучащие голоса для штрафа b9 (хук с терциями), по умолчанию — ``notes``."""
    options = [[(q, _triad(key, ChordSym(d, q))) for q in qualities(key, d)] for d in range(7)]
    out = []
    for slot in range(slots):
        inside = _slot_notes(notes, slot, chord_beats)
        voices = inside if clash is None else _slot_notes(clash, slot, chord_beats)
        pin = pins[slot] if slot < len(pins) else None
        out.append([_best(inside, voices, [(pin.quality, _triad(key, pin))] if pin is not None and pin.degree == d
                          else options[d]) for d in range(7)])
    return out


def _best(inside: Sequence[PitchEvent], voices: Sequence[PitchEvent],
          cands: Sequence[Tuple[str, set]]) -> Tuple[float, str]:
    """(оценка, качество) лучшего из аккордов ``cands`` (качество, трезвучие) под нотами слота; ничья — раньше."""
    hh = kn.HOOK_HARMONY
    total = sum(_weight(e, hh) for e in inside) or 1.0

    def fit(t: set) -> float:
        return (sum(_weight(e, hh) for e in inside if e.midi % 12 in t)
                - hh.b9_weight * sum(_weight(e, hh) for e in voices if _strong(e) and _b9(e.midi % 12, t))) / total
    score, _i, quality = max((fit(t), -i, q) for i, (q, t) in enumerate(cands))
    return score, quality


def _slot_notes(notes: Sequence[PitchEvent], slot: int, chord_beats: float) -> List[PitchEvent]:
    return [e for e in notes if slot * chord_beats <= e.beat < (slot + 1) * chord_beats]


def transfer_b9(key: Key, notes: Sequence[PitchEvent], chord: AnyChord, author: Iterable[int]) -> bool:
    """Малая нона на сильной доле, которую дал перенос аккорда автора в лад трека: нота ``notes`` на полутон выше тона
    трезвучия, которое сыграет трек (``chord``), а у аккорда автора (``author`` — его звуки в тональности трека) её
    нет. Аккорд автора со своим качеством её не даёт (Ф2); остаётся у недиатонического, приведённого к ступени лада.
    Неаккордовый тон самого автора (хроматика Грига) — его замысел, не перенос."""
    triad, own = _triad(key, chord), set(author)
    return any(_strong(e) and _b9(e.midi % 12, triad) and not _b9(e.midi % 12, own) for e in notes)


def strong_b9(key: Key, notes: Sequence[PitchEvent], chord: AnyChord) -> bool:
    """Есть ли нота сильной доли ``notes`` на полутон выше тона трезвучия аккорда ``chord``."""
    triad = _triad(key, chord)
    return any(_strong(e) and _b9(e.midi % 12, triad) for e in notes)


def _style_moves(progressions: Iterable[Sequence[int]]) -> frozenset:
    """Переходы (a, b) петель стиля, по кругу петли."""
    return frozenset((p[i - 1], p[i]) for p in progressions for i in range(len(p)) if p[i - 1] != p[i])


def _log_step(table: Table, a: int, b: int, moves: frozenset) -> float:
    """log P(a → b) слота: удержание — ``HOOK_HARMONY.hold_p``, смена — переход таблицы корпуса (таблица — переходы
    между разными аккордами), переход петель стиля — плюс ``style_bonus``."""
    hh = kn.HOOK_HARMONY
    if a == b:
        return math.log(hh.hold_p)
    row = table["next"][a]
    return math.log((1 - hh.hold_p) * row[b] / (1 - row[a])) + (hh.style_bonus if (a, b) in moves else 0.0)


def viterbi(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int,
            fixed: Sequence[Optional[AnyChord]] = (), table: Optional[Table] = None, *, ring: bool = False,
            degrees: Sequence[int] = range(7), progressions: Iterable[Sequence[int]] = (),
            clash: Optional[Sequence[PitchEvent]] = None) -> Tuple[int, ...]:
    """Ступени ``slots`` слотов по ``melody_weight × эмиссия + log P(переход)`` (HMM, ADR-0154 альтернатива F;
    числа — ``knowledge.HOOK_HARMONY``). Лучший путь из :func:`paths`."""
    return paths(key, notes, chord_beats, slots, fixed, table, ring=ring, degrees=degrees,
                 progressions=progressions, clash=clash)[0][1]


def paths(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int,
          fixed: Sequence[Optional[AnyChord]] = (), table: Optional[Table] = None, *, ring: bool = False,
          degrees: Sequence[int] = range(7), progressions: Iterable[Sequence[int]] = (),
          clash: Optional[Sequence[PitchEvent]] = None) -> List[Tuple[float, Tuple[int, ...]]]:
    """Лучшие пути Витерби, от лучшего: (оценка, ступени); у петли (``ring``) — по одному на каждую первую ступень,
    у цепочки — один.

    ``notes`` — мелодия в долях от начала первого слота, аккорд держится ``chord_beats`` долей. ``fixed`` —
    аккорды, известные заранее (аккорды материала: ступень или :class:`ChordSym` с качеством автора), ``None`` —
    слот выбирает Витерби; ``degrees`` — из каких ступеней выбирает (педаль — тоника/доминанта). Эмиссия ступени —
    лучшее из её качеств (:func:`qualities`). ``ring`` — петля: переход «последний → первый» входит в оценку.
    Уменьшённое трезвучие (и из ``fixed`` — по его качеству) — только перед ``knowledge.DIM_RESOLUTION``, последним в
    незамкнутой цепочке — нет; сам Витерби берёт только вводное (``knowledge.DIM_ROOT``). ``table`` — для опытов leave-one-out;
    по умолчанию таблица лада ``key``. ``progressions`` — петли стиля (бонус их переходам). ``clash`` — все
    звучащие голоса для штрафа b9 (по умолчанию ``notes``). Ничья — меньшие ступени.
    """
    table = table or transition_table(key.mode)
    w = kn.HOOK_HARMONY.melody_weight
    pins = _pins(key, fixed, slots)
    gains = [[w * score for score, _q in row] for row in _emissions(key, notes, chord_beats, slots, clash, pins)]
    allowed = _allowed(key, [None if p is None else p.degree for p in pins], degrees)
    dims = _dims(key, pins, allowed)
    logs = _log_matrix(table, progressions)
    starts = [(f,) for f in allowed[0]] if ring else [allowed[0]]  # петля — путь на каждую первую ступень
    found = (_forward(table, gains, allowed, logs, firsts, ring, dims) for firsts in starts)
    out = [sp for sp in found if sp is not None and sp[0] > -math.inf]
    if not out:
        raise ValueError(f"нет пути гармонии из ступеней {tuple(degrees)}")
    return sorted(out, key=lambda sp: (-sp[0], sp[1]))


def _pins(key: Key, fixed: Sequence[Optional[AnyChord]], slots: int) -> List[Optional[ChordSym]]:
    """Закреплённые аккорды ``slots`` слотов (:func:`sym`), ``None`` — слот свободен."""
    pins = [None if f is None else sym(key, f) for f in list(fixed)[:slots]]
    return pins + [None] * (slots - len(pins))


def _dims(key: Key, pins: Pins, allowed: Sequence[Sequence[int]]) -> List[frozenset]:
    """Ступени слота, которые звучат уменьшённым трезвучием: закреплённая — по качеству аккорда (ii автора в миноре
    без «°» — не уменьшённое), свободная — диатоническое уменьшённое лада."""
    dim = diminished(key)
    return [frozenset(d for d in opts if d in dim) if pin is None
            else frozenset({pin.degree} if pin.quality == "dim" else ()) for pin, opts in zip(pins, allowed)]


def _allowed(key: Key, pinned: Sequence[Optional[int]], degrees: Sequence[int]) -> List[Tuple[int, ...]]:
    """Ступени, из которых выбирает каждый слот: известная — она одна; иначе ``degrees`` без уменьшённых, которых
    Витерби не берёт (:func:`unresolved_dims`)."""
    banned = unresolved_dims(key)
    free = tuple(d for d in degrees if d not in banned)
    return [free if p is None else (p,) for p in pinned]


Path = Tuple[float, Tuple[int, ...]]


def _top(cands: Iterable[Path]) -> Optional[Path]:
    """Лучший путь: больше оценка; ничья — меньшие ступени."""
    best = None
    for sp in cands:
        if best is None or sp[0] > best[0] or (sp[0] == best[0] and sp[1] < best[1]):
            best = sp
    return best


def _log_matrix(table: Table, progressions: Iterable[Sequence[int]]) -> List[List[float]]:
    """log P(a → b) слота (:func:`_log_step`)."""
    moves = _style_moves(progressions)
    return [[_log_step(table, a, b, moves) for b in range(7)] for a in range(7)]


def _forward(table: Table, gains: Sequence[Sequence[float]], allowed: Sequence[Sequence[int]],
             logs: Sequence[Sequence[float]], firsts: Sequence[int], ring: bool,
             dims: Sequence[frozenset]) -> Optional[Path]:
    """Проход Витерби от первых ступеней ``firsts``: лучший путь; из уменьшённого слота (``dims``) — только в
    ``knowledge.DIM_RESOLUTION``; петля — со стыком «последний → первый», цепочка — не кончается уменьшённым."""
    def step(slot: int, a: int, b: int) -> float:
        return -math.inf if a in dims[slot] and b != kn.DIM_RESOLUTION else logs[a][b]

    best = {f: (math.log(table["start"][f]) + gains[0][f], (f,)) for f in firsts}
    for slot in range(1, len(gains)):
        best = {d: _top((score + step(slot - 1, p, d) + gains[slot][d], path + (d,))
                        for p, (score, path) in best.items()) for d in allowed[slot]}
    last = len(gains) - 1
    closing = ring and len(gains) > 1
    return _top((score + (step(last, path[-1], path[0]) if closing else 0.0), path) for score, path in best.values()
                if ring or path[-1] not in dims[last])


def qualify(key: Key, notes: Sequence[PitchEvent], chord_beats: float, degrees: Sequence[int],
            fixed: Sequence[Optional[AnyChord]] = (), clash: Optional[Sequence[PitchEvent]] = None
            ) -> Tuple[ChordSym, ...]:
    """Аккорды слотов ступеней ``degrees`` (аудит 07.10 Ф2): аккорд ``fixed`` слота (аккорд автора) на той же ступени —
    его качество; иначе качество ступени (:func:`qualities`), лучше всех покрывшее звучащую мелодию слота — та же
    эмиссия, по которой Витерби выбрал ступень."""
    rows = _emissions(key, notes, chord_beats, len(degrees), clash, _pins(key, fixed, len(degrees)))
    return tuple(ChordSym(d, rows[i][d][1]) for i, d in enumerate(degrees))


def author_chord(chord: ChordSpan, key: Key) -> Optional[ChordSym]:
    """Аккорд материала в ступенях лада ``key`` (лад материала; аудит 07.10 Ф2): прима на ступени — та же ступень и
    качество автора (``knowledge.AUTHOR_QUALITIES``: V в миноре остаётся мажорной; ``other`` — диатоническое);
    прима вне лада — аккорд ступени из :func:`qualities` (диатонический или гармонический V) с наибольшим числом общих
    звуков (≥ :data:`COMMON_TONES_MIN`; ничья — та же прима, затем аккорд с примой автора среди тонов — vii° минора
    становится мажорной V, — меньшая ступень, диатонический), иначе ``None``."""
    if chord.degree is not None:
        quality = chord.quality if chord.quality in kn.AUTHOR_QUALITIES else kn.diatonic_quality(key.mode, chord.degree)
        return ChordSym(chord.degree, quality)
    pcs = {(chord.root_pc + i) % 12 for i in kn.CHORD_INTERVALS[chord.quality]}
    scale = kn.SCALES[key.mode]
    common, _same_root, _has_root, _d, _i, best = max(
        (len(pcs & t), (key.root + scale[d]) % 12 == chord.root_pc, chord.root_pc in t, -d, -i, ChordSym(d, q))
        for d in range(7) for i, q in enumerate(qualities(key, d)) for t in [_triad(key, ChordSym(d, q))])
    return best if common >= COMMON_TONES_MIN else None


def _adapt(chord: ChordSpan, key: Key) -> Optional[int]:
    """Ступень аккорда материала (:func:`author_chord`) или ``None``."""
    c = author_chord(chord, key)
    return None if c is None else c.degree


def _chord_at(material: ScoreMaterial, starts: Sequence[float], beat: float) -> Optional[ChordSpan]:
    i = bisect.bisect_right(starts, beat) - 1
    if i < 0:
        return None
    chord = material.chords[i]
    return chord if beat < chord.beat + chord.dur_beats else None


def material_beat(material: ScoreMaterial, phrase: Phrase, scale: float = 1.0) -> Callable[[float], Optional[float]]:
    """Доля трека ``t`` → доля материала: перевод размера в 4/4 (``material.meter_map``, режим
    ``knowledge.TRIPLE_METER_MODE``) от начала такта фразы (хук материала начальную паузу не срезает, ``hook._onsets(bars=True)``: затакт
    стоит перед сильной долей, #3531) и множитель темпа ``scale`` (``hook.material_scale``). Доля в паузе перевода (4-я доля 3/4 в режиме ``pause``) — ``None``.
    Одно отображение на гармонию и бас материала."""
    mm = meter_map(material.meter)
    if mm is None:
        raise ValueError(f"размер {material.meter[0]}/{material.meter[1]} в 4/4 клуба не переводится")
    first = phrase.bar * mm.bar

    def at(t: float) -> Optional[float]:
        beat = mm.from_club(t / scale)
        return None if beat is None else first + beat
    return at


def material_slots(material: ScoreMaterial, phrase: Phrase, chord_beats: float, slots: int,
                   scale: float = 1.0) -> List[Optional[ChordSpan]]:
    """Аккорд материала каждого слота трека — самый долгий на сетке 16-х трека (ничья — раньше в слоте); доля
    трека → доля материала — :func:`material_beat`. Слот без аккорда (пауза 3/4, нет разметки) — ``None``."""
    at = material_beat(material, phrase, scale)
    starts = [c.beat for c in material.chords]
    out: List[Optional[ChordSpan]] = []
    for slot in range(slots):
        weight: Dict[ChordSpan, int] = {}
        for k in range(int(round(chord_beats / _SAMPLE_BEATS))):
            beat = at(slot * chord_beats + k * _SAMPLE_BEATS)
            chord = _chord_at(material, starts, beat) if beat is not None else None
            if chord is not None:
                weight[chord] = weight.get(chord, 0) + 1
        out.append(max(weight, key=weight.__getitem__) if weight else None)
    return out


def _cadence(degrees: List[int], table: Table) -> List[int]:
    """Последний слот петли, из которого нет хода в первый (P < :data:`CADENCE_MIN_P`), — ступень, лучше всех
    ведущая из предпоследнего в первый."""
    first, last = degrees[0], degrees[-1]
    if len(set(degrees)) < 2 or last == first or table["next"][last][first] >= CADENCE_MIN_P:
        return degrees
    prev = degrees[-2]
    best = max(range(7), key=lambda d: (table["next"][prev][d] * table["next"][d][first], -d))
    return degrees[:-1] + [best]


def from_material(material: ScoreMaterial, phrase: Phrase, key: Key, notes: Sequence[PitchEvent],
                  chord_beats: float, slots: int, scale: float = 1.0) -> Tuple[int, ...]:
    """Ступени слотов трека из аккордов фразы материала (ADR-0154 §3.3) — гармония автора, а не шаблон стиля
    (ступени :func:`material_chords`)."""
    return tuple(c.degree for c in material_chords(material, phrase, key, notes, chord_beats, slots, scale))


def material_chords(material: ScoreMaterial, phrase: Phrase, key: Key, notes: Sequence[PitchEvent],
                    chord_beats: float, slots: int, scale: float = 1.0) -> Tuple[ChordSym, ...]:
    """Аккорды слотов трека из аккордов фразы материала (ADR-0154 §3.3): ступень и качество автора (аудит 07.10 Ф2).

    Аккорд слота — :func:`material_slots`, в ступенях лада — :func:`author_chord` (качество автора сохраняется:
    перенос в тональность трека — та же ступень и то же качество). Аккорд проверяется под звучащую мелодию слота:
    малая нона на сильной доле от приведения недиатонического аккорда к ступени лада (:func:`transfer_b9`) или
    уменьшённое трезвучие без разрешения в тонику следующим аккордом автора — слот выбирает Витерби, как слоты без
    аккорда (аудит 07.10 Ф1). Слоты без аккорда (и материал без разметки
    целиком) выбирает :func:`viterbi` по ``notes`` (хук в долях трека, тональность ``key``) с известными слотами как
    опорой; фраза на одном аккорде — тоже Витерби от первого слота (Н8: петля без движения). Петля закрывается
    каденцией (:func:`_cadence`). Лад ``key`` — лад материала (``hook.from_material``), ступени не зависят от тоники.
    Лад не семиступенный — ``ValueError``.
    """
    table = transition_table(key.mode)
    if transition_table(material.key.mode) is not table:
        raise ValueError(f"лад трека {key.mode} и лад материала {material.key.mode} разные — ступени не переносятся")
    spans = material_slots(material, phrase, chord_beats, slots, scale)
    author = _author_chords(material, key, spans, notes, chord_beats)
    known = [c for c in author if c is not None]
    fixed = [known[0]] + [None] * (slots - 1) if slots > 1 and len(set(known)) == 1 else author
    degrees = list(viterbi(key, notes, chord_beats, slots, fixed, table))
    degrees = _cadence(degrees, table) if slots > 1 else degrees
    return qualify(key, notes, chord_beats, degrees, author)


def _author_chords(material: ScoreMaterial, key: Key, spans: Sequence[Optional[ChordSpan]],
                   notes: Sequence[PitchEvent], chord_beats: float) -> List[Optional[ChordSym]]:
    """Аккорды автора по слотам (:func:`author_chord`), проверенные под звучащую мелодию: слот, где приведение к
    ступени дало малую нону на сильной доле (:func:`transfer_b9`) или уменьшённое трезвучие не на вводном тоне
    (``knowledge.DIM_ROOT``) либо без разрешения следующим аккордом в ``knowledge.DIM_RESOLUTION``, — ``None``
    (выбирает Витерби)."""
    chords = [None if c is None else author_chord(c, material.key) for c in spans]
    scale = kn.SCALES[key.mode]
    shift = key.root - material.key.root
    out: List[Optional[ChordSym]] = []
    for i, c in enumerate(chords):
        nxt = chords[i + 1] if i + 1 < len(chords) else None
        if c is None or (c.quality == "dim" and (scale[c.degree] != kn.DIM_ROOT or nxt is None
                                                 or nxt.degree != kn.DIM_RESOLUTION)):
            out.append(None)
            continue
        own = {(spans[i].root_pc + shift + k) % 12 for k in kn.CHORD_INTERVALS[spans[i].quality]}
        out.append(None if transfer_b9(key, _slot_notes(notes, i, chord_beats), c, own) else c)
    return out


def melody_progression(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int, *,
                       ring: bool = False, progressions: Iterable[Sequence[int]] = (), recent: Sequence[str] = (),
                       rng: Optional[random.Random] = None) -> Tuple[int, ...]:
    """Ступени слотов под мелодию без аккордов автора (хук и тема RTTTL, мотив; ADR-0154 PR-7, аудит Ф1):
    :func:`paths` по выученной таблице лада, ``ring`` — петля хука (стык «последний → первый» в оценке).
    ``recent`` — прогрессии прошлых треков (свежие первыми): путь, сыгранный ``PROGRESSION_CAP`` раз за
    ``PROGRESSION_LOOKBACK``, не берётся, пока есть другой (A13); ничья лучших — ``diversity.weighted_pick`` сидом.
    Лад не семиступенный — ``ValueError``."""
    found = paths(key, notes, chord_beats, slots, ring=ring, progressions=progressions)
    window = list(recent)[:PROGRESSION_LOOKBACK]
    allowed = [sp for sp in found if window.count(progression_name(sp[1])) < PROGRESSION_CAP] or found
    top = {progression_name(p): p for score, p in allowed if score == allowed[0][0]}
    return top[weighted_pick(list(top), window, rng or random.Random(0))] if len(top) > 1 else allowed[0][1]


def _chain_options(style: kn.Style, key: Key, chords: Sequence[ChordSym], register: Tuple[int, int]
                   ) -> List[List[Tuple[int, ...]]]:
    """Обращения каждого аккорда в регистре; аккорд без обращения — ``ValueError``."""
    options = [voicings(chord_pcs(style, key, c.degree, c.quality), register) for c in chords]
    if not all(options):
        raise ValueError(f"прогрессия {tuple(c.degree for c in chords)} не помещается в регистр {register}")
    return options


def voice_chain(style: kn.Style, key: Key, degrees: Sequence[AnyChord], register: Tuple[int, int], *,
                ring: bool = False) -> Tuple[Chord, ...]:
    """Обращения последовательности аккордов (ступень — диатонический, :class:`ChordSym` — своего качества) с
    минимальным суммарным движением голосов (динамика по обращениям, при равенстве — ближе к середине регистра);
    ``ring`` — со стыком «последний → первый» (петля). Аккорд, у которого нет обращения в регистре, — ``ValueError``."""
    chords = [sym(key, c) for c in degrees]
    options = _chain_options(style, key, chords, register)
    mid = sum(register) / 2

    def off(v: Tuple[int, ...]) -> float:
        return abs(sum(v) / len(v) - mid)

    best = None
    for first in (options[0] if ring else [None]):
        layer = {v: ((0, off(v)), (v,)) for v in ([first] if ring else options[0])}
        for opts in options[1:]:
            layer = {v: min(((cost[0] + _movement(path[-1], v), cost[1] + off(v)), path + (v,))
                            for cost, path in layer.values()) for v in opts}
        for cost, path in layer.values():
            cand = ((cost[0] + (_movement(path[-1], path[0]) if ring else 0), cost[1]), path)
            best = cand if best is None or cand < best else best
    return tuple(Chord(c.degree, v, c.quality) for c, v in zip(chords, best[1]))


__all__ = ["AnyChord", "CADENCE_MIN_P", "COMMON_TONES_MIN", "ChordSym", "PROGRESSION_CAP", "PROGRESSION_LOOKBACK",
           "PROGRESSION_SKIPPED", "PROGRESSION_WINDOW", "author_chord", "chord_pcs", "diminished", "fit_progression",
           "from_material", "material_beat", "material_chords", "material_slots", "melody_progression", "pad_chords",
           "paths", "progression_name", "qualify", "qualities", "strong_b9", "sym", "table_mode", "transfer_b9",
           "transition_table", "unresolved_dims", "viterbi", "voice_chain", "voicings"]
