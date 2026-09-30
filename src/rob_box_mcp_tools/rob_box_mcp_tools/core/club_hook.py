"""club_hook.py — узнаваемый хук RTTTL-мелодии для lead клубного трека (issue #3181).

Зачем
=====
Lead ``style="club"`` (:mod:`core.club_arranger`) — сидированный риф в
пентатонике аккорда: трек «клубный», но ничей. Живой сет 29.09 «вечеринка
любителей денди» шёл тремя безымянными треками, хотя в RTTTL-библиотеке
лежат Contra, Tetris, Zelda, Mario. Здесь из RTTTL вырезается хук и
раскладывается так, чтобы клубный каркас сыграл его на слоте lead ВМЕСТО
рифа (гейты матрицы, уровни и пампинг — те же).

Модуль чистый (без Renardo/ROS), детерминированный.

Что такое хук здесь
===================
НАЧАЛО темы: :data:`HOOK_MAX_BARS` такта от первой звучащей ноты (ведущие
паузы отброшены), если мелодия короче — 2 такта (``CHORD_BARS``, блок
матрицы). Игровые рингтоны почти всегда начинаются с главного мотива
(Contra, Tetris, Zelda, Mario — проверено по нотам в тестах); поиск «самого
повторяющегося фрагмента» не делается — на этих темах он дал бы тот же
первый мотив, а на остальных рисковал бы вырезать аккомпанирующую фигуру.

Шаги
====
1. **Темп.** Длительности RTTTL — в долях при темпе рингтона ``b=``;
   хук играет в темпе сета. Масштаб — степень двойки, ближайшая к
   ``bpm_сета / b`` (0.5…2): Contra ``b=285`` четвертями при 124 BPM
   становится восьмыми и звучит почти в родном темпе, Tetris ``b=160``
   остаётся как есть. Если самая короткая нота после масштаба < 16-й,
   масштаб удваивается (16-я — шаг сетки, короче не сыграть).
2. **Сетка.** Онсеты квантуются в 16-е (``dur=1/4`` плеера lead — пампинг
   ``amplify`` совпадает с бочкой только при одном событии на 16-ю).
   Длинная нота — одна атака с ``sus`` на её длину, остальные её шаги —
   паузы (``None``).
3. **Тональность.** Тональность темы — :func:`core.rtttl_compose.detect_key`
   по всей мелодии.

   * Минорная тема: тоника темы → тоника сета.
   * Мажорная тема (Mario, Zelda): club поддерживает только minor
     (``SUPPORTED_SCALES``), а перевод в ОДНОИМЁННЫЙ минор меняет терции и
     сексты — мотив «грустнеет» и узнаётся хуже. Поэтому тема играется в
     РОДНОМ мажоре от III ступени сета (параллельный мажор, «relative
     major»: A# minor → C# major). Звукоряд у них общий, интервалы мотива
     не меняются ни на полутон, а аккорды берутся из того же диатонического
     набора сета — бас/пэд с хуком не конфликтуют, переходы DJ-сета между
     треками остаются в одной тональности. Цена: центр трека — мажорная
     тоника, а не минорная тоника сета; ответ тула говорит это прямо.
4. **Регистр.** Октава хука — та, где меньше всего нот вне
   ``LEAD_LOW..LEAD_TOP_LIMIT`` (ближе к середине); оставшиеся выбросы
   переносятся на октаву внутрь. Перенос отдельных нот ломает контур —
   поэтому он последний шаг, а не первый.
5. **Аккорды.** На каждый 2-тактовый блок хука — трезвучие из диатоники
   сета (плюс ``V`` и мажорная ``I``), в котором больше всего звучащего
   времени хука (сильная доля весит больше) и меньше нот в полутоне от
   звуков аккорда; при равенстве — ближе к тонике темы. Первый блок
   получает бонус тоники темы (:data:`_OPENING_TONIC_BONUS`). Прогрессия из
   банка ``PROGRESSIONS`` с хуком не используется: её аккорды к чужой
   мелодии не подобраны.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .arranger import VALID_ROOTS
from .club_arranger import CHORD_BARS, LEAD_LOW, LEAD_TOP_LIMIT, STEPS_PER_BAR, TRIAD, Chord
from .rtttl import parse_rtttl
from .rtttl_compose import detect_key

__all__ = [
    "HOOK_MAX_BARS",
    "LEAD_LOOP_BARS",
    "ClubHook",
    "HookArrangement",
    "extract_hook",
]

#: Самый длинный хук (тактов). 4 такта — фраза, по которой тему узнают,
#: и дважды укладывается в 8-тактовый цикл аккордов.
HOOK_MAX_BARS = 4

#: Длина цикла lead/баса/пэда club (4 аккорда × ``CHORD_BARS``).
LEAD_LOOP_BARS = 8

#: Масштаб длительностей RTTTL → темп сета (степени двойки).
_SCALE_RANGE = (0.5, 2.0)

#: Самая короткая звучащая нота после масштаба, долей (16-я — шаг сетки).
_MIN_NOTE_BEATS = 0.25

#: ``sus`` атаки = длина ноты × это (легато без слипания соседних атак).
_SUS_RATIO = 0.9
_SUS_MAX = 4.0
#: ``sus`` на шагах-паузах (нота не звучит, значение не слышно).
_REST_SUS = 0.15

#: Вес ноты на сильной доле (шаг 0 или 8 такта) при подборе аккорда.
_STRONG_WEIGHT = 1.5

#: Первый блок хука — на тонике темы, если она не проигрывает лучшему
#: аккорду больше этой доли звучащего времени блока: фраза открывается
#: тоникой (как у тонального центра в ``rtttl_compose``), иначе
#: единственная «блюзовая» нота на сильной доле (Contra) уводит весь хук
#: на VI ступень.
_OPENING_TONIC_BONUS = 0.25

#: Штраф ноты хука в полутоне от звука аккорда (малая секунда — «грязь»).
_CLASH_WEIGHT = 0.5

#: Трезвучия-кандидаты от тоники сета: (сдвиг от тоники, лад) → ступень.
#: Диатоника натурального минора плюс ``V`` (вводный тон гармонического
#: минора) и ``I`` (мажорная тоника — темы, где звучит большая терция
#: тоники, как у Zelda). Последние двое — в конце порядка при равенстве.
_MINOR_CHORDS: Dict[Chord, str] = {
    (0, "m"): "i", (3, "M"): "III", (5, "m"): "iv", (7, "m"): "v",
    (7, "M"): "V", (8, "M"): "VI", (10, "M"): "VII", (0, "M"): "I",
}

#: Порядок при равном счёте: от тоники темы по функциям.
_TIE_ORDER: Dict[str, Tuple[Chord, ...]] = {
    "minor": ((0, "m"), (8, "M"), (3, "M"), (10, "M"), (5, "m"), (7, "m"), (7, "M"), (0, "M")),
    # мажорная тема от III сета: I=III, IV=VI, V=VII, vi=i, ii=iv, iii=v
    "major": ((3, "M"), (8, "M"), (10, "M"), (0, "m"), (5, "m"), (7, "m"), (7, "M"), (0, "M")),
}

#: Сдвиг тоники темы от тоники сета: минор — на тонику, мажор — на III.
_MODE_TONIC_OFFSET: Dict[str, int] = {"minor": 0, "major": 3}

HookNote = Tuple[int, int, int]  # (шаг от начала хука, длина в шагах, MIDI как в RTTTL)


@dataclass(frozen=True)
class HookArrangement:
    """Хук, разложенный в тональность сета на весь цикл lead.

    ``lead`` — MIDI на каждую 16-ю цикла (``None`` — пауза/тянется нота),
    ``sus`` — той же длины, ``chords`` — аккорд на каждый 2-тактовый блок
    цикла, ``label`` — строка-комментарий для кода и лога.
    """

    lead: Tuple[Optional[int], ...]
    sus: Tuple[float, ...]
    chords: Tuple[Chord, ...]
    transpose: int
    romans: str
    label: str


@dataclass(frozen=True)
class ClubHook:
    """Хук мелодии на сетке 16-х в ИСХОДНОЙ высоте (тональность — :meth:`arrange`)."""

    melody_id: str
    title: str
    notes: Tuple[HookNote, ...]
    bars: int
    key_root: int
    key_mode: str
    time_scale: float
    #: Смещение фрагмента от первой звучащей ноты, 16-х (issue #3225); 0 — начало темы.
    offset: int = 0

    @property
    def key_name(self) -> str:
        return f"{VALID_ROOTS[self.key_root]} {self.key_mode}"

    def arrange(self, tonic_pc: int) -> HookArrangement:
        """Разложить хук в тональность сета с тоникой ``tonic_pc`` (0 = C)."""
        own_tonic = (tonic_pc + _MODE_TONIC_OFFSET[self.key_mode]) % 12
        transpose = _fit_octave([m for _s, _l, m in self.notes], (own_tonic - self.key_root) % 12)
        placed = [(s, ln, _fold(m + transpose)) for s, ln, m in self.notes]
        block_chords = _fit_chords(placed, tonic_pc, self.bars, self.key_mode)
        repeats = LEAD_LOOP_BARS // self.bars
        lead, sus = _step_grid(placed, self.bars)
        romans = "-".join(_MINOR_CHORDS[c] for c in block_chords)
        label = (
            f"хук {self.melody_id} «{self.title}», {self.bars} такта, тема в {self.key_name}"
            f" → {_target_name(tonic_pc, self.key_mode)}, сдвиг {transpose:+d}, аккорды {romans}"
        )
        return HookArrangement(
            lead=tuple(lead * repeats), sus=tuple(sus * repeats),
            chords=tuple(block_chords * repeats), transpose=transpose, romans=romans, label=label,
        )


def _target_name(tonic_pc: int, mode: str) -> str:
    own = (tonic_pc + _MODE_TONIC_OFFSET[mode]) % 12
    if mode == "minor":
        return f"{VALID_ROOTS[own]} minor"
    return f"{VALID_ROOTS[own]} major (параллельный мажор {VALID_ROOTS[tonic_pc]} minor)"


# ---------------------------------------------------------------------------
# Извлечение
# ---------------------------------------------------------------------------


def time_scale(set_bpm: float, rtttl_bpm: float, durations: Sequence[float]) -> float:
    """Множитель длительностей RTTTL для темпа сета (степень двойки 0.5…2)."""
    lo, hi = _SCALE_RANGE
    ratio = float(set_bpm) / float(rtttl_bpm) if rtttl_bpm > 0 else 1.0
    scale = min(hi, max(lo, 2.0 ** round(math.log2(ratio))))
    shortest = min(durations) if durations else _MIN_NOTE_BEATS
    while scale < hi and shortest * scale < _MIN_NOTE_BEATS - 1e-9:
        scale *= 2.0
    return scale


def quantize(notes: Sequence[Tuple[Optional[int], float]], scale: float) -> List[HookNote]:
    """Ноты RTTTL → ``(шаг, длина, MIDI)`` на сетке 16-х от первой звучащей ноты.

    Две атаки в один шаг — остаётся первая; длина ноты — до её конца или до
    следующей атаки (что раньше), минимум один шаг.
    """
    timed: List[Tuple[int, int, int]] = []
    cursor = 0.0
    start: Optional[float] = None
    for midi, beats in notes:
        if midi is not None and start is None:
            start = cursor
        if midi is not None:
            onset = round((cursor - start) * scale * 4)
            end = round((cursor - start + beats) * scale * 4)
            if not timed or onset > timed[-1][0]:
                timed.append((onset, end, midi))
        cursor += beats
    out: List[HookNote] = []
    for i, (onset, end, midi) in enumerate(timed):
        nxt = timed[i + 1][0] if i + 1 < len(timed) else end
        out.append((onset, max(1, min(end, nxt) - onset), midi))
    return out


def hook_bars(notes: Sequence[HookNote]) -> int:
    """Длина хука в тактах: :data:`HOOK_MAX_BARS`, если тема не короче, иначе блок."""
    if not notes:
        return CHORD_BARS
    total = notes[-1][0] + notes[-1][1]
    return HOOK_MAX_BARS if total >= HOOK_MAX_BARS * STEPS_PER_BAR else CHORD_BARS


def _window(notes: Sequence[HookNote], bars: int) -> Tuple[HookNote, ...]:
    """Ноты, начинающиеся в первых ``bars`` тактах; хвост последней обрезан."""
    limit = bars * STEPS_PER_BAR
    return tuple((s, min(ln, limit - s), m) for s, ln, m in notes if s < limit)


def _shift(notes: Sequence[HookNote], offset: int) -> List[HookNote]:
    """Ноты с атакой от ``offset`` шагов, сдвинутые к нулю (звучавшие ДО окна — отброшены)."""
    return [(s - offset, ln, m) for s, ln, m in notes if s >= offset]


def extract_hook(
    rtttl: str, bpm: float, melody_id: str = "", title: str = "", *, offset: int = 0, bars: Optional[int] = None,
) -> ClubHook:
    """RTTTL → :class:`ClubHook` для темпа сета ``bpm``.

    ``offset`` (issue #3225) — фрагмент со смещением в 16-х от первой
    звучащей ноты (кратно шагу такта/доли); ``bars`` — длина окна (``None`` —
    по правилу :func:`hook_bars`; для фрагментов ``CHORD_BARS``). При
    ``offset=0, bars=None`` поведение прежнее (#3181).

    Raises:
        ValueError: RTTTL не разбирается, в нём нет ни одной ноты или ``offset < 0``.
    """
    if offset < 0:
        raise ValueError(f"offset={offset} < 0")
    name, rtttl_bpm, notes = parse_rtttl(rtttl)
    sounding = [(m, d) for m, d in notes if m is not None]
    if not sounding:
        raise ValueError(f"В RTTTL {name!r} нет ни одной ноты — хук не из чего сделать")
    scale = time_scale(bpm, rtttl_bpm, [d for _m, d in sounding])
    grid = quantize(notes, scale)
    if offset:
        grid = _shift(grid, offset)
    bars = bars or hook_bars(grid)
    key_root, key_scale = detect_key([m for m, _d in notes], [d for _m, d in notes])
    return ClubHook(
        melody_id=melody_id or name,
        title=title or name,
        notes=_window(grid, bars),
        bars=bars,
        key_root=VALID_ROOTS.index(key_root),
        key_mode="major" if key_scale == "major" else "minor",
        time_scale=scale,
        offset=offset,
    )


# ---------------------------------------------------------------------------
# Раскладка
# ---------------------------------------------------------------------------


def _center() -> float:
    return (LEAD_LOW + LEAD_TOP_LIMIT) / 2.0


def _fit_octave(midis: Sequence[int], semis: int) -> int:
    """Сдвиг ``semis + 12·k``: меньше всего нот вне регистра lead, ближе к середине."""
    def cost(shift: int) -> Tuple[int, float]:
        moved = [m + shift for m in midis]
        outside = sum(1 for m in moved if not LEAD_LOW <= m <= LEAD_TOP_LIMIT)
        return outside, abs(sum(moved) / len(moved) - _center())

    return min((semis + 12 * k for k in range(-6, 7)), key=cost)


def _fold(midi: int) -> int:
    """Нота вне регистра lead — на октаву(ы) внутрь."""
    while midi < LEAD_LOW:
        midi += 12
    while midi > LEAD_TOP_LIMIT:
        midi -= 12
    return midi


def _chord_score(block: Sequence[HookNote], tonic_pc: int, chord: Chord) -> float:
    """Звучащее время хука в звуках аккорда минус полутоновые столкновения."""
    tones = {(tonic_pc + chord[0] + i) % 12 for i in TRIAD[chord[1]]}
    clashes = {(t + d) % 12 for t in tones for d in (1, 11)} - tones
    score = 0.0
    for step, length, midi in block:
        weight = length * (_STRONG_WEIGHT if step % (STEPS_PER_BAR // 2) == 0 else 1.0)
        if midi % 12 in tones:
            score += weight
        elif midi % 12 in clashes:
            score -= _CLASH_WEIGHT * weight
    return score


def _fit_chords(notes: Sequence[HookNote], tonic_pc: int, bars: int, mode: str) -> List[Chord]:
    """Аккорд на каждый 2-тактовый блок хука (см. модульный докстринг, шаг 5)."""
    order = _TIE_ORDER[mode]
    block_steps = CHORD_BARS * STEPS_PER_BAR
    chords: List[Chord] = []
    for start in range(0, bars * STEPS_PER_BAR, block_steps):
        block = [n for n in notes if start <= n[0] < start + block_steps]
        bonus = _OPENING_TONIC_BONUS * sum(n[1] for n in block) if start == 0 else 0.0

        def rank(chord: Chord) -> Tuple[float, int]:
            opening = bonus if chord == order[0] else 0.0
            return _chord_score(block, tonic_pc, chord) + opening, -order.index(chord)

        chords.append(max(order, key=rank))
    return chords


def _step_grid(notes: Sequence[HookNote], bars: int) -> Tuple[List[Optional[int]], List[float]]:
    """Хук → MIDI и ``sus`` на каждую 16-ю (пауза — ``None``)."""
    size = bars * STEPS_PER_BAR
    lead: List[Optional[int]] = [None] * size
    sus = [_REST_SUS] * size
    for step, length, midi in notes:
        lead[step] = midi
        sus[step] = min(_SUS_MAX, round(length * 0.25 * _SUS_RATIO, 3))
    return lead, sus
