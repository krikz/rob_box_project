"""Мелодия → все партии аранжировки (бас, пэд, ударные, контрмелодия).

Зачем модуль существует
-----------------------
До него RTTTL-путь давал аранжировщику ТОЛЬКО тему (``lead_midi`` +
``lead_dur``), а бас, пэд, прогрессию и рисунок ударных придумывала LLM —
не видя ни одной ноты мелодии. Получалась не аранжировка, а «тема и рядом
несколько слоёв»: в живом прогоне 14.09 имперский марш в ре-миноре шёл
под басом на ми, потому что модель написала ``progression='1,5,6,4'``
наугад. Никакой обратной связи от мелодии к аккомпанементу не было
вообще.

Здесь она появляется: гармония ВЫВОДИТСЯ из нот темы по тактам, а бас,
пэд и контрмелодия строятся из полученных аккордов. Промахнуться мимо
тональности такой аккомпанемент не может — он буквально сделан из тех же
нот.

Что остаётся за LLM
-------------------
Тембр и характер: какими синтами играть, каким сэмплом бить, какая форма
и темп. Ноты аккомпанемента — нет: это арифметика от темы, а не вкус.

Абсолютный MIDI везде
---------------------
Все выведенные партии — абсолютные MIDI-ноты, а не ступени лада. Ступени
зависят от ``Root.default``/``Scale.default``, которые в аранжировке
меняются по форме; тема при этом фиксирована, и любой такой сдвиг
развалил бы согласие баса с мелодией. Абсолютные ноты развалить нельзя.
Как абсолютный MIDI попадает в Renardo — см. ``core.rtttl``.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import Dict, List, Optional, Sequence, Tuple

from .arranger import BEATS_PER_BAR, SCALE_INTERVALS, VALID_ROOTS

__all__ = [
    "ChordWindow",
    "Harmonization",
    "harmonize",
]

#: Сколько 16-х в такте — сетка, на которую квантуются атаки мелодии при
#: выводе рисунка ударных. 16-я — самая мелкая длительность, которая
#: реально встречается в RTTTL-рингтонах как ритмическая (32-е там почти
#: всегда «дыхательные» паузы, а не ноты).
STEPS_PER_BAR = 16

#: Октава баса и пэда в MIDI-нотах. Бас держится ниже темы, пэд — между
#: басом и темой: это то же разделение регистров, что у ролей
#: аранжировщика (ROLE_PROFILE), только в абсолютных нотах.
BASS_MIDI_FLOOR = 36   # C2
PAD_MIDI_FLOOR = 52    # E3

#: Насколько корень аккорда весомее остальных его тонов при выборе
#: гармонии. Без перевеса трезвучия с общими нотами (например i и VI в
#: миноре — две ноты из трёх общие) выбираются монеткой, и гармония
#: дёргается между ними каждый такт.
_ROOT_WEIGHT = 1.6

#: Бонус аккорду, чья тоника совпадает с ПЕРВОЙ прозвучавшей нотой такта.
#: Сильная доля слышится как опора гармонии сильнее, чем любая нота
#: в середине такта, и вес по длительности этого не передаёт.
_DOWNBEAT_BONUS = 0.9

#: Штраф за смену аккорда относительно предыдущего такта. Гармония,
#: меняющаяся каждый такт, звучит суетливо даже когда каждый отдельный
#: выбор формально верен; инерция склеивает соседние такты в фразу.
_CHANGE_PENALTY = 0.5

#: Штраф трезвучию, которое не мажорное и не минорное.
#:
#: Терции «по ладу» на некоторых ступенях дают увеличенные и уменьшённые
#: аккорды: в гармоническом миноре III — это F-A-C# (увеличенное), в любом
#: миноре II — уменьшённое. Как проходящий аккорд они уместны, но пэд
#: держит своё трезвучие ЦЕЛЫЙ такт, и неустойчивое созвучие такой длины
#: слышится не как краска, а как фальшь. Штраф не запрещает их вовсе —
#: если мелодия такта состоит ровно из этих нот, аккорд всё равно
#: победит.
_UNSTABLE_TRIAD_PENALTY = 1.1

#: Бонус тонике в ПЕРВОМ и ПОСЛЕДНЕМ такте темы. Луп смыкается сам с
#: собой, и если стык приходится не на тонику, каждый повтор слышится
#: как оборванная фраза.
_CADENCE_BONUS = 1.2


@dataclass(frozen=True)
class ChordWindow:
    """Аккорд одного окна гармонизации (такта темы).

    Attributes:
        start: начало окна в битах от начала темы.
        beats: длина окна в битах.
        degree: ступень лада, на которой построено трезвучие (0-based).
        root_midi: абсолютный MIDI корня в басовом регистре.
        tones: абсолютные MIDI тонов трезвучия в регистре пэда.
        pitch_classes: классы высоты трезвучия (0..11) — для контрмелодии.
    """

    start: float
    beats: float
    degree: int
    root_midi: int
    tones: Tuple[int, ...]
    pitch_classes: Tuple[int, ...]


@dataclass(frozen=True)
class Harmonization:
    """Тема, разложенная на партии. Всё — абсолютные MIDI и биты.

    Партии отдаются парами ``(ноты, длительности)`` одинаковой длины, как
    их принимает аранжировщик: ``None`` в нотах — пауза, в аккорде пэда
    кортеж — одновременно звучащие ноты.
    """

    bpm: int
    root: str
    scale: str
    bars: int
    chords: Tuple[ChordWindow, ...]
    lead: Tuple[Tuple[Optional[int], float], ...]
    bass: Tuple[Tuple[Optional[int], float], ...]
    pad: Tuple[Tuple[Optional[Tuple[int, ...]], float], ...]
    counter: Tuple[Tuple[Optional[int], float], ...]
    drums: str
    hats: str


# ---------------------------------------------------------------------------
# Разбор темы во времени
# ---------------------------------------------------------------------------


def _timed(
    notes: Sequence[Tuple[Optional[int], float]],
) -> Tuple[List[Tuple[float, Optional[int], float]], float]:
    """``[(midi, дл.)]`` → ``[(начало, midi, дл.)]`` + общая длина в битах."""
    out: List[Tuple[float, Optional[int], float]] = []
    cursor = 0.0
    for midi, dur in notes:
        out.append((cursor, midi, float(dur)))
        cursor += float(dur)
    return out, cursor


def _pitch_weights(
    timed: Sequence[Tuple[float, Optional[int], float]],
    start: float,
    end: float,
) -> Dict[int, float]:
    """Вес каждого класса высоты в окне ``[start, end)``.

    Вес — сколько битов нота реально звучит ВНУТРИ окна. Нота, начавшаяся
    раньше и тянущаяся в окно, считается по пересечению: именно она
    держит гармонию такта, хотя её атака осталась в прошлом такте.
    """
    weights: Dict[int, float] = {}
    for onset, midi, dur in timed:
        if midi is None:
            continue
        overlap = min(onset + dur, end) - max(onset, start)
        if overlap > 0:
            pc = midi % 12
            weights[pc] = weights.get(pc, 0.0) + overlap
    return weights


def _first_note_in(
    timed: Sequence[Tuple[float, Optional[int], float]],
    start: float,
    end: float,
) -> Optional[int]:
    """Первая нота, АТАКА которой попадает в окно (``None`` — таких нет)."""
    for onset, midi, _dur in timed:
        if midi is not None and start <= onset < end:
            return midi
    return None


# ---------------------------------------------------------------------------
# Гармония
# ---------------------------------------------------------------------------


def _triad_pitch_classes(
    degree: int, root_semitone: int, intervals: Sequence[int]
) -> Tuple[int, ...]:
    """Трезвучие на ступени лада → классы высоты (0..11).

    Терции берутся ПО ЛАДУ (ступени d, d+2, d+4), а не по полутонам —
    поэтому на каждой ступени получается свой натуральный вид аккорда
    (в миноре: i минорный, III мажорный, V минорный и так далее), и
    ни одна нота трезвучия не выпадает из лада мелодии.
    """
    size = len(intervals)
    out: List[int] = []
    for step in (0, 2, 4):
        index = degree + step
        octave, wrapped = divmod(index, size)
        out.append((root_semitone + intervals[wrapped] + 12 * octave) % 12)
    return tuple(out)


def _is_stable_triad(pitch_classes: Sequence[int]) -> bool:
    """Мажорное или минорное трезвучие? (увеличенное/уменьшённое — нет)."""
    third = (pitch_classes[1] - pitch_classes[0]) % 12
    fifth = (pitch_classes[2] - pitch_classes[0]) % 12
    return fifth == 7 and third in (3, 4)


def _pick_chords(
    timed: Sequence[Tuple[float, Optional[int], float]],
    total_beats: float,
    root: str,
    scale: str,
) -> Tuple[ChordWindow, ...]:
    """Выбрать по аккорду на каждый такт темы.

    Кандидаты — трезвучия на всех ступенях лада. Скор такта складывается
    из длительностей его нот, попавших в трезвучие (корень весомее),
    бонуса за совпадение с первой нотой такта, бонуса тонике на стыке
    лупа и штрафа за смену аккорда. Побеждает максимум.
    """
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    root_semitone = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    bars = max(1, int(round(total_beats / BEATS_PER_BAR)))

    chords: List[ChordWindow] = []
    previous: Optional[int] = None
    for bar in range(bars):
        start = bar * BEATS_PER_BAR
        end = start + BEATS_PER_BAR
        weights = _pitch_weights(timed, start, end)
        downbeat = _first_note_in(timed, start, end)
        is_edge = bar == 0 or bar == bars - 1

        best_degree = 0
        best_score = float("-inf")
        for degree in range(len(intervals)):
            pcs = _triad_pitch_classes(degree, root_semitone, intervals)
            score = sum(weights.get(pc, 0.0) for pc in pcs)
            score += (_ROOT_WEIGHT - 1.0) * weights.get(pcs[0], 0.0)
            if downbeat is not None and downbeat % 12 == pcs[0]:
                score += _DOWNBEAT_BONUS
            if is_edge and degree == 0:
                score += _CADENCE_BONUS
            if not _is_stable_triad(pcs):
                score -= _UNSTABLE_TRIAD_PENALTY
            if previous is not None and degree != previous:
                score -= _CHANGE_PENALTY
            if score > best_score:
                best_score = score
                best_degree = degree

        pcs = _triad_pitch_classes(best_degree, root_semitone, intervals)
        chords.append(
            ChordWindow(
                start=float(start),
                beats=float(BEATS_PER_BAR),
                degree=best_degree,
                root_midi=_lift(pcs[0], BASS_MIDI_FLOOR),
                tones=_stack_chord(pcs, PAD_MIDI_FLOOR),
                pitch_classes=pcs,
            )
        )
        previous = best_degree
    return tuple(chords)


def _lift(pitch_class: int, floor_midi: int) -> int:
    """Класс высоты → ближайшая MIDI-нота не ниже ``floor_midi``."""
    note = floor_midi + ((pitch_class - floor_midi) % 12)
    return note


def _stack_chord(pitch_classes: Sequence[int], floor_midi: int) -> Tuple[int, ...]:
    """Классы высоты → трезвучие, сложенное ВВЕРХ от ``floor_midi``.

    Складываем именно вверх (каждая следующая нота выше предыдущей), а не
    берём три ближайшие ноты по отдельности: иначе трезвучие вывернется в
    случайное обращение и пэд будет прыгать регистром от такта к такту.
    """
    out: List[int] = []
    current = floor_midi
    for pc in pitch_classes:
        note = current + ((pc - current) % 12)
        out.append(note)
        current = note + 1
    return tuple(out)


# ---------------------------------------------------------------------------
# Партии
# ---------------------------------------------------------------------------


def _build_bass(
    chords: Sequence[ChordWindow], dense: bool
) -> Tuple[Tuple[Optional[int], float], ...]:
    """Бас: корень аккорда на сильных долях, квинта — на слабых.

    ``dense`` (плотная тема, атака почти на каждой доле — марш, чиптюн)
    даёт бас четвертями: ровный шаг держит такую тему лучше, чем половины,
    под которыми она рассыпается. Разреженная тема получает половины,
    чтобы бас не забивал её собственное движение.
    """
    out: List[Tuple[Optional[int], float]] = []
    for chord in chords:
        root = chord.root_midi
        fifth = root + (chord.pitch_classes[2] - chord.pitch_classes[0]) % 12
        if dense:
            out.extend([(root, 1.0), (root, 1.0), (fifth, 1.0), (root, 1.0)])
        else:
            out.extend([(root, 2.0), (fifth, 2.0)])
    return tuple(out)


def _build_pad(
    chords: Sequence[ChordWindow],
) -> Tuple[Tuple[Optional[Tuple[int, ...]], float], ...]:
    """Пэд: трезвучие такта, взятое целиком и выдержанное весь такт."""
    return tuple((chord.tones, chord.beats) for chord in chords)


def _build_counter(
    timed: Sequence[Tuple[float, Optional[int], float]],
    chords: Sequence[ChordWindow],
) -> Tuple[Tuple[Optional[int], float], ...]:
    """Контрмелодия: тема, положенная на ближайший аккордовый тон снизу.

    Ритм — ритм темы нота в ноту, высота — ближайший тон текущего аккорда
    строго ниже ноты темы (терция или секста снизу, смотря что подвернётся
    в аккорде). Это классический второй голос: он движется вместе с темой,
    но никогда не спорит с гармонией, потому что состоит только из её нот.
    Паузы темы остаются паузами.
    """
    out: List[Tuple[Optional[int], float]] = []
    for onset, midi, dur in timed:
        if midi is None:
            out.append((None, dur))
            continue
        chord = _chord_at(chords, onset)
        if chord is None:
            out.append((None, dur))
            continue
        below = [
            note
            for pc in chord.pitch_classes
            for note in (midi - ((midi - pc) % 12 or 12),)
        ]
        # Ближайший аккордовый тон снизу, но не ближе малой терции:
        # секунда под темой звучит как грязь, а не как второй голос.
        candidates = [n for n in below if midi - n >= 3]
        out.append((max(candidates) if candidates else None, dur))
    return tuple(out)


def _chord_at(
    chords: Sequence[ChordWindow], beat: float
) -> Optional[ChordWindow]:
    for chord in chords:
        if chord.start <= beat < chord.start + chord.beats:
            return chord
    return chords[-1] if chords else None


# ---------------------------------------------------------------------------
# Ударные
# ---------------------------------------------------------------------------


def _onset_histogram(
    timed: Sequence[Tuple[float, Optional[int], float]],
) -> List[float]:
    """Плотность атак темы по 16-м долям такта, сложенная по всем тактам.

    Свёртка по тактам (а не по всей теме) — потому что рисунок ударных
    длиной в такт: нам нужно, КУДА внутри такта тема чаще всего бьёт, а
    не где она бьёт в конкретном такте.
    """
    hist = [0.0] * STEPS_PER_BAR
    step_beats = BEATS_PER_BAR / STEPS_PER_BAR
    for onset, midi, dur in timed:
        if midi is None:
            continue
        step = int(round((onset % BEATS_PER_BAR) / step_beats)) % STEPS_PER_BAR
        # Длинная нота на доле — более весомая опора, чем проходящая
        # шестнадцатая, поэтому вес атаки растёт с её длительностью.
        hist[step] += 1.0 + min(float(dur), 2.0)
    return hist


def _build_drums(hist: Sequence[float]) -> str:
    """Рисунок бочки и малого из плотности атак темы.

    Бочка встаёт на первую долю (она же начало лупа) и на ту из
    оставшихся долей, куда тема бьёт сильнее всего; малый — на
    оставшиеся доли. Так грув повторяет собственный акцент мелодии,
    а не навязывает ей чужой.
    """
    beats = [0, 4, 8, 12]
    rest = sorted(beats[1:], key=lambda step: -hist[step])
    kick = {0, rest[0]}
    snare = set(beats) - kick

    pattern = ["."] * STEPS_PER_BAR
    for step in kick:
        pattern[step] = "X"
    for step in snare:
        pattern[step] = "o"
    return "".join(pattern)


def _build_hats(timed: Sequence[Tuple[float, Optional[int], float]]) -> str:
    """Хэты: сетка по плотности темы — 16-е, 8-е или четверти.

    Медиана длительности нот — устойчивая мера «мелкости» темы (среднее
    сбивает одна длинная финальная нота). Хэты мельче самой темы звучат
    как суета, крупнее — как будто их забыли включить.
    """
    durs = [dur for _onset, midi, dur in timed if midi is not None]
    typical = median(durs) if durs else 1.0
    if typical <= 0.3:
        every = 1
    elif typical <= 0.6:
        every = 2
    else:
        every = 4
    return "".join(
        "-" if step % every == 0 else "." for step in range(STEPS_PER_BAR)
    )


# ---------------------------------------------------------------------------
# Единый вход
# ---------------------------------------------------------------------------


def harmonize(
    notes: Sequence[Tuple[Optional[int], float]],
    bpm: int,
    root: str,
    scale: str,
) -> Harmonization:
    """Разложить тему на партии: аккорды, бас, пэд, контрмелодию, ударные.

    Args:
        notes: тема как ``[(midi|None, биты)]`` — выход RTTTL-парсера,
            уже выровненный по такту.
        bpm: темп темы.
        root: тоника, определённая по теме (``detect_key``).
        scale: лад, определённый по теме.

    Returns:
        :class:`Harmonization` — все партии в абсолютных MIDI и битах.

    Raises:
        ValueError: тема пустая или состоит из одних пауз — выводить
            гармонию не из чего.
    """
    if not notes:
        raise ValueError("Пустая тема: гармонизировать нечего.")
    if all(midi is None for midi, _dur in notes):
        raise ValueError("Тема состоит из одних пауз: гармонизировать нечего.")

    timed, total = _timed(notes)
    chords = _pick_chords(timed, total, root, scale)
    hist = _onset_histogram(timed)

    sounding = [dur for _onset, midi, dur in timed if midi is not None]
    dense = bool(sounding) and median(sounding) <= 0.75

    return Harmonization(
        bpm=int(bpm),
        root=root,
        scale=scale,
        bars=len(chords),
        chords=chords,
        lead=tuple((midi, float(dur)) for midi, dur in notes),
        bass=_build_bass(chords, dense),
        pad=_build_pad(chords),
        counter=_build_counter(timed, chords),
        drums=_build_drums(hist),
        hats=_build_hats(timed),
    )
