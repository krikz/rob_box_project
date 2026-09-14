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

from dataclasses import dataclass, replace
from statistics import median
from typing import Dict, List, Optional, Sequence, Tuple

from .arranger import BEATS_PER_BAR, SCALE_INTERVALS, VALID_ROOTS

__all__ = [
    "ChordWindow",
    "Harmonization",
    "harmonize",
]

#: Порог плотности темы: атак на один бит.
#:
#: 🔴 FIX (live 14.09, «Pink Panther не узнать»): плотность считалась по
#: МЕДИАННОЙ ДЛИНЕ ноты, а это не то же самое. Тема Пантеры — пары
#: восьмых, разделённые паузами по два бита: ноты короткие, но времени
#: она занимает мало, и по медиане считалась «плотной». В ответ
#: аранжировка закатывала её паузы остинато на каждую долю, басом
#: четвертями, удвоением в октаву и вторым голосом — а именно в этих
#: паузах весь её характер. Тему было не узнать не потому, что ноты
#: неверные, а потому что её засыпало.
#:
#: Атаки на бит меряют ровно то, что нужно: насколько густо тема
#: заполняет время. Порог выбран по контрольной выборке — марши и
#: чиптюн дают 1.4-1.8, крадущаяся тема Пантеры 0.95.
DENSE_ONSETS_PER_BEAT = 1.2

#: Сколько 16-х в такте — сетка, на которую квантуются атаки мелодии при
#: выводе рисунка ударных. 16-я — самая мелкая длительность, которая
#: реально встречается в RTTTL-рингтонах как ритмическая (32-е там почти
#: всегда «дыхательные» паузы, а не ноты).
STEPS_PER_BAR = 16

#: Нижняя граница баса в MIDI-нотах. Ниже до-большой октавы бас на
#: динамике робота превращается в гул без высоты.
BASS_MIDI_FLOOR = 36   # C2

#: Зазор между верхней нотой подклада и самой низкой нотой темы, полутоны.
#: Целый тон — минимум, при котором подклад перестаёт сливаться с темой в
#: унисон и ловится слухом как отдельный слой.
PAD_CLEARANCE = 2

#: Куда опускать потолок подклада, если тема сама лежит низко. Без этого
#: предела низкая тема (басовый рифф, мужской вокал) вдавила бы подклад в
#: бас, и оба слоя слиплись бы в кашу.
PAD_MIDI_FLOOR = 48    # C3

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
_CHANGE_PENALTY = 0.9

#: Штраф аккорду, которого нет в определённом ладу темы.
#:
#: Заимствованные аккорды нужны (см. :func:`_chord_candidates`), но по
#: умолчанию гармония обязана оставаться в тональности: без штрафа любой
#: хроматический проход тянул бы за собой смену аккорда, и тональный
#: центр рассыпался бы. Недиатонический аккорд выигрывает только когда
#: объясняет ноты окна заметно лучше любого диатонического.
_CHROMATIC_PENALTY = 1.6

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
    #: Атак темы на один бит. Мера того, насколько густо тема заполняет
    #: время; ею определяется, сколько аранжировки тема выдержит.
    density: float
    #: ``density >= DENSE_ONSETS_PER_BEAT``. Плотной теме положен полный
    #: наряд (остинато по долям, удвоение в октаву, второй голос), редкой
    #: — только скелет.
    dense: bool
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


def _chord_candidates(
    root_semitone: int, intervals: Sequence[int]
) -> List[Tuple[Tuple[int, ...], bool]]:
    """Все аккорды-кандидаты: ``(классы высоты, диатонический ли)``.

    Кандидаты строятся на ВСЕХ ДВЕНАДЦАТИ ступенях хроматики, мажорные и
    минорные, а не только на семи ступенях лада.

    🔴 FIX (live 14.09, «Марио — восемь одинаковых аккордов»): диатоникой
    одного лада часто нечем описать такт. Заимствованные аккорды (♭VI,
    ♭VII), побочные доминанты и короткие отклонения — обычное дело в
    мелодиях этой библиотеки, и без них такт сваливается на тонику по
    умолчанию. Плюс это страховка от неточной тональности: даже когда
    ``detect_key`` ошибся ладом, нужный аккорд остаётся достижим.

    Строятся только мажорные и минорные трезвучия: увеличенные и
    уменьшённые, которые раньше возникали сами собой из терций по ладу,
    как выдержанный аккорд слышатся фальшью, а не краской.

    Недиатонические помечаются, а не запрещаются: вызывающий добавляет им
    штраф, чтобы они выигрывали только там, где действительно объясняют
    ноты окна лучше диатонических.

    🔴 FIX (live 14.09, «марш странно играет»): «диатонический» проверялся
    как ПРИНАДЛЕЖНОСТЬ НОТ ЛАДУ — а это не то же самое, что «аккорд этого
    лада». В ре гармоническом миноре трезвучие на си-бемоле мажорное
    (A#-D-F), но си-бемоль МИНОР (A#-C#-F) тоже проходил проверку: его
    до-диез — это повышенная седьмая ступень, она в ладу есть. И он шёл
    без штрафа наравне с правильным, выигрывая в каждом такте, где звучит
    до-диез, — хотя его ре-бемоль бьётся с ре, тоникой всей пьесы.
    Теперь диатоническими считаются ровно те трезвучия, что строятся
    терциями ПО СТУПЕНЯМ лада.
    """
    diatonic_sets = {
        frozenset(_triad_pitch_classes(degree, root_semitone, intervals))
        for degree in range(len(intervals))
    }
    out: List[Tuple[Tuple[int, ...], bool]] = []
    for pc_root in range(12):
        for third in (4, 3):  # мажорное и минорное трезвучие
            pcs = (pc_root, (pc_root + third) % 12, (pc_root + 7) % 12)
            out.append((pcs, frozenset(pcs) in diatonic_sets))
    return out


def _pick_chords(
    timed: Sequence[Tuple[float, Optional[int], float]],
    total_beats: float,
    root: str,
    scale: str,
) -> Tuple[ChordWindow, ...]:
    """Выбрать гармонию: аккорд на каждые полтакта, соседние одинаковые слить.

    Окно — ПОЛТАКТА, а не такт. Гармония живых мелодий меняется и внутри
    такта, а окном в целый такт такая смена невидима: обе половины
    усредняются, побеждает тоника, и тема получает один аккорд на всю
    длину. Слияние соседних одинаковых окон возвращает целый такт там, где
    половины согласны, — то есть частота смены гармонии определяется самой
    мелодией, а не сеткой.

    Скор окна: длительности нот, попавших в аккорд (корень весомее), бонус
    за совпадение с первой нотой окна, бонус тонике на стыке лупа, штрафы
    недиатоническому аккорду и смене аккорда.
    """
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    root_semitone = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    candidates = _chord_candidates(root_semitone, intervals)
    pad_ceiling = _pad_ceiling(timed)
    tonic_pcs = _triad_pitch_classes(0, root_semitone, intervals)

    window = BEATS_PER_BAR / 2.0
    count = max(1, int(round(total_beats / window)))

    picked: List[Tuple[float, Tuple[int, ...]]] = []
    previous: Optional[Tuple[int, ...]] = None
    for index in range(count):
        begin = index * window
        weights = _pitch_weights(timed, begin, begin + window)
        downbeat = _first_note_in(timed, begin, begin + window)
        is_edge = index == 0 or index == count - 1
        # Инерция и бонусы соразмерны весу окна: в почти пустом окне
        # (хвост темы, выдержанная нота) абсолютная добавка перевешивала
        # сами ноты и намертво тянула прошлый аккорд.
        scale_factor = min(1.0, sum(weights.values()) / window)

        best: Tuple[int, ...] = tonic_pcs
        best_score = float("-inf")
        for pcs, diatonic in candidates:
            score = sum(weights.get(pc, 0.0) for pc in pcs)
            # Бонус за корень — ТОЛЬКО диатоническим. Он существует, чтобы
            # разводить аккорды лада с общими нотами (i и VI в миноре — две
            # ноты из трёх общие). Хроматическому он давал выиграть по
            # совпадению баса: в гимне окно B(1.5)+A(0.5) забрал СИ МАЖОР,
            # объясняющий из него одну ноту — свою же тонику, — обойдя
            # диатонический G на 0.05 балла.
            if diatonic:
                score += (_ROOT_WEIGHT - 1.0) * weights.get(pcs[0], 0.0)
            if not diatonic:
                score -= _CHROMATIC_PENALTY * scale_factor
            if downbeat is not None and downbeat % 12 == pcs[0]:
                score += _DOWNBEAT_BONUS * scale_factor
            if is_edge and pcs == tonic_pcs:
                score += _CADENCE_BONUS * scale_factor
            if previous is not None and pcs != previous:
                score -= _CHANGE_PENALTY * scale_factor
            if score > best_score:
                best_score = score
                best = pcs
        picked.append((begin, best))
        previous = best

    # Слияние: соседние окна с одним аккордом становятся одним окном.
    chords: List[ChordWindow] = []
    for begin, pcs in picked:
        if chords and chords[-1].pitch_classes == pcs:
            last = chords[-1]
            chords[-1] = replace(last, beats=last.beats + window)
            continue
        chords.append(
            ChordWindow(
                start=float(begin),
                beats=float(window),
                degree=_scale_degree(pcs[0], root_semitone, intervals),
                root_midi=_lift(pcs[0], BASS_MIDI_FLOOR),
                tones=_stack_chord(pcs, pad_ceiling),
                pitch_classes=pcs,
            )
        )
    return tuple(chords)


def _pad_ceiling(
    timed: Sequence[Tuple[float, Optional[int], float]],
) -> int:
    """Потолок подклада: на :data:`PAD_CLEARANCE` ниже самой низкой ноты темы.

    Именно самой низкой, а не средней: достаточно одной ноты темы,
    попавшей в аккорд подклада, чтобы её атака в нём утонула.
    """
    pitches = [midi for _onset, midi, _dur in timed if midi is not None]
    if not pitches:
        return PAD_MIDI_FLOOR
    return max(PAD_MIDI_FLOOR, min(pitches) - PAD_CLEARANCE)


def _scale_degree(
    pitch_class: int, root_semitone: int, intervals: Sequence[int]
) -> int:
    """Ступень лада для корня аккорда; ``-1`` для недиатонического."""
    offset = (pitch_class - root_semitone) % 12
    for degree, semitones in enumerate(intervals):
        if semitones % 12 == offset:
            return degree
    return -1


def _lift(pitch_class: int, floor_midi: int) -> int:
    """Класс высоты → ближайшая MIDI-нота не ниже ``floor_midi``."""
    note = floor_midi + ((pitch_class - floor_midi) % 12)
    return note


def _stack_chord(
    pitch_classes: Sequence[int], ceiling: int
) -> Tuple[int, ...]:
    """Классы высоты → аккорд, уложенный ВНИЗ от ``ceiling``.

    🔴 FIX (live 14.09, «будто ноты пропускает»): аккорд складывался ВВЕРХ
    от постоянного пола (E3), не глядя, где лежит тема. У имперского марша
    это ставило подклад в D4-D5 — прямо в тему: 23 из 66 её нот подклад
    играл В УНИСОН и долбил их аккордом на каждую долю. Атаки мелодии
    тонули в этом пульсе, и на слух казалось, что тема пропускает ноты.

    В оркестровке аккомпанемент стоит ПОД мелодией — это не стиль, а
    условие того, чтобы мелодию было слышно. Поэтому укладываем вниз от
    потолка, а потолок задаёт вызывающий по самой низкой ноте темы.

    Кладём именно последовательно вниз (каждая следующая нота ниже
    предыдущей), а не берём три ближайшие по отдельности: иначе аккорд
    вывернется в случайное обращение и подклад будет прыгать регистром от
    такта к такту.

    Ровно три ноты, без удвоений. Удвоение корня октавой ниже тут было —
    его добавляли ради веса, когда подклад стоял в регистре темы и звучал
    тонко. Теперь вес даёт разделение регистров, а удвоение только роняло
    подклад в бас (у марша — до D2, прямо в басовую партию) и стоило
    лишнего голоса scsynth на каждой доле остинато.
    """
    out: List[int] = []
    current = ceiling
    for pc in reversed(pitch_classes):
        note = current - ((current - pc) % 12)
        out.append(note)
        current = note - 1
    return tuple(sorted(out))


# ---------------------------------------------------------------------------
# Партии
# ---------------------------------------------------------------------------


#: Форма басовой линии внутри одного окна гармонии: индексы тонов аккорда
#: (0 — корень, 1 — терция, 2 — квинта) по шагам.
#:
#: Корень на сильных долях, квинта в середине, терция как краска. Это
#: скелет; интереснее его делают подходы к следующему аккорду, см.
#: :func:`_approach_note`.
_BASS_SHAPE_DENSE = (0, 0, 2, 1)
_BASS_SHAPE_SPARSE = (0, 2)


def _approach_note(previous: int, target: int) -> int:
    """Нота-подход к ``target``: полутон снизу или сверху, что ближе к ``previous``.

    Подход — то, чем осмысленная басовая линия отличается от механической.
    Без него бас просто перескакивает на новый корень, и смена гармонии
    ничем не подготовлена; с ним последняя нота перед сменой ведёт в неё
    за полтона, и линия слышится как ЛИНИЯ, а не как набор опор.

    Сторона выбирается по близости к предыдущей ноте, чтобы бас шёл
    плавно, а не прыгал октавами ради подхода.
    """
    below, above = target - 1, target + 1
    return below if abs(below - previous) <= abs(above - previous) else above


def _build_bass(
    chords: Sequence[ChordWindow], dense: bool
) -> Tuple[Tuple[Optional[int], float], ...]:
    """Бас: тоны аккорда по долям, с подходом к следующему аккорду.

    ``dense`` (плотная тема, атака почти на каждой доле — марш, чиптюн)
    даёт бас четвертями: ровный шаг держит такую тему лучше, чем половины,
    под которыми она рассыпается. Разреженная тема получает половины,
    чтобы бас не забивал её собственное движение.

    Последняя нота перед сменой гармонии заменяется на подход к корню
    следующего аккорда (:func:`_approach_note`) — кроме случая, когда на
    всё окно приходится одна нота: там опора важнее движения.

    Окна гармонии переменной длины (см. :func:`_pick_chords`), поэтому шаг
    раскладывается по фактической длине окна, а остаток достаётся
    последней ноте: сумма длительностей баса обязана совпадать с темой
    нота в ноту, иначе партии разъедутся на первом же повторе лупа.

    Аккорды берутся ПО КРУГУ: тема зациклена, и последнее окно ведёт не в
    тишину, а обратно в первое.
    """
    step = 1.0 if dense else 2.0
    shape = _BASS_SHAPE_DENSE if dense else _BASS_SHAPE_SPARSE
    out: List[Tuple[Optional[int], float]] = []
    previous = chords[0].root_midi if chords else BASS_MIDI_FLOOR
    for index, chord in enumerate(chords):
        following = chords[(index + 1) % len(chords)]
        changes = following.pitch_classes[0] != chord.pitch_classes[0]
        tones = tuple(
            chord.root_midi + (pc - chord.pitch_classes[0]) % 12
            for pc in chord.pitch_classes
        )
        count = max(1, int(chord.beats // step))
        remainder = chord.beats - count * step
        for position in range(count):
            is_last = position == count - 1
            if is_last and changes and count > 1:
                note = _approach_note(previous, following.root_midi)
            else:
                note = tones[shape[position % len(shape)]]
            out.append((note, step + (remainder if is_last else 0.0)))
            previous = note
    return tuple(out)


def _build_pad(
    chords: Sequence[ChordWindow], dense: bool
) -> Tuple[Tuple[Optional[Tuple[int, ...]], float], ...]:
    """Подклад: аккорд, ПОВТОРЯЕМЫЙ по долям, а не выдержанный весь такт.

    🔴 FIX (live 14.09, «звучит плосковато, соло и фон»): здесь был один
    аккорд на такт длиной в такт. Выдержанное созвучие под подвижной темой
    на слух перестаёт быть партией и превращается в подложку — ровно то,
    что слышал человек. В настоящей оркестровке марша струнные ведут
    РИТМИЧЕСКОЕ остинато сквозь всю пьесу: повторяющаяся фигура и есть
    маршевый шаг, а гармонию она держит заодно.

    Плотная тема получает аккорд на каждой доле (маршевый шаг), разреженная
    — на сильных долях, чтобы не забивать собственное движение темы.
    Короткими эти удары делает ``sus`` на стороне аранжировщика: длинные
    ноты слились бы обратно в тот же выдержанный аккорд.
    """
    step = 1.0 if dense else 2.0
    out: List[Tuple[Optional[Tuple[int, ...]], float]] = []
    for chord in chords:
        hits = max(1, int(round(chord.beats / step)))
        out.extend([(chord.tones, step)] * hits)
    return tuple(out)


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


def _build_drums(hist: Sequence[float], dense: bool) -> str:
    """Рисунок бочки и малого: жёсткий каркас + синкопа от мелодии.

    🔴 FIX (live 14.09, «ломаные ритмы»): здесь бочка ставилась на первую
    долю и на ту из оставшихся, куда тема бьёт сильнее всего, а малый —
    на все прочие. Правило выглядело «выводим грув из мелодии», а на деле
    давало хромые рисунки: у Pink Panther вышло ``X...o...o...X...``
    (бочка на 1 и 4, то есть ДВЕ БОЧКИ ПОДРЯД на стыке тактов), у Марио —
    ``X...X...o...o...`` (бочка на 1 и 2). Ни то, ни другое не читается
    как доля.

    Обратный бит — малый на 2 и 4 — это не вкус и не свойство конкретной
    мелодии, а то, из чего вообще складывается ощущение доли в
    четырёхдольном размере. Поэтому каркас фиксирован: бочка на 1, малый
    на 2 и 4. Мелодия решает ровно одно: даётся ли бочка на 3.

    🔴 FIX #2 (live 14.09): вместе с починкой каркаса сюда добавлялась
    ещё и «синкопа от мелодии» — добавочная бочка на той восьмушке между
    долями, куда тема бьёт сильнее. Задумывалась как то, что отличает
    грув одного трека от другого, а на деле сломала марш: он получил
    бочку на «и» второй доли сразу после малого (``X...o.X.X...o...``),
    и квадратный маршевый шаг превратился в хромой. Признак был негодный
    — у марша вес распределён 40% на долях и 40% между ними, то есть он
    вовсе не синкопирован, просто много шестнадцатых. Разнообразие грува
    даёт выбор сэмпла и плотность хэтов; выдумывать его в рисунке бочки
    не нужно.
    """
    pattern = ["."] * STEPS_PER_BAR
    pattern[0] = "X"
    pattern[4] = "o"
    pattern[12] = "o"
    if dense:
        pattern[8] = "X"
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

    onsets = sum(1 for _onset, midi, _dur in timed if midi is not None)
    density = onsets / total if total > 0 else 0.0
    dense = density >= DENSE_ONSETS_PER_BEAT

    return Harmonization(
        bpm=int(bpm),
        root=root,
        scale=scale,
        bars=max(1, int(round(total / BEATS_PER_BAR))),
        density=density,
        dense=dense,
        chords=chords,
        lead=tuple((midi, float(dur)) for midi, dur in notes),
        bass=_build_bass(chords, dense),
        pad=_build_pad(chords, dense),
        counter=_build_counter(timed, chords),
        drums=_build_drums(hist, dense),
        hats=_build_hats(timed),
    )
