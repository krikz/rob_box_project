"""RTTTL-мелодия → плоские параметры ``compose_music``.

Связка между RTTTL-библиотекой (SQLite, :mod:`core.rtttl_library`) и
композитором (``ComposeMusicTool``). Раньше модель сама разбирала RTTTL и
генерировала Renardo-код вручную — это был ручной шаг, на котором она
ошибалась (корень #1810 «сыграл гамму и назвал её кузнечиком»). Теперь
``compose_music(name=..., variants=...)`` сам ищет мелодию, конвертирует
ноты в абсолютные MIDI и передаёт их аранжировщику.

Абсолютные MIDI (``lead_midi``) + точный ритм (``lead_dur``) — путь ТОЧНОГО
воспроизведения: аранжировщик играет тему дословно, а форму, бас и ударные
строит вокруг неё. Ступени лада (``lead_notes``) сюда не подходят: в них
нельзя выразить хроматические ноты (диез/бемоль вне лада), поэтому точность
мелодии была бы потеряна.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .arranger import BEATS_PER_BAR, SCALE_INTERVALS, VALID_ROOTS
from .harmonize import DEFAULT_DRUM_STYLE, harmonize
from .rtttl import parse_rtttl

__all__ = [
    "RtttlMelody",
    "rtttl_to_melody",
    "detect_key",
    "melody_to_compose_params",
]

#: Профили Крумхансл-Шмуклера: насколько «своей» слышится каждая ступень
#: хроматики в мажоре и в миноре. Числа — усреднённые оценки слушателей
#: из психоакустических экспериментов Кэрол Крумхансл; тоника весит
#: больше всех, за ней доминанта и медианта.
#:
#: 🔴 FIX (live 14.09): здесь считалось, сколько веса нот ПОПАДАЕТ в лад.
#: Такой счёт не различает параллельные тональности и лады-повороты в
#: принципе: у ля-минора и до-мажора набор нот совпадает полностью, у
#: ля-фригийского и фа-мажора тоже — счёт у них одинаков до последнего
#: знака, и выбор решал порядок перебора, то есть монетка. «В пещере
#: горного короля» (ля-минор) определялась как ре-мажор, «Ода к радости»
#: (фа-мажор) — как ля-фригийский.
#:
#: Профиль различает их, потому что смотрит НЕ на вхождение ноты в лад, а
#: на то, какие ступени несут вес: тема в миноре задерживается на минорной
#: терции, тема в мажоре — на большой. Сравнение идёт корреляцией, а не
#: суммой, чтобы результат не зависел от общей длины темы.
_KRUMHANSL_MAJOR = (
    6.35, 2.23, 3.48, 2.33, 4.38, 4.09, 2.52, 5.19, 2.39, 3.66, 2.29, 2.88,
)
_KRUMHANSL_MINOR = (
    6.33, 2.68, 3.52, 5.38, 2.60, 3.53, 2.54, 4.75, 3.98, 2.69, 3.34, 3.17,
)


#: Штраф за долю веса, лежащую ВНЕ лада. Соразмерен корреляции (та живёт
#: в [-1, 1]), поэтому тональность, не содержащую заметной части нот темы,
#: он снимает, а на выбор между двумя одинаково подходящими не влияет.
_OUT_OF_SCALE_PENALTY = 2.0


def _correlation(xs: Sequence[float], ys: Sequence[float]) -> float:
    """Корреляция Пирсона двух векторов одной длины (0.0 при вырождении)."""
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    dx = [x - mean_x for x in xs]
    dy = [y - mean_y for y in ys]
    denom = (sum(v * v for v in dx) * sum(v * v for v in dy)) ** 0.5
    if denom == 0:
        return 0.0
    return sum(a * b for a, b in zip(dx, dy)) / denom


@dataclass(frozen=True)
class RtttlMelody:
    """Разобранная RTTTL-мелодия: темп + список ``(midi|None, доли)``.

    ``midi`` — абсолютный номер MIDI-ноты (``None`` — пауза).
    ``dur`` — длительность ноты в битах (четверть = 1.0).
    """

    bpm: int
    notes: Tuple[Tuple[Optional[int], float], ...]


def rtttl_to_melody(rtttl: str) -> RtttlMelody:
    """Разобрать RTTTL-строку в :class:`RtttlMelody` (через ``core.rtttl``)."""
    _name, bpm, notes = parse_rtttl(rtttl)
    return RtttlMelody(bpm=bpm, notes=tuple(notes))


def detect_key(
    midi_notes: Sequence[Optional[int]],
    durations: Optional[Sequence[float]] = None,
) -> Tuple[str, str]:
    """Определить ``(тоника, лад)`` по набору абсолютных MIDI-нот.

    Скор каждой пары (тоника, лад) — суммарная длительность нот, лежащих в
    ладу (без ``durations`` — просто количество нот). Взвешивание по
    длительности критично: в хроматических мелодиях (марш, классика)
    встречаются все 12 ступеней, и простой подсчёт даёт одинаковый скор
    любому ладу — тоника «теряется», бас и подклад уезжают в случайный лад.
    Долгая/частая тоника перевешивает проходящие ноты.

    Паузы (``None``) игнорируются. Без нот — ``("C", "major")``.

    Тональность нужна не для самой темы (она играется абсолютным MIDI),
    а для баса и подклада, которые аранжировщик достраивает вокруг неё.
    """
    if durations is None:
        durations = [1.0] * len(midi_notes)
    weights: Dict[int, float] = {}
    for midi, dur in zip(midi_notes, durations):
        if midi is not None:
            pc = midi % 12
            weights[pc] = weights.get(pc, 0.0) + float(dur)
    if not weights:
        return "C", "major"

    profile = [weights.get(pc, 0.0) for pc in range(12)]
    total = sum(profile) or 1.0
    best_root = 0
    best_major = True
    best_score = float("-inf")
    for root_idx in range(12):
        rotated = profile[root_idx:] + profile[:root_idx]
        for is_major, reference, scale_name in (
            (True, _KRUMHANSL_MAJOR, "major"),
            (False, _KRUMHANSL_MINOR, "minor"),
        ):
            # Корреляция объясняет ИЕРАРХИЮ ступеней, но ничего не знает о
            # принадлежности: она не против ноты, которой в ладу нет вовсе.
            # На коротком фрагменте этого мало — «Jingle Bells» (фа-мажор)
            # почти не касается своей тоники и всем весом лежит на терции,
            # из-за чего выигрывал ля-минор, где си-бемоля темы просто нет.
            # Второй член требует, чтобы лад ещё и СОДЕРЖАЛ ноты темы.
            in_scale = {i % 12 for i in SCALE_INTERVALS[scale_name]}
            outside = sum(
                w
                for pc, w in weights.items()
                if (pc - root_idx) % 12 not in in_scale
            )
            score = _correlation(rotated, reference)
            score -= _OUT_OF_SCALE_PENALTY * (outside / total)
            if score > best_score:
                best_score = score
                best_root = root_idx
                best_major = is_major

    if best_major:
        return VALID_ROOTS[best_root], "major"

    # Натуральный минор или гармонический — решает седьмая ступень:
    # повышенная (вводный тон) против натуральной. Это единственное, чем
    # они отличаются, и профиль минора их не различает (он один на оба).
    raised_seventh = weights.get((best_root + 11) % 12, 0.0)
    natural_seventh = weights.get((best_root + 10) % 12, 0.0)
    scale = "harmonicMinor" if raised_seventh > natural_seventh else "minor"
    return VALID_ROOTS[best_root], scale


def melody_to_compose_params(
    melody: RtttlMelody, drum_style: str = DEFAULT_DRUM_STYLE
) -> Dict[str, object]:
    """RTTTL-мелодия → плоские параметры ``compose_music``.

    Возвращает dict с ключами:
      * ``bpm`` — темп из RTTTL (``compose_music.bpm``);
      * ``root`` / ``scale`` — определённая тональность (для баса/подклада);
      * ``lead_midi`` — строка абсолютных MIDI через запятую (``None`` = пауза);
      * ``lead_dur`` — ритм в битах, той же длины;
      * ``harmony`` — :class:`core.harmonize.Harmonization`: та же тема,
        разложенная на бас, пэд, контрмелодию и рисунки ударных.

    Мелодия выравнивается по такту (хвостовая пауза доводит луп до целого
    числа тактов) — иначе луп плывёт относительно ударной сетки и тема
    звучит «не в тайминг».

    НОТЫ аккомпанемента модель больше не выбирает: они выведены из самой
    темы (``harmony``). За моделью остаются тембры, форма и темп — см.
    :mod:`core.harmonize`. Плоские ``lead_midi``/``lead_dur`` остаются в
    ответе для обратной совместимости и для логов.

    ``drum_style`` — жанровый каркас ударных темы (issue #2841, см.
    :data:`core.harmonize.DRUM_STYLES`); ``auto`` — прежний рисунок.
    """
    melody = _snap_to_bar(_normalize_tempo(melody))
    melody = _normalize_lead_register(melody)
    root, scale = detect_key(
        [m for m, _ in melody.notes],
        [d for _, d in melody.notes],
    )
    midi: List[str] = ["None" if m is None else str(int(m)) for m, _ in melody.notes]
    dur: List[str] = [f"{d:g}" for _, d in melody.notes]
    return {
        "bpm": melody.bpm,
        "root": root,
        "scale": scale,
        "lead_midi": ", ".join(midi),
        "lead_dur": ", ".join(dur),
        "harmony": harmonize(
            melody.notes, melody.bpm, root, scale, drum_style=drum_style
        ),
    }


#: Рабочий диапазон темпа аранжировщика (совпадает с ``arranger.BPM_RANGE``).
#: Держим копию, а не импорт, по той же причине, что и SCALE_INTERVALS:
#: модуль остаётся независимым от деталей рендера.
_TEMPO_RANGE = (60.0, 180.0)


def _normalize_tempo(melody: RtttlMelody) -> RtttlMelody:
    """Свернуть темп в рабочий диапазон, ВДВОЕ меняя и bpm, и длительности.

    🔴 FIX (live 14.09): аранжировщик клампит bpm в [60, 180], а в архиве
    1321 мелодия записана быстрее и 547 медленнее — 18% библиотеки. Кламп
    не трогает длительности, поэтому такая мелодия играла в чужом темпе:
    «В пещере горного короля» с ``b=260`` превращалась в 180 и шла на
    треть медленнее, чем задумано.

    Сворачивание вдвое звучит РОВНО так же: половинный темп с половинными
    длительностями даёт то же абсолютное время (``beats/2`` при ``bpm/2``
    — та же секунда), просто «четверть при 260» записывается как «восьмая
    при 130». Это стандартная смена единицы записи, а не изменение музыки.

    Выход из диапазона больше чем вдвое-втрое встречается (до ``b=900``),
    поэтому свёртка идёт циклом; ограничитель шагов защищает от
    вырожденных значений вроде ``b=0``.
    """
    bpm = float(melody.bpm)
    if bpm <= 0:
        return melody
    factor = 1.0
    for _ in range(8):
        if bpm > _TEMPO_RANGE[1]:
            bpm /= 2.0
            factor /= 2.0
        elif bpm < _TEMPO_RANGE[0]:
            bpm *= 2.0
            factor *= 2.0
        else:
            break
    if factor == 1.0:
        return melody
    return RtttlMelody(
        bpm=int(round(bpm)),
        notes=tuple((midi, dur * factor) for midi, dur in melody.notes),
    )


#: Центр рабочего регистра лида — MIDI 78 (между C5=72 и C6=84, см.
#: ``rtttl._to_midi``: ``12*(octave+1)+semitone`` — стандартная MIDI-шкала,
#: где C4=60). Аранжировщик строит вокруг темы бас (``BASS_MIDI_FLOOR=36``,
#: C2) и подклад (``PAD_MIDI_FLOOR=48``, C3, потолок — на 2 полутона ниже
#: САМОЙ НИЗКОЙ ноты темы, см. ``harmonize._pad_ceiling``): если тема стоит
#: в o=7 (медиана ~MIDI 98, как у мусорной ``russiann``, issue #2840), пэд и
#: контрмелодия громоздятся следом за ней туда же, в тот же визг, а не под
#: неё. Транспонирование — единственный рычаг: инструменты аранжировщика
#: (``imperialbrass`` и т.п.) сами по себе диапазон не ограничивают.
_LEAD_TARGET_CENTER = 78.0

#: Жёсткий потолок лида после нормализации (issue #2840, живой прогон:
#: сдвиг по одной медиане пропускал ``terminat`` — median=80 (в рабочем
#: регистре, сдвиг 0), но max=99: несколько высоких проходящих нот тянут
#: потолок за собой, медиана их не видит). Если после сдвига по медиане
#: max всё ещё выше потолка — досдвигаем ещё на октаву вниз, пока не
#: упрёмся в :data:`_LEAD_MIN_FLOOR` (чтобы не утопить и без того низкие
#: темы в подвал баса).
_LEAD_MAX_CEILING = 88
_LEAD_MIN_FLOOR = 55


def _normalize_lead_register(melody: RtttlMelody) -> RtttlMelody:
    """Транспонировать тему ЦЕЛЫМИ октавами в рабочий регистр лида.

    Двухшаговый сдвиг, оба — целыми октавами (не меняет мелодию: интервалы
    между нотами и лад сохраняются один в один, просто переносит её в
    другой регистр):

    1. По медиане высоты нот (без пауз) — к :data:`_LEAD_TARGET_CENTER`
       (~C5–C6). Тема, уже стоящая в рабочем регистре (медиана в пределах
       половины октавы от центра — округление даёт сдвиг 0), им не
       трогается.
    2. По максимуму — если после шага 1 верхняя нота всё ещё выше
       :data:`_LEAD_MAX_CEILING` (медиана не видит одиночных высоких
       проходящих нот, см. ``terminat`` MIDI 71-99 из живого прогона),
       досдвигаем вниз ещё октавами, пока максимум не впишется или
       минимум не упрётся в :data:`_LEAD_MIN_FLOOR`.

    Транспонировать нужно ДО :func:`~core.harmonize.harmonize` — гармонизация
    строит бас/пэд/контрмелодию от фактической высоты нот темы (пэд —
    "на 2 полутона ниже самой низкой ноты темы"), так что применённый после
    неё сдвиг рассинхронизировал бы тему с уже построенным аккомпанементом.
    """
    pitches = sorted(m for m, _dur in melody.notes if m is not None)
    if not pitches:
        return melody
    n = len(pitches)
    mid = n // 2
    if n % 2:
        median = float(pitches[mid])
    else:
        median = (pitches[mid - 1] + pitches[mid]) / 2.0
    shift = int(round((_LEAD_TARGET_CENTER - median) / 12.0)) * 12

    lo, hi = pitches[0], pitches[-1]
    while hi + shift > _LEAD_MAX_CEILING:
        if lo + shift - 12 < _LEAD_MIN_FLOOR:
            break
        shift -= 12

    if shift == 0:
        return melody
    return RtttlMelody(
        bpm=melody.bpm,
        notes=tuple(
            (None if m is None else m + shift, dur) for m, dur in melody.notes
        ),
    )


def _snap_to_bar(melody: RtttlMelody) -> RtttlMelody:
    """Довести длину мелодии до целого числа тактов хвостовой паузой.

    RTTTL-мелодии — рингтоны с «дыхательными» паузами (``32p``), из-за
    которых суммарная длина не кратна такту. Без выравнивания луп каждый
    повтор смещается на дробный остаток и уезжает от ударной сетки.
    """
    total = sum(d for _, d in melody.notes)
    remainder = total % BEATS_PER_BAR
    if remainder == 0:
        return melody
    pad = BEATS_PER_BAR - remainder
    return RtttlMelody(
        bpm=melody.bpm,
        notes=tuple(melody.notes) + ((None, pad),),
    )
