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

Ручки подготовки темы (ADR-0132 PR-3)
-------------------------------------
Три авто-решения подготовки — ``HarmonizeOptions.key_detection``
(``auto`` — корреляция + тональный центр #2873, ``profile`` — чистый
Крумхансл), ``lead_octave`` (``auto`` — к рабочему регистру, ``keep``,
либо -2..+2 октавы от записанного) и ``lead_outliers`` (``fix``/``keep``)
— исполняются здесь, до :func:`~core.harmonize.harmonize`; остальные
ручки передаются в неё. Выбранное значение каждой ручки пишется в
``decisions`` (``key_detection``, ``lead_octave_mode``,
``lead_outliers_mode``) для партитуры. Все по умолчанию — байт-в-байт
прежнее поведение (``test_arranger_golden``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

# Тональность перенесена в rob_box_music (ADR-0149 PR-3): одна реализация, здесь — импорт/реэкспорт.
from rob_box_music.tonality import (  # noqa: F401 — _pitch_weights/_profile_score нужны старым тестам
    KeyCandidate,
    _pitch_weights,
    _profile_score,
    detect_key,
    detect_key_ranked,
    key_fit,
)

from rob_box_music.knowledge import ROOTS as VALID_ROOTS
from rob_box_music.model import BEATS_PER_BAR
from .harmonize import (
    AUTO,
    DEFAULT_DRUM_STYLE,
    HarmonizeOptions,
    _weighted_percentile,
    harmonize,
)
from .rtttl import parse_rtttl

__all__ = [
    "RtttlMelody",
    "rtttl_to_melody",
    "KeyCandidate",
    "detect_key",
    "detect_key_ranked",
    "melody_to_compose_params",
    "key_fit",
    "ContourBreak",
    "detect_contour_breaks",
    "detect_half_time_risk",
]


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


def melody_to_compose_params(
    melody: RtttlMelody,
    drum_style: str = DEFAULT_DRUM_STYLE,
    root: Optional[str] = None,
    scale: Optional[str] = None,
    options: Optional[HarmonizeOptions] = None,
) -> Dict[str, object]:
    """RTTTL-мелодия → плоские параметры ``compose_music``.

    Возвращает dict с ключами:
      * ``bpm`` — темп из RTTTL (``compose_music.bpm``);
      * ``root`` / ``scale`` — определённая тональность (для баса/подклада);
      * ``lead_midi`` — строка абсолютных MIDI через запятую (``None`` = пауза);
      * ``lead_dur`` — ритм в битах, той же длины;
      * ``harmony`` — :class:`core.harmonize.Harmonization`: та же тема,
        разложенная на бас, пэд, контрмелодию и рисунки ударных.

    Мелодия выравнивается по такту с обоих концов: затакт в начале сдвигает
    сетку паузой-лид-ином, чтобы первая сильная нота темы попала на долю 0
    такта (:func:`_anacrusis_lead_in`, issue #2960), а хвостовая пауза
    доводит луп до целого числа тактов (:func:`_snap_to_bar`) — иначе луп
    плывёт относительно ударной сетки и тема звучит «не в тайминг».

    НОТЫ аккомпанемента модель больше не выбирает: они выведены из самой
    темы (``harmony``). За моделью остаются тембры, форма и темп — см.
    :mod:`core.harmonize`. Плоские ``lead_midi``/``lead_dur`` остаются в
    ответе для обратной совместимости и для логов.

    ``drum_style`` — жанровый каркас ударных темы (issue #2841, см.
    :data:`core.harmonize.DRUM_STYLES`); ``auto`` — прежний рисунок.

    ``decisions`` (ADR-0132) — что автоматика решила за модель по дороге:
    свёртка темпа, хвостовая пауза, перенос регистра темы, подтянутые
    выбросы, ранжированные кандидаты тональности. Только запись: на ноты
    и на аккомпанемент она не влияет (golden-тест ``test_arranger_golden``).

    ``root`` / ``scale`` (ADR-0132 PR-2, issue #3293) — явная тональность
    от модели. При заданной тонике ТЕМА ТРАНСПОНИРУЕТСЯ целиком (та же
    мелодия, сдвиг ближайший по регистру, :func:`_transpose_theme_to_key`),
    а аккомпанемент строится в итоговой тональности. Если заданный лад не
    ложится на тему (major↔minor), берётся относительная тональность
    (F# major → «A minor» = тема в C major), иначе — лад темы с заданной
    тоникой; замена пишется в ``decisions`` (``key_transpose``,
    ``key_scale_kept``, ``key_relative``). Заданная только тоника берёт
    лад определённой тональности, заданный только лад — её тонику (тема не
    двигается). Значения должны быть уже проверены (``harmonize.check_root``
    / ``check_scale``). Оба ``None`` — прежнее поведение байт-в-байт.
    В ``decisions`` пишутся ``key_detected`` и ``key_fit`` (доля
    длительности темы в итоговом ладу) для предупреждения партитуры.

    ``options`` (ADR-0132 PR-3) — ручки :class:`core.harmonize.HarmonizeOptions`:
    ``key_detection``/``lead_octave``/``lead_outliers`` исполняются здесь,
    остальные — в :func:`~core.harmonize.harmonize`. ``None`` — все ``auto``.
    """
    options = options or HarmonizeOptions()
    source_bpm = melody.bpm
    half_time_risk = detect_half_time_risk(melody)
    folded = _normalize_tempo(melody)
    leadin = _anacrusis_lead_in(folded)
    snapped = _snap_to_bar(leadin)
    registered = _apply_lead_octave(snapped, options.lead_octave)
    contour_breaks = detect_contour_breaks(registered)
    contoured = _apply_contour_fixes(registered, contour_breaks)
    melody = _apply_lead_outliers(contoured, options.lead_outliers)
    ranked = detect_key_ranked(
        [m for m, _ in melody.notes],
        [d for _, d in melody.notes],
        method=options.key_detection,
    )
    explicit = root is not None or scale is not None
    key_moves: Dict[str, object] = {}
    if root is not None:
        melody, root, scale, key_moves = _transpose_theme_to_key(
            melody, ranked[0], root, scale
        )
    root = root or ranked[0].root
    scale = scale or ranked[0].scale
    midi: List[str] = ["None" if m is None else str(int(m)) for m, _ in melody.notes]
    dur: List[str] = [f"{d:g}" for _, d in melody.notes]
    anacrusis_pad = sum(d for _, d in leadin.notes) - sum(d for _, d in folded.notes)
    decisions = _prep_decisions(
        source_bpm, (folded, leadin, snapped, registered, melody), ranked
    )
    decisions.update(_prep_option_decisions(options))
    decisions.update(
        _prep_quality_decisions(contour_breaks, half_time_risk)
    )
    if explicit:
        decisions.update(_explicit_key_decisions(melody, ranked[0], root, scale))
        decisions.update(key_moves)
    return {
        "bpm": melody.bpm,
        "root": root,
        "scale": scale,
        "lead_midi": ", ".join(midi),
        "lead_dur": ", ".join(dur),
        "harmony": harmonize(
            melody.notes, melody.bpm, root, scale, drum_style=drum_style,
            options=options, anacrusis_pad=anacrusis_pad,
        ),
        "decisions": decisions,
    }


def _shift_melody(melody: RtttlMelody, shift: int) -> RtttlMelody:
    if not shift:
        return melody
    return RtttlMelody(
        bpm=melody.bpm,
        notes=tuple((None if m is None else m + shift, d) for m, d in melody.notes),
    )


def _nearest_shift(melody: RtttlMelody, from_pc: int, to_pc: int) -> int:
    """Сдвиг в полутонах ``from_pc → to_pc``: ближайший (−6..+5), регистр в рамках."""
    shift = (to_pc - from_pc + 6) % 12 - 6
    pitches = [m for m, _ in melody.notes if m is not None]
    if not pitches:
        return shift
    lo, hi = min(pitches), max(pitches)
    if hi + shift > _LEAD_MAX_CEILING and lo + shift - 12 >= _LEAD_MIN_FLOOR:
        shift -= 12
    elif lo + shift < _LEAD_MIN_FLOOR and hi + shift + 12 <= _LEAD_MAX_CEILING:
        shift += 12
    return shift


def _transpose_theme_to_key(
    melody: RtttlMelody, detected: KeyCandidate, root: str, scale: Optional[str]
) -> Tuple[RtttlMelody, str, Optional[str], Dict[str, object]]:
    """Перенести ТЕМУ в заданную тонику (issue #3293); вернуть итоговые root/scale.

    Раньше явная тональность перегармонизировала только аккомпанемент, а
    тема играла абсолютным MIDI — лид F# major поверх Am/Em (key_fit 0.2).
    Кандидаты по порядку: тоника темы → ``root`` (лад как просили); если
    лад темы другой — относительная тональность (тема ложится в ноты
    заданной «root scale»); иначе лад темы с заданной тоникой. Берётся
    первый с ``key_fit ≥`` :data:`_KEY_FIT_OK`.
    """
    tonic = VALID_ROOTS.index(detected.root)
    target = VALID_ROOTS.index(root)
    want = scale or detected.scale
    cands: List[Tuple[int, str, str]] = [(target, want, "")]
    rel = {("major", "minor"): 3, ("minor", "major"): -3}.get((detected.scale, want))
    if rel is not None:
        cands.append(((target + rel) % 12, want, "relative"))
    cands.append((target, detected.scale, "scale_kept"))
    best: Optional[Tuple[float, RtttlMelody, int, str, str]] = None
    for pc, sc, tag in cands:
        moved = _shift_melody(melody, _nearest_shift(melody, tonic, pc))
        fit = key_fit(moved.notes, root, sc)
        if best is None or fit > best[0] + 1e-9:
            best = (fit, moved, pc, sc, tag)
        if fit >= _KEY_FIT_OK:
            best = (fit, moved, pc, sc, tag)
            break
    assert best is not None
    _fit, moved, _pc, sc, tag = best
    moves: Dict[str, object] = {"key_transpose": _nearest_shift(melody, tonic, _pc)}
    if tag == "relative":
        moves["key_relative"] = True
    if tag == "scale_kept" and scale is not None and sc != scale:
        moves["key_scale_kept"] = sc
    return moved, root, sc, moves


#: Минимальный key_fit, при котором транспонированная тема считается «в ладу».
_KEY_FIT_OK = 0.9


def key_honesty_note(prep: Optional[Dict[str, object]]) -> str:
    """Строка для ОТВЕТА ``compose_music``: что сделали с явной тональностью.

    ``prep`` — результат :func:`melody_to_compose_params` (или ``None``).
    Пусто, если явной тональности не было, либо она применена как просили и
    тема в ней сидит. Иначе — честно: сдвиг, замена лада, остаточный спор.
    """
    decisions = (prep or {}).get("decisions") or {}
    if not isinstance(decisions, dict) or decisions.get("key_source") != "explicit":
        return ""
    det = decisions.get("key_detected") or ("?", "?")
    exp = decisions.get("key_explicit") or ("?", "?")
    parts: List[str] = []
    shift = decisions.get("key_transpose")
    if shift:
        parts.append(
            f"тема (в оригинале {det[0]} {det[1]}) перенесена на {shift:+d} пт "
            "под заданную тональность"
        )
    if decisions.get("key_relative"):
        parts.append(
            f"лад «{exp[1]}» ложится на тему только как относительный — "
            f"тоника звучит иначе, чем {exp[0]}"
        )
    kept = decisions.get("key_scale_kept")
    if kept:
        parts.append(
            f"заданный лад не ложится на тему и ЗАМЕНЁН на «{kept}» "
            f"(тоника {exp[0]})"
        )
    fit = decisions.get("key_fit")
    if isinstance(fit, (int, float)) and fit < 0.7:
        parts.append(f"тема всё равно в ладу лишь на {fit:.0%} длительности")
    if not parts:
        return ""
    return "Тональность: " + "; ".join(parts) + "."


def _apply_lead_octave(melody: "RtttlMelody", mode: object) -> "RtttlMelody":
    """Регистр темы по ручке ``lead_octave`` (ADR-0132 PR-3).

    ``auto`` — :func:`_normalize_lead_register` (к рабочему регистру);
    ``keep`` — как записано, БЕЗ нормализации; целое N — N октав от уже
    НОРМАЛИЗОВАННОГО регистра (см. ниже), с клампом в рабочий диапазон.

    🔴 FIX (live 24.09, issue #2962): ручной сдвиг применялся поверх
    СЫРОЙ, ненормализованной темы — ``_LEAD_MAX_CEILING`` в этом случае не
    проверялся вовсе. Live-прогон: ``terminat`` (auto переносит в рабочий
    регистр, медиана ~80) + ``lead_octave='+1'`` → тема уехала в MIDI
    104-111 (свист, «темы не слышно»), контрмелодия следом за ней — до 107
    (:func:`~core.harmonize._build_counter` кладёт её от нот темы, поэтому
    отдельного клампа не требует — чинится клампом самой темы).

    Теперь ручной сдвиг считается от того же нормализованного регистра,
    что и ``auto`` (:func:`_normalize_lead_register`) — ``+1``/``-1``
    значит «на октаву выше/ниже РАБОЧЕГО регистра», а не записанного as
    is. Если результат не помещается в :data:`_LEAD_MAX_CEILING` /
    :data:`_LEAD_MIN_FLOOR` — честная ``ValueError`` вместо тихого выхода
    за рабочий диапазон: смысла в частичном (не целую октаву) сдвиге нет
    — это была бы уже не та тема.
    """
    if mode == AUTO:
        return _normalize_lead_register(melody)
    if mode == "keep":
        return melody
    shift_octaves = int(mode)  # type: ignore[call-overload]
    if shift_octaves == 0:
        return melody
    base = _normalize_lead_register(melody)
    pitches = [m for m, _dur in base.notes if m is not None]
    if not pitches:
        return base
    shift = 12 * shift_octaves
    lo, hi = min(pitches), max(pitches)
    if hi + shift > _LEAD_MAX_CEILING:
        raise ValueError(
            f"lead_octave={mode!r}: тема уже у потолка рабочего регистра "
            f"после нормализации (макс. нота {hi}, потолок "
            f"{_LEAD_MAX_CEILING}) — выше сдвигать нельзя, иначе тема "
            "уйдёт в свист. Оставь auto/keep или меньший сдвиг."
        )
    if lo + shift < _LEAD_MIN_FLOOR:
        raise ValueError(
            f"lead_octave={mode!r}: тема уже у пола рабочего регистра "
            f"после нормализации (мин. нота {lo}, пол {_LEAD_MIN_FLOOR}) "
            "— ниже сдвигать нельзя, тема утонет в басу. Оставь auto/keep "
            "или меньший сдвиг."
        )
    return RtttlMelody(
        bpm=base.bpm,
        notes=tuple((None if m is None else m + shift, d) for m, d in base.notes),
    )


def _apply_lead_outliers(melody: "RtttlMelody", mode: str) -> "RtttlMelody":
    """Выбросы темы по ручке ``lead_outliers``: ``fix`` — подтянуть, ``keep`` — нет."""
    if mode == "keep":
        return melody
    return _fix_isolated_lead_outliers(melody)


def _prep_option_decisions(options: HarmonizeOptions) -> Dict[str, object]:
    """Значения ручек подготовки темы (ADR-0132 PR-3) — для партитуры."""
    return {
        "key_detection": options.key_detection,
        "lead_octave_mode": options.lead_octave,
        "lead_outliers_mode": options.lead_outliers,
    }


def _prep_quality_decisions(
    contour_breaks: Sequence[ContourBreak], half_time_risk: Optional[str]
) -> Dict[str, object]:
    """Итог общего детектора качества (issue #2959).

    Для партитуры/``lookup_melody``.

    ``contour_breaks_fixed`` — сколько разрывов исправлено автоматически
    (однозначные одиночные октавные ошибки); ``contour_breaks_flagged`` —
    сколько осталось только пометкой (правка неоднозначна). Оба числа —
    не по конкретной записи, а по тому, что реально нашёл/применил
    :func:`detect_contour_breaks` для ЭТОЙ темы.
    """
    fixed = [b for b in contour_breaks if b.auto_fixable]
    flagged = [b for b in contour_breaks if not b.auto_fixable]
    return {
        "contour_breaks_fixed": len(fixed),
        "contour_breaks_flagged": len(flagged),
        "contour_break_details": [
            {
                "index": b.index,
                "before": b.note_before,
                "at": b.note_at,
                "interval": b.interval,
                "fixed": b.auto_fixable,
            }
            for b in contour_breaks
        ],
        "half_time_risk": half_time_risk,
    }


def _explicit_key_decisions(
    melody: RtttlMelody, detected: KeyCandidate, root: str, scale: str
) -> Dict[str, object]:
    """Запись явной тональности вызова рядом с определённой (ADR-0132 PR-2)."""
    return {
        "key_source": "explicit",
        "key_explicit": (root, scale),
        "key_detected": (detected.root, detected.scale),
        "key_fit": key_fit(melody.notes, root, scale),
    }


#: Сколько кандидатов тональности кроме лучшего показывать в решениях.
_KEY_ALTERNATIVES = 3


def _prep_decisions(
    source_bpm: int,
    steps: Tuple[RtttlMelody, RtttlMelody, RtttlMelody, RtttlMelody, RtttlMelody],
    ranked: Sequence[KeyCandidate],
) -> Dict[str, object]:
    """Запись авто-решений подготовки темы (ADR-0132) — только для партитуры.

    ``steps`` — тема после каждого шага конвейера: свёртка темпа,
    затактовый лид-ин (issue #2960), выравнивание по такту, перенос
    регистра, подтяжка выбросов. Решения считаются сравнением соседних
    шагов, сами шаги не трогаются.
    """
    folded, leadin, snapped, registered, final = steps
    before = [m for m, _ in snapped.notes if m is not None]
    after = [m for m, _ in registered.notes if m is not None]
    moved = sum(
        1 for (a, _da), (b, _db) in zip(registered.notes, final.notes) if a != b
    )
    gap = ranked[0].score - ranked[1].score if len(ranked) > 1 else 0.0
    return {
        "source_bpm": int(source_bpm),
        "bpm": int(final.bpm),
        "anacrusis_pad_beats": round(
            sum(d for _, d in leadin.notes) - sum(d for _, d in folded.notes), 4
        ),
        "tail_pad_beats": round(
            sum(d for _, d in snapped.notes) - sum(d for _, d in leadin.notes), 4
        ),
        "lead_shift": (after[0] - before[0]) if before else 0,
        "outliers_moved": moved,
        "key_ranked": [
            (c.root, c.scale, round(c.score, 3))
            for c in ranked[: 1 + _KEY_ALTERNATIVES]
        ],
        "key_gap": round(gap, 3),
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


class ContourBreak(NamedTuple):
    """Разрыв контура фразы: скачок ПРОТИВ уже установившегося направления.

    ``index`` — позиция в ``RtttlMelody.notes`` (с паузами, для правки на
    месте). ``note_before``/``note_at`` — MIDI до и на разрыве.
    ``interval`` — полутона от ``note_before`` до ``note_at`` (знак —
    направление). ``run_direction`` — направление разбитого хода (+1 вверх,
    -1 вниз). ``auto_fixable`` — перенос ``note_at`` на октаву навстречу
    ходу (``octave_shift``, ``±12``) продолжает ход шагом, без создания
    нового разрыва; для не-однозначных случаев ``octave_shift == 0`` и
    правка не применяется — только пометка (см. :func:`detect_contour_breaks`).
    """

    index: int
    note_before: int
    note_at: int
    interval: int
    run_direction: int
    auto_fixable: bool
    octave_shift: int


#: Шаг мелодии (полутона), который ещё считается «плавным ходом» и может
#: наращивать бегущий тренд направления (секунда/терция/кварта — обычный
#: словарь ступенчатого/скачкообразного, но не «сорвавшегося», движения).
_CONTOUR_STEP_MAX = 4

#: Минимальный скачок (полутона), который считается «разрывом» контура,
#: если он идёт ПРОТИВ уже установившегося направления — тритон/септима и
#: шире редко встречаются как продолжение ровного хода в реальных темах и
#: являются типичным следом ошибки транскрипции на октаву (issue #2959,
#: живой прогон 24.09: гимн России, «...вели́кая сла́ва» — после трёх ходов
#: вверх ``A→B→C6`` следующая нота была записана на септиму НИЖЕ ожидаемого
#: продолжения, хотя реально мелодия идёт кульминационно вверх).
_CONTOUR_LEAP_MIN = 7

#: Сколько подряд шагов в ОДНОМ направлении нужно, чтобы считать его
#: «установившимся трендом» и вообще ПОМЕТИТЬ разрыв — с одного шага
#: направление не показательно (могло быть просто широкой, но осмысленной
#: терцией/квартой).
_CONTOUR_RUN_MIN = 2

#: Сколько подряд шагов нужно для АВТОПРАВКИ (строже :data:`_CONTOUR_RUN_MIN`
#: — только пометки). 🔴 Живая проверка на архиве (issue #2959): при
#: run_min=2 детектор ловил и настоящую октавную ошибку гимна (3-шаговый
#: разбег ``A→B→C6`` перед разрывом), и ЛОЖНОЕ срабатывание на той же
#: записи — двухшаговый разбег ``G5→A5→B5`` перед совершенно законным
#: мелодическим ходом вниз на квинту (``B5→E5``, обычная фигура, не
#: ошибка). Требование трёх шагов разбега отличает «мелодия явно набрала
#: направление» от «просто два соседних интервала подряд» — без этого
#: узла автоправка молча портила бы настоящую музыку.
_CONTOUR_RUN_MIN_FIX = 3

#: Насколько широким (полутонов) может быть шаг ПОСЛЕ применения фикса,
#: чтобы он всё ещё правдоподобно продолжал ровный ход — чуть шире
#: :data:`_CONTOUR_STEP_MAX`, потому что кульминация фразы иногда берёт
#: шаг чуть больше обычного (issue #2959: реальный скачок к D6 — секунда
#: от C6, вписывается с запасом).
_CONTOUR_FIX_MAX_RESULT_STEP = _CONTOUR_STEP_MAX + 2

#: Диапазон (полутона) вокруг ровной октавы, в котором сам разрыв должен
#: лежать, чтобы считаться ПОХОЖИМ на октавную ошибку, а не на широкий, но
#: законный ход мелодии вниз/вверх — реальная октавная опечатка даёт
#: разрыв примерно «октава минус/плюс маленький шаг» (12 ± до
#: :data:`_CONTOUR_STEP_MAX`), а не любой большой интервал.
_CONTOUR_FIX_LEAP_RANGE = (12 - _CONTOUR_STEP_MAX, 12 + _CONTOUR_STEP_MAX)


def detect_contour_breaks(melody: RtttlMelody) -> List[ContourBreak]:
    """Найти скачки, разрывающие уже установившийся ход мелодии.

    🔴 Источник (issue #2959, живой прогон 24.09.2026): «национальный
    гимн» в архиве RTTTL оказался записан с октавной ошибкой ровно в
    точке кульминации восходящей фразы — вместо продолжения вверх мелодия
    «падала» на септиму. Товарищ Шифу попросил ОБЩИЙ детектор такого
    класса ошибок (не патч одной записи, ADR-0132/ADR-0018): скачок
    ПРОТИВ уже установившегося направления (не менее
    :data:`_CONTOUR_RUN_MIN` шагов подряд одним курсом, каждый шаг —
    не шире :data:`_CONTOUR_STEP_MAX` полутонов) величиной от
    :data:`_CONTOUR_LEAP_MIN` полутонов и больше — типичный след того,
    что записанная нота промахнулась на октаву мимо настоящей.

    Пауза (``None``) МЕЖДУ двумя нотами обрывает тренд и НЕ участвует в
    проверке разрыва вовсе — она почти всегда сама и есть граница фразы
    (следующая фраза законно начинается на другой высоте, это не ошибка).

    🔴 FIX (живая проверка на реальном архиве, вторая итерация): первая
    версия работала по списку ЗВУЧАЩИХ нот без пауз (`_pitched_with_index`,
    удалено) — интервал между нотами считался и ЧЕРЕЗ паузу. На «К Элизе»
    (``furelise``) это ловило ложное срабатывание: после разбега
    восходящими шагами арпеджио обрывается ПАУЗОЙ, и с паузы начинается
    повтор темы («ми-ре#-ми...») на новой высоте — совершенно законный
    возврат к началу пьесы, а не «упавшая на октаву» нота. Разрыв через
    паузу детектор больше не проверяет вовсе.

    Каждый найденный разрыв возвращается как :class:`ContourBreak`.
    ``auto_fixable=True`` — только если ВСЁ сразу: перенос ноты на октаву
    НАВСТРЕЧУ ходу превращает разрыв в шаг не шире
    :data:`_CONTOUR_FIX_MAX_RESULT_STEP` полутонов в ТУ ЖЕ сторону, что и
    установившийся тренд; разбег не короче :data:`_CONTOUR_RUN_MIN_FIX`
    шагов; сам разрыв — в пределах :data:`_CONTOUR_FIX_LEAP_RANGE`; и нота
    НЕ разрешается гладко дальше по мелодии (см. 🔴 FIX ниже). Иначе —
    только пометка (``octave_shift == 0``), без правки: то же правило
    «честный FAIL лучше красивого PASS» (AGENTS.md) — детектор не гадает,
    если после фикса разрыв не закрывается уверенно.

    🔴 FIX (живая проверка, третья итерация): «The Final Countdown»
    (``finalcou``) после первых двух фильтров всё ещё ловился ложно —
    3+-шаговый спуск, затем скачок ВВЕРХ к высокой ноте (реальная
    кульминация риффа), и СРАЗУ ЗА НЕЙ, без паузы, шаг вниз на разрешение
    (её собственный «выдох»). У настоящей октавной ошибки (гимн России)
    такого гладкого продолжения нет — ошибочная нота либо последняя перед
    паузой, либо сама рвёт мелодию дальше. Поэтому нота, чей СЛЕДУЮЩИЙ
    (смежный, без паузы) сосед — маленький шаг от неё самой, в
    ``auto_fixable`` не идёт: она уже встроена в непрерывную линию,
    трогать её — рисковать задуманным широким ходом, а не ошибкой.

    Работает на ЛЮБОЙ записи архива (не завязан на конкретную мелодию,
    имя или список): применяется как общий шаг пайплайна
    (:func:`melody_to_compose_params`) и как отдельный аудит
    (``scripts/music/melody_quality_report.py``).
    """
    breaks: List[ContourBreak] = []
    run_dir = 0
    run_len = 0
    # (индекс в notes, MIDI) последней ноты, смежной с текущей (без
    # паузы между ними).
    prev: Optional[Tuple[int, int]] = None
    for i, (m, _d) in enumerate(melody.notes):
        if m is None:
            # Пауза — граница фразы: тренд не переживает её, и разрыв
            # через неё не проверяется (следующая нота начинает НОВЫЙ
            # отсчёт, а не продолжает прерванный).
            run_dir = 0
            run_len = 0
            prev = None
            continue
        if prev is None:
            prev = (i, m)
            continue
        prev_idx, prev_m = prev
        delta = m - prev_m

        is_established_run = run_len >= _CONTOUR_RUN_MIN and run_dir != 0
        leap_dir = 1 if delta > 0 else -1
        is_reversal = (
            is_established_run
            and abs(delta) >= _CONTOUR_LEAP_MIN
            and leap_dir == -run_dir
        )
        if is_reversal:
            breaks.append(
                _evaluate_contour_break(melody, i, prev_m, m, run_dir, run_len)
            )
            # Разрыв закрывает тренд — не каскадировать на следующий шаг.
            run_dir = 0
            run_len = 0
            prev = (i, m)
            continue

        step_dir = 1 if delta > 0 else (-1 if delta < 0 else 0)
        is_step = 0 < abs(delta) <= _CONTOUR_STEP_MAX
        if is_step and step_dir == run_dir:
            run_len += 1
        elif is_step:
            run_dir = step_dir
            run_len = 1
        else:
            run_dir = 0
            run_len = 0
        prev = (i, m)
    return breaks


def _evaluate_contour_break(
    melody: RtttlMelody,
    index: int,
    prev_m: int,
    m: int,
    run_dir: int,
    run_len: int,
) -> ContourBreak:
    """Собрать :class:`ContourBreak` и решить ``auto_fixable`` для разрыва.

    Вынесено из :func:`detect_contour_breaks` отдельной функцией — там
    решается ТОЛЬКО когда разрыв вообще есть (бегущий тренд + скачок
    против него), здесь — насколько он однозначен для автоправки. См.
    докстринг :func:`detect_contour_breaks` про все условия и их историю
    (живые проверки на реальном архиве, три итерации).
    """
    delta = m - prev_m
    shift = 12 * run_dir
    fixed_delta = (m + shift) - prev_m
    leap_near_octave = (
        _CONTOUR_FIX_LEAP_RANGE[0] <= abs(delta) <= _CONTOUR_FIX_LEAP_RANGE[1]
    )
    # Нота УЖЕ гладко разрешается ДАЛЬШЕ по мелодии (следующая, смежная,
    # без паузы — маленький шаг) — значит, встроена в непрерывную линию
    # и это, вероятнее всего, ЗАДУМАННЫЙ широкий ход (скачок к
    # кульминации, шаг вниз на разрешение), а не одинокая ошибка
    # транскрипции. Реальная октавная опечатка либо завершает фразу перед
    # паузой, либо сама продолжает разрыв — не разрешается гладко.
    has_next = index + 1 < len(melody.notes)
    next_m = melody.notes[index + 1][0] if has_next else None
    resolves_forward = (
        next_m is not None and abs(next_m - m) <= _CONTOUR_STEP_MAX
    )
    fixed_step_ok = 0 < fixed_delta * run_dir <= _CONTOUR_FIX_MAX_RESULT_STEP
    auto_fixable = (
        run_len >= _CONTOUR_RUN_MIN_FIX
        and leap_near_octave
        and not resolves_forward
        and fixed_step_ok
    )
    return ContourBreak(
        index=index,
        note_before=prev_m,
        note_at=m,
        interval=delta,
        run_direction=run_dir,
        auto_fixable=auto_fixable,
        octave_shift=shift if auto_fixable else 0,
    )


def _apply_contour_fixes(
    melody: RtttlMelody, breaks: Sequence[ContourBreak]
) -> RtttlMelody:
    """Применить ТОЛЬКО однозначные (``auto_fixable``) правки.

    Правки берутся из :func:`detect_contour_breaks`.
    """
    shifts = {
        b.index: b.octave_shift
        for b in breaks
        if b.auto_fixable and b.octave_shift
    }
    if not shifts:
        return melody
    notes = list(melody.notes)
    for idx, shift in shifts.items():
        m, dur = notes[idx]
        if m is not None:
            notes[idx] = (m + shift, dur)
    return RtttlMelody(bpm=melody.bpm, notes=tuple(notes))


#: Минимум звучащих нот, чтобы половинно-темповую эвристику вообще
#: применять — короткие мотивы (сигналы, риффы) слишком коротки, чтобы
#: статистика длительностей вообще что-то показывала.
_HALF_TIME_MIN_NOTES = 24

#: Темп (b=), ниже которого россыпь мелких длительностей — обычное дело
#: (медленная тема из одних восьмых при b=70 звучит нормально) и эвристика
#: не применяется вовсе.
_HALF_TIME_MIN_BPM = 150

#: Длительность (в долях такта), начиная с которой нота считается «долгой»
#: (половинная и длиннее). Достаточно ОДНОЙ такой ноты на тему, чтобы
#: снять подозрение.
_HALF_TIME_LONG_NOTE_BEATS = 2.0

#: Длительность (в долях такта) — «шестнадцатая или мельче»: собственно
#: единица, которую подозреваем как «записанную вместо восьмой/четверти».
_HALF_TIME_FINE_NOTE_BEATS = 0.25

#: Доля нот темы мельче :data:`_HALF_TIME_FINE_NOTE_BEATS`, начиная с
#: которой это уже не «немного быстрых проходящих нот», а «весь ритм
#: составлен из самой мелкой единицы».
_HALF_TIME_FINE_FRACTION = 0.7


def detect_half_time_risk(melody: RtttlMelody) -> Optional[str]:
    """Эвристика «запись в половинных длительностях» — ТОЛЬКО пометка.

    Без правки — см. подробности ниже.

    🔴 issue #2959: у гимна России весь ритм был записан вдвое мельче
    настоящего (половинная — четвертью и т.п.) при темпе, который сам по
    себе не выглядит подозрительно (``b=125`` — в рабочем диапазоне,
    :func:`_normalize_tempo` его не трогает). Без эталонной партитуры
    отличить «так и задумано» от «записано вдвое мельче» алгоритмически
    нельзя — поэтому это ТОЛЬКО предупреждение (в духе AGENTS.md «честный
    FAIL лучше красивого PASS»), не автоправка.

    🔴 Живая проверка на архиве (issue #2959, вторая итерация): первая
    версия («нет ни одной ноты ≥ половинной, темп ≥100») помечала ~4000 из
    10461 записей (40% архива) — архив ``mixed3`` почти целиком состоит из
    попсовых рингтонов, у которых «весь ритм из восьмых/шестнадцатых на
    быстром темпе» — норма жанра, а не брак. Такой уровень шума
    бесполезен для модели («подозрительно почти всё» = «не подозрительно
    ничто»). Порог ужесточён по трём осям сразу (не по одной — иначе то же
    перекрытие с нормой жанра): длинная тема
    (:data:`_HALF_TIME_MIN_NOTES`+ нот), высокий темп
    (:data:`_HALF_TIME_MIN_BPM`+) И подавляющее большинство нот темы
    (:data:`_HALF_TIME_FINE_FRACTION`) короче шестнадцатой
    (:data:`_HALF_TIME_FINE_NOTE_BEATS`) — то есть ритм не просто «быстрый
    рингтон», а «явно весь записан на ступень мельче своей естественной
    единицы». На архиве это ~0.7% записей (см.
    ``scripts/music/melody_quality_report.py``) — сам ``national_2``
    (b=125) под порог :data:`_HALF_TIME_MIN_BPM` не попадает: это
    ОБЩИЙ, а не подогнанный под гимн детектор (задача issue #2959 —
    системная эвристика, не фикс одной записи).

    Не привязан к конкретной записи/имени/тегу — тот же вызов для любой
    темы архива.
    """
    durations = [d for m, d in melody.notes if m is not None]
    if len(durations) < _HALF_TIME_MIN_NOTES:
        return None
    if melody.bpm < _HALF_TIME_MIN_BPM:
        return None
    if any(d >= _HALF_TIME_LONG_NOTE_BEATS for d in durations):
        return None
    fine = sum(1 for d in durations if d <= _HALF_TIME_FINE_NOTE_BEATS)
    fine_fraction = fine / len(durations)
    if fine_fraction < _HALF_TIME_FINE_FRACTION:
        return None
    return (
        f"{len(durations)} нот, b={melody.bpm}, {fine_fraction:.0%} нот "
        f"<= {_HALF_TIME_FINE_NOTE_BEATS:g} доли и ни одной >= "
        f"{_HALF_TIME_LONG_NOTE_BEATS:g} долей — похоже на запись в "
        "половинных длительностях (весь ритм на ступень мельче метра)"
    )


#: Перцентили (взвешенные длительностью), задающие «корпус темы» для
#: :func:`_fix_isolated_lead_outliers` — 10-й и 90-й, тот же выбор, что и
#: у потолка подклада (``harmonize._PAD_CEILING_PERCENTILE``): достаточно
#: широкий, чтобы не задеть саму тему, и достаточно узкий, чтобы короткий
#: затакт/проходящая нота в него не попали.
_LEAD_OUTLIER_LO_PERCENTILE = 0.10
_LEAD_OUTLIER_HI_PERCENTILE = 0.90

#: Дальше скольких полутонов от края корпуса нота считается «выбросом» и
#: переносится октавой ближе. 12 — сама октава: issue #2876 явно требует
#: убирать именно скачки БОЛЬШЕ октавы, оставляя обычные широкие ходы
#: мелодии (терцдецима, октава с хвостиком в рабочем диапазоне) нетронутыми.
_LEAD_OUTLIER_OCTAVE_SPAN = 12


def _fix_isolated_lead_outliers(melody: RtttlMelody) -> RtttlMelody:
    """Затакт/одиночную ноту дальше октавы от корпуса темы — подтянуть к нему.

    🔴 FIX (issue #2876, живой прогон 23.09.2026, «диджей Снупдог» — Still
    Dre): :func:`_normalize_lead_register` переносит ВСЮ тему октавами как
    один блок — она не может починить одну ноту внутри уже нормальной
    темы. У Still Dre затакт перед каждой фразой (4 раза, четверть длины
    соседних нот) стоял на MIDI 72, тело фразы — на 87-89; после общего
    сдвига в рабочий регистр это 60 против 75-77 — скачок в 15-17
    полутонов на КАЖДОМ повторе затакта, и потолок подклада
    (``harmonize._pad_ceiling``) до FIX #2876 в этом модуле садился на
    затакт же, утаскивая подклад в бас.

    «Корпус темы» — диапазон между 10-м и 90-м перцентилем высоты нот,
    взвешенным длительностью (:func:`~core.harmonize._weighted_percentile`,
    тот же приём, что у потолка подклада): короткий затакт не может
    сдвинуть перцентиль, его вес тонет в весе длинных нот тела фразы.

    Нота дальше :data:`_LEAD_OUTLIER_OCTAVE_SPAN` полутонов от ближайшего
    края корпуса переносится ЦЕЛЫМИ октавами навстречу корпусу — ровно
    «затакт переносится октавой к теме» из акцептанса, но универсально
    (не только затакты, любая одиночная нота вне корпуса), и ровно
    настолько, чтобы выйти из-под четвертьоктавного разрыва, не залезая
    внутрь корпуса дальше необходимого.

    Применяется ПОСЛЕ :func:`_normalize_lead_register` (весь блок уже в
    рабочем регистре) и ДО :func:`~core.harmonize.harmonize` — по той же
    причине: гармонизация строит бас/пэд от фактической высоты нот темы.
    """
    pairs = [(m, dur) for m, dur in melody.notes if m is not None]
    if len(pairs) < 2:
        return melody
    core_lo = int(round(_weighted_percentile(pairs, _LEAD_OUTLIER_LO_PERCENTILE)))
    core_hi = int(round(_weighted_percentile(pairs, _LEAD_OUTLIER_HI_PERCENTILE)))

    changed = False
    out: List[Tuple[Optional[int], float]] = []
    for m, dur in melody.notes:
        if m is None:
            out.append((m, dur))
            continue
        note = m
        while note < core_lo - _LEAD_OUTLIER_OCTAVE_SPAN:
            note += 12
        while note > core_hi + _LEAD_OUTLIER_OCTAVE_SPAN:
            note -= 12
        if note != m:
            changed = True
        out.append((note, dur))

    if not changed:
        return melody
    return RtttlMelody(bpm=melody.bpm, notes=tuple(out))


def _anacrusis_lead_in(melody: RtttlMelody) -> RtttlMelody:
    """Затакт (пикап) — добавить паузу в начало, чтобы сильная доля темы
    совпала с долей 0 такта аккомпанемента (issue #2960).

    🔴 FIX (live 24.09, гимн России ``national_2``: ``8g,8p,c6,8g.,...``):
    тема начинается с короткой затактовой ноты (``G``, восьмая), за ней —
    первая ОПОРНАЯ, сильная нота (``C6``, четверть). Аранжировщик ставит
    ПЕРВУЮ ноту темы на долю 0 (см. :func:`~core.harmonize.harmonize`,
    :func:`_timed`) — поэтому опорная нота гимна («си-») оказывалась на
    доле 1, а не на сильной доле такта, и била мимо каркаса ударных
    (бочка ``X`` на 0/4, малый ``o`` на 2/4) и смен аккорда пэда (тоже по
    долям такта): на слух — «ноты промахиваются».

    Детектор — базовый критерий затакта, тот же, что и в основе
    :func:`_first_strong_note` для тональности (issue #2961 сузил ЕЁ
    критерий до тоники конкретного кандидата — здесь кандидата ещё нет,
    детекция идёт ДО :func:`detect_key_ranked`, поэтому оставлен простой
    и root-независимый признак): если первая звучащая нота темы КОРОЧЕ
    следующей звучащей ноты, она — затакт, а следующая — первая сильная
    нота фразы. Общий признак затакта (пикап слабее и короче опорной
    ноты), без привязки к конкретной песне (ADR-0132).

    Величина паузы — ровно столько долей, чтобы онсет первой сильной ноты
    (считая паузы между затактом и ней) стал кратен такту
    (:data:`rob_box_music.model.BEATS_PER_BAR`): затакт оказывается в хвосте
    нового вступительного такта (доигрывает его последними долями — «или
    в intro», см. issue), а первая сильная нота начинает СЛЕДУЮЩИЙ такт
    ровно с доли 0, синхронно с ударными и первой сменой аккорда пэда.

    Тема без затакта (первая звучащая нота не короче второй, как в
    большинстве RTTTL-рингтонов) не трогается — байт-в-байт прежнее
    поведение (``test_arranger_golden``).
    """
    notes = melody.notes
    sounding_idx = [i for i, (m, _d) in enumerate(notes) if m is not None]
    if len(sounding_idx) < 2:
        return melody
    first_i, second_i = sounding_idx[0], sounding_idx[1]
    first_dur = notes[first_i][1]
    second_dur = notes[second_i][1]
    if first_dur >= second_dur:
        return melody
    onset = sum(d for _, d in notes[:second_i])
    remainder = onset % BEATS_PER_BAR
    if remainder == 0:
        return melody
    pad = BEATS_PER_BAR - remainder
    return RtttlMelody(bpm=melody.bpm, notes=((None, pad),) + tuple(notes))


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
