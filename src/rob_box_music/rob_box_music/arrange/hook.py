"""Хук из RTTTL: начало мелодии (4–8 тактов) в тональности трека и его развитие по секциям (ADR-0149 §3.3).

Берутся **первые** такты мелодии (не случайное 2-тактовое окно старого ``club_fragments``): по ним мелодию и
узнают. Ритм ринг-тона подгоняется к темпу трека степенью двойки (чтобы «быстрая» тема не стала вдвое
медленнее), ноты встают на сетку 16-х. Тональность темы — :func:`rob_box_music.tonality.detect_key`; тема
переносится на тонику трека целиком (интервалы сохраняются), лад трека — лад темы (#3293: явная тональность
транспонирует тему, а не только аккомпанемент). Хроматика темы — проходящие: ``key_fit`` (доля длительности в
ладу трека) ≥ ``knowledge.HOOK_KEY_FIT_MIN`` (I12). Регистр клампится в коридор лида (:func:`fit_register`): тема
встаёт целиком на октаву с наименьшим числом нот вне коридора, оставшиеся переносятся октавой — ближе к соседней ноте
(контур); переносом ломается больше :data:`MAX_FOLDED_SHARE` нот — октава своя у каждых двух тактов, затем у
каждого такта (#3542). Тема чужого лада (``key_fit`` ниже порога) или мотив, который и так перенос ломает, честно
отвергается :class:`HookError` — вызывающий берёт следующую мелодию.

Развитие (:func:`develop`) — детерминированные операции над мотивом по имени секции: ``build`` — первые
2 такта мотива и пауза перед дропом; ``drop`` — мотив целиком; ``break`` — начало мотива вдвое медленнее
(увеличение); ``drop2`` — мотив в параллельных терциях лада; остальные секции — без хука. Хук из материала
партитуры с ответом (``Hook.answer``, :func:`rhythm_answer`) в ``drop2`` звучит развитием ``rhythm``: ритм хука,
контур следующей фразы (ADR-0154 Н10).
"""

from __future__ import annotations

import bisect
import functools
import logging
import math
from dataclasses import replace
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..material import MaterialError, MeterMap, Phrase, ScoreMaterial, meter_map, meter_pending, validate_material
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Hook, Key, PitchEvent
from ..rtttl import contour, parse_rtttl
from ..tonality import detect_key, key_fit

_LOG = logging.getLogger(__name__)
STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
#: Длины мотива в тактах, по убыванию предпочтения (ADR-0149 §3.3: 4–8 тактов от начала).
HOOK_BARS: Tuple[int, ...] = (8, 4)
#: Во второй половине 8-тактового мотива должно быть хотя бы столько нот, иначе берём 4 такта.
SECOND_HALF_MIN_NOTES = 4
MIN_NOTES = 6
MIN_PITCHES = 3
MIN_RANGE = 3
#: Предпочтительный низ мотива: верх пэда = низ лида − 3 (ADR-0149 §3.6), пэду нужно место для трезвучия.
PREFERRED_LOW = 62
TARGET_CENTER = 72
#: Перенос октавой — не больше трети нот мотива; дальше это уже не та мелодия (#3409: «Терминатор» — до 0.29).
MAX_FOLDED_SHARE = 1 / 3
BUILD_BARS = 2
#: Короткая мелодия: фраза на столько тактов и ответ на IV (вверх) или V (вниз) ступени.
PHRASE_BARS = 2
ANSWER_STEPS: Tuple[int, ...] = (3, -3, 4, -4)


class HookError(ValueError):
    """Мелодия не годится в хук трека (коротка, вне лада, не помещается в регистр) — берётся другая."""


def time_scale(melody_bpm: float, track_bpm: float) -> float:
    """Множитель долей темы: степень двойки, ближайшая к ``track_bpm / melody_bpm`` (¼…4)."""
    if melody_bpm <= 0:
        return 1.0
    power = round(math.log2(track_bpm / melody_bpm))
    return 2.0 ** max(-2, min(2, power))


def _onsets(notes: Sequence[Tuple[Optional[int], float]], scale: float,
            bars: bool = False) -> List[Tuple[float, float, int]]:
    """Звучащие ноты ``(доля, длительность, MIDI)`` на сетке 16-х. Начальная пауза срезана у источника без тактов
    (RTTTL); у материала (``bars``) она остаётся — доли отсчитываются от начала такта автора, и затакт честно
    стоит перед сильной долей (#3531). Гармония и бас идут от того же начала такта (``harmony.material_beat``)."""
    out: List[Tuple[float, float, int]] = []
    t = 0.0
    start: Optional[float] = 0.0 if bars else None
    for midi, beats in notes:
        if midi is not None:
            start = t if start is None else start
            beat = round((t - start) / STEP_BEATS) * STEP_BEATS
            if out and beat <= out[-1][0]:
                beat = out[-1][0] + STEP_BEATS  # две ноты в одном шаге: вторая — на следующий
            out.append((beat, max(STEP_BEATS, round(beats * scale / STEP_BEATS) * STEP_BEATS), int(midi)))
        t += beats * scale
    return [(b, min(d, out[i + 1][0] - b) if i + 1 < len(out) else d, m) for i, (b, d, m) in enumerate(out)]


def _window(onsets: List[Tuple[float, float, int]]) -> Tuple[int, List[Tuple[float, float, int]], bool]:
    """``(такты, ноты, нужен_ответ)``: 8 тактов, если во второй половине есть материал, иначе 4.

    Мелодия на 1–2 такта (Калинка, Nokia) — фраза ``PHRASE_BARS`` тактов, мотив = фраза + ответ (§3.3, #2965).
    """
    last = onsets[-1][0] if onsets else -1.0
    for bars in HOOK_BARS:
        span = bars * BEATS_PER_BAR
        cut = [(b, min(d, span - b), m) for b, d, m in onsets if b < span]
        thin = bars > HOOK_BARS[-1] and sum(b >= span / 2 for b, _d, _m in cut) < SECOND_HALF_MIN_NOTES
        if last < span / 2 or thin:
            continue
        return bars, cut, False
    if last < BEATS_PER_BAR:
        raise HookError("мелодия короче такта")
    span = PHRASE_BARS * BEATS_PER_BAR
    return 2 * PHRASE_BARS, [(b, min(d, span - b), m) for b, d, m in onsets if b < span], True


def diatonic(midi: int, key: Key, steps: int) -> int:
    """Нота лада на ``steps`` ступеней выше (ниже) ``midi``; хроматическая (проходящая темы) идёт вместе с
    ближайшим звуком лада снизу и остаётся на том же расстоянии от него."""
    scale = [(key.root + st) % 12 for st in kn.SCALES[key.mode]]
    below = next(midi - k for k in range(12) if (midi - k) % 12 in scale)
    idx = scale.index(below % 12) + steps
    base = below - (below - key.root) % 12
    return base + (idx // len(scale)) * 12 + (scale[idx % len(scale)] - key.root) % 12 + (midi - below)


def _answer(cut: List[Tuple[float, float, int]], key: Key, register: Tuple[int, int]) -> List[Tuple[float, float, int]]:
    """Ответ фразе: та же фраза на IV/V ступени (диатонический сдвиг — ноты остаются в ладу), через ``PHRASE_BARS``.
    Целиком в коридор не встаёт ни одна ступень — та, где вне коридора меньше нот, с переносом октавой
    (:func:`_placement`, октава — ближе к концу фразы), если перенесено не больше :data:`MAX_FOLDED_SHARE` (#3542)."""
    tries = []
    for steps in ANSWER_STEPS:
        moved = [(b + PHRASE_BARS * BEATS_PER_BAR, d, diatonic(m, key, steps)) for b, d, m in cut]
        if all(register[0] <= m <= register[1] for _b, _d, m in moved):
            return moved
        tries.append(_placement([m for _b, _d, m in moved], 0, register, cut[-1][2]) + (moved,))
    placed, folded, moved = min(tries, key=lambda t: t[1])
    if folded > MAX_FOLDED_SHARE * len(placed):
        raise HookError(f"ответ фразе не помещается в коридор лида: {folded} из {len(placed)} нот вне коридора")
    return [(b, d, m) for (b, d, _m), m in zip(moved, placed)]


def _check_musical(cut: List[Tuple[float, float, int]]) -> None:
    pitches = [m for _b, _d, m in cut]
    if len(pitches) < MIN_NOTES or len(set(pitches)) < MIN_PITCHES or max(pitches) - min(pitches) < MIN_RANGE:
        raise HookError(f"в начале мелодии {len(pitches)} нот, {len(set(pitches))} высот — не мотив")


def track_key(hook_root: int, hook_mode: str, root: int, mode: str) -> Key:
    """Тональность трека под тему: тоника — ``root`` плана; мажорная тема в минорном плане — в параллельный мажор."""
    minor_like = hook_mode != "major"
    if not minor_like and mode != "major":
        return Key((root + 3) % 12, hook_mode)
    if minor_like and mode == "major":
        return Key((root - 3) % 12, hook_mode)
    return Key(root, hook_mode)


def _placement(pitches: Sequence[int], shift: int, register: Tuple[int, int],
               prev: Optional[int] = None) -> Tuple[List[int], int]:
    """Тема в коридоре лида и число нот, перенесённых октавой.

    Тема целиком сдвигается на ``shift`` плюс октавы — там, где меньше всего нот вне коридора (при равенстве низ
    ≥ :data:`PREFERRED_LOW`, затем начало ближе к ``prev`` — последней ноте предыдущего отрезка, если он есть, иначе
    середина ближе к :data:`TARGET_CENTER`); оставшиеся вне коридора ноты переносятся октавой в коридор — на ту, что
    ближе к предыдущей ноте (контур мотива сохраняется где можно).
    """
    lo, hi = register
    center = (min(pitches) + max(pitches)) / 2

    def outside(s: int) -> int:
        return sum(1 for m in pitches if not lo <= m + s <= hi)

    def near(s: int) -> float:
        return abs(center + s - TARGET_CENTER) if prev is None else abs(pitches[0] + s - prev)

    best = min((shift + 12 * k for k in range(-6, 7)),
               key=lambda s: (outside(s), min(pitches) + s < PREFERRED_LOW, near(s)))
    placed: List[int] = []
    for midi in pitches:
        m = midi + best
        if not lo <= m <= hi:
            last = placed[-1] if placed else (TARGET_CENTER if prev is None else prev)
            m = min((m + 12 * k for k in range(-9, 10) if lo <= m + 12 * k <= hi), key=lambda c: abs(c - last))
        placed.append(m)
    return placed, outside(best)


#: Отрезки своей октавы (тактов клуба), если мотив (тема) на своих отрезках ломается переносом
#: (:data:`MAX_FOLDED_SHARE`): по ``PHRASE_BARS`` такта, затем по такту (#3542: у партитуры с широким диапазоном
#: «Моцарт» — 7 из 20 нот вне коридора целиком; по такту октава своя, перенесённых нот — единицы).
OCTAVE_SPANS: Tuple[int, ...] = (PHRASE_BARS, 1)


def fit_register(notes: Sequence[Tuple[float, int]], ends: Sequence[float], shift: int,
                 register: Tuple[int, int]) -> Tuple[List[int], int, int]:
    """Высоты ``notes`` (``(доля, MIDI)``) в коридоре ``register`` после сдвига ``shift``: октава — своя у каждого
    отрезка до концов ``ends`` (доли: конец мотива или фраз темы, :func:`_placement`). Перенесено октавой больше
    :data:`MAX_FOLDED_SHARE` нот — отрезки мельче (:data:`OCTAVE_SPANS`), октава следующего — ближе к концу
    предыдущего (контур через стык). ``(высоты, перенесено, тактов в отрезке — 0: отрезки ``ends``)``: первая
    раскладка, где перенесено не больше доли, иначе — исходная (вызывающий отвергает с её числом).

    Единственный критерий «мотив помещается в коридор» хука, темы и годности материала (``material_unfit``)."""
    first: Optional[Tuple[List[int], int, int]] = None
    end = max(ends)
    for span in (0, *OCTAVE_SPANS):
        step = span * BEATS_PER_BAR
        cuts = sorted(set(ends) | ({k * step for k in range(1, int(end // step) + 1)} if span else set()))
        placed: List[int] = []
        folded = 0
        for start, stop in zip([0.0, *cuts], cuts):
            part = [m for b, m in notes if start - 1e-9 <= b < stop - 1e-9]
            if part:
                moved, out = _placement(part, shift, register, placed[-1] if span and placed else None)
                placed, folded = placed + moved, folded + out
        if first is None:
            first = (placed, folded, span)
        if folded <= MAX_FOLDED_SHARE * len(placed):
            return placed, folded, span
    assert first is not None
    return first


Notes = Sequence[Tuple[Optional[int], float]]


def from_rtttl(rtttl: str, melody_id: str, bpm: int, root: int, mode: str,
               register: Tuple[int, int] = kn.REGISTERS["lead"], theme_max: int = 0,
               theme_rtttl: Optional[str] = None) -> Tuple[Hook, Key]:
    """Хук и тональность трека из RTTTL. ``root``/``mode`` — тональность плана; лад трека берётся у темы.
    ``theme_max`` > 0 — ещё и тема целиком (:func:`with_theme`): вся мелодия до ``theme_max`` тактов; ``theme_rtttl``
    — запись того же произведения, откуда брать тему (:func:`theme_version`: ринг-тон хука бывает одной фразой), её
    ноты — в темпе трека (свой :func:`time_scale`) и с первой нотой на высоте первой ноты хука.

    Разбор строки — здесь, всё остальное — :func:`from_notes` (общий путь с материалом партитуры, ADR-0154 §3.3).
    """
    try:
        _name, melody_bpm, notes = parse_rtttl(rtttl)
    except ValueError as exc:
        _LOG.info("🎵 [music v2] hook melody=%s отказ: RTTTL не разбирается: %s", melody_id, exc)
        raise HookError(f"RTTTL не разбирается: {exc}") from exc
    theme = None
    if theme_max > 0 and theme_rtttl and theme_rtttl != rtttl:
        _n, theme_bpm, theme_notes = parse_rtttl(theme_rtttl)  # разбирается: выбрана theme_version
        ratio = time_scale(theme_bpm, bpm) / time_scale(melody_bpm, bpm)
        first = next(m for m, _d in notes if m is not None)
        offset = (first - next(m for m, _d in theme_notes if m is not None) + 6) % 12 - 6
        theme = ([(None if m is None else m + offset, d * ratio) for m, d in theme_notes], ())
    return from_notes(notes, melody_bpm, melody_id, bpm, root, mode, register, theme_max=theme_max, theme=theme)


def contour_match(steps: Sequence[int], ref: Sequence[int]) -> float:
    """Доля совпавших интервалов контура ``steps`` с эталоном ``ref`` (по длине эталона; интервалы —
    транспозиционно-инвариантно)."""
    return sum(x == y for x, y in zip(steps, ref)) / len(ref) if ref else 0.0


def theme_version(melody_id: str, melodies: Mapping[str, str], bpm: int) -> str:
    """Запись темы целиком для хука ``melody_id`` (#3542): самая длинная в тактах трека (темп ``bpm``, свой
    :func:`time_scale`) из ``melodies`` того же произведения — начало совпадает с началом хука контуром
    (``knowledge.THEME_REF_NOTES`` нот) не меньше ``knowledge.THEME_REF_MATCH_MIN``, как эталон у материала
    (:func:`for_theme`). Ринг-тон бывает одной фразой (``hallofth_2`` — 4 такта = хук, тема не длиннее хука), а
    другая запись той же темы (``hallofth``) — вся мелодия. Длиннее хука нет — сам ``melody_id``."""
    ref = _ref_contour(melodies[melody_id])

    def length(name: str) -> float:
        try:
            _n, melody_bpm, notes = parse_rtttl(melodies[name])
        except ValueError:
            return -1.0
        return sum(d for _m, d in notes) * time_scale(melody_bpm, bpm)

    same = [n for n, r in melodies.items() if n == melody_id or (
        ref is not None and (c := _ref_contour(r)) is not None
        and contour_match(c, ref) >= kn.THEME_REF_MATCH_MIN)]
    best = max(same, key=lambda n: (length(n), n == melody_id))
    return best if length(best) > length(melody_id) else melody_id


def from_notes(notes: Notes, melody_bpm: float, melody_id: str, bpm: int, root: int, mode: str,
               register: Tuple[int, int] = kn.REGISTERS["lead"], known_key: Optional[Key] = None,
               theme_max: int = 0, theme: Optional[Tuple[Notes, Sequence[float]]] = None,
               bars: bool = False) -> Tuple[Hook, Key]:
    """Хук и тональность трека из нот ``(MIDI | None для паузы, длительность в четвертях)`` мелодии в темпе
    ``melody_bpm`` — единственный путь получения хука из нот для любого источника (RTTTL, партитура).

    ``known_key`` — тональность источника, если она известна (партитура, ADR-0154 Н2); иначе ``detect_key`` по
    нотам окна. Исход в логе (I12): принятый хук — с ``key_fit``, отказ — с причиной.

    ``theme_max`` > 0 — тема целиком (ADR-0154 PR-7, :func:`with_theme`): ``theme`` — ноты темы и концы её фраз в
    тактах клуба от начала темы (материал: тематическая секция; пусто — фразы по длине хука, другая запись RTTTL,
    :func:`from_rtttl`); без ``theme`` — вся мелодия ``notes``, фразы по длине хука. Тема не годится — хук без темы,
    причина в логе.
    """
    try:
        hook, key, shift, answered = _hook(notes, melody_bpm, melody_id, bpm, root, mode, register, known_key, bars)
    except HookError as exc:
        _LOG.info("🎵 [music v2] hook melody=%s отказ: %s", melody_id, exc)
        raise
    _LOG.info("🎵 [music v2] hook melody=%s key=%s %s key_fit=%.2f bars=%d", melody_id, kn.ROOTS[key.root], key.mode,
              hook.key_fit, hook.bars)
    if theme_max <= 0 or (answered and theme is None):
        return hook, key  # короткая мелодия RTTTL с ответом — тема и есть хук
    onsets = _onsets(theme[0] if theme else notes, time_scale(melody_bpm, bpm), bars)
    cuts = (theme[1] if theme else ()) or _even_cuts(onsets, hook.bars)
    return with_theme(hook, key, onsets, cuts, theme_max, shift, register), key


def _even_cuts(onsets: Sequence[Tuple[float, float, int]], unit: int) -> List[float]:
    """Концы «фраз» мелодии без разметки: каждые ``unit`` тактов (длина хука) и конец мелодии."""
    end = max(b + d for b, d, _m in onsets) / BEATS_PER_BAR
    return [float(c) for c in range(unit, int(math.ceil(end - 1e-9)), unit)] + [end]


def _bars4(cut: float) -> int:
    """Тактов секции под тему до ``cut``: целые такты, кратно 4 (секции клуба кратны 4)."""
    return int(math.ceil(cut / 4 - 1e-9)) * 4


def with_theme(hook: Hook, key: Key, onsets: Sequence[Tuple[float, float, int]], cuts: Sequence[float],
               theme_max: int, shift: int, register: Tuple[int, int]) -> Hook:
    """Хук с темой целиком (ADR-0154 PR-7): ноты ``onsets`` (доли клуба от первой ноты, уже в темпе трека) до
    последнего конца фразы ``cuts`` (такты клуба), для которого секция (:func:`_bars4`) не длиннее ``theme_max``;
    перенос в тонику трека — ``shift``, коридор — от низа хука до верха ``register`` (пэд под лидом не теряет места,
    как у ответа). Октава — своя у каждой фразы (:func:`_placement` по фразе): тема, которая в оригинале
    поднимается на октаву («Горный король»), в коридор лида целиком не помещается, а фраза — помещается.

    Тема не длиннее хука, вне лада (``HOOK_KEY_FIT_MIN``), ломается переносом (:data:`MAX_FOLDED_SHARE`) или не
    мотив — хук без темы; причина в логе (I12)."""
    fit = [c for c in cuts if _bars4(c) <= theme_max]
    try:
        if not fit or _bars4(max(fit)) <= hook.bars:
            raise HookError(f"тема не длиннее хука ({hook.bars} тактов) в потолке {theme_max}")
        cut = max(fit) * BEATS_PER_BAR
        notes = [(b, min(d, cut - b), m) for b, d, m in onsets if b < cut - 1e-9]
        ends = [c * BEATS_PER_BAR for c in sorted(fit)]
        events = _theme_events(notes, ends, key, shift, (min(e.midi for e in hook.notes), register[1]))
    except HookError as exc:
        _LOG.info("🎵 [music v2] hook melody=%s темы нет: %s", hook.source, exc)
        return hook
    bars = _bars4(max(fit))
    _LOG.info("🎵 [music v2] hook melody=%s тема целиком: %d тактов, %d нот", hook.source, bars, len(events))
    return replace(hook, theme=events, theme_bars=bars)


def _theme_events(notes: List[Tuple[float, float, int]], ends: Sequence[float], key: Key, shift: int,
                  register: Tuple[int, int]) -> Tuple[PitchEvent, ...]:
    """Ноты темы в тонике трека и коридоре ``register``, октава — по фразе (концы ``ends``, доли); вне лада или
    ломается переносом — :class:`HookError`."""
    fit = key_fit([(m + shift, d) for _b, d, m in notes], kn.ROOTS[key.root], key.mode)
    if fit < kn.HOOK_KEY_FIT_MIN:
        raise HookError(f"тема вне лада: key_fit {fit:.2f} < {kn.HOOK_KEY_FIT_MIN}")
    placed, folded, _span = fit_register([(b, m) for b, _d, m in notes], ends, shift, register)
    if folded > MAX_FOLDED_SHARE * len(placed):
        raise HookError(f"тема ломается: {folded} из {len(placed)} нот перенесены октавой")
    moved = [(b, d, m) for (b, d, _m), m in zip(notes, placed)]
    _check_musical(moved)
    return tuple(PitchEvent(m, b, d, 3 if b % BEATS_PER_BAR == 0 else 2) for b, d, m in moved)


def _hook(notes: Notes, melody_bpm: float, melody_id: str, bpm: int, root: int, mode: str,
          register: Tuple[int, int], known_key: Optional[Key] = None,
          material: bool = False) -> Tuple[Hook, Key, int, bool]:
    bars, cut, answered = _window(_onsets(notes, time_scale(melody_bpm, bpm), material))
    _check_musical(cut)
    pitches = [m for _b, _d, m in cut]
    tonic, hook_mode = ((kn.ROOTS[known_key.root], known_key.mode) if known_key
                        else detect_key(pitches, [d for _b, d, _m in cut]))
    key = track_key(kn.ROOTS.index(tonic), hook_mode, root, mode)
    shift = (key.root - kn.ROOTS.index(tonic) + 6) % 12 - 6
    fit = key_fit([(m + shift, d) for _b, d, m in cut], kn.ROOTS[key.root], key.mode)
    if fit < kn.HOOK_KEY_FIT_MIN:
        raise HookError(f"тема вне лада {kn.ROOTS[key.root]} {key.mode}: key_fit {fit:.2f} < {kn.HOOK_KEY_FIT_MIN}")
    placed, folded, span = fit_register([(b, m) for b, _d, m in cut], [bars * BEATS_PER_BAR], shift, register)
    if folded > MAX_FOLDED_SHARE * len(placed):
        raise HookError(f"мотив ломается: {folded} из {len(placed)} нот не помещаются в коридор {register} "
                        f"без переноса октавой ни целиком, ни по {', '.join(map(str, OCTAVE_SPANS))} такта")
    if span:
        _LOG.info("🎵 [music v2] hook melody=%s октава по %d такта: перенесено %d из %d нот", melody_id, span, folded,
                  len(placed))
    moved = [(b, d, m) for (b, d, _m), m in zip(cut, placed)]
    _check_musical(moved)
    if answered:
        moved += _answer(moved, key, register)
    events = tuple(PitchEvent(m, b, d, 3 if b % BEATS_PER_BAR == 0 else 2) for b, d, m in moved)
    return Hook(events, bars, melody_id, fit), key, shift, answered


def _loop(notes: Sequence[PitchEvent], period_beats: float, length_beats: float) -> List[PitchEvent]:
    out: List[PitchEvent] = []
    start = 0.0
    while start < length_beats:
        out += [PitchEvent(e.midi, start + e.beat, min(e.dur_beats, length_beats - start - e.beat), e.accent)
                for e in notes if start + e.beat < length_beats]
        start += period_beats
    return out


def _build(hook: Hook, bars: int, key: Key, register: Tuple[int, int]) -> List[PitchEvent]:
    span = BUILD_BARS * BEATS_PER_BAR
    head = [PitchEvent(e.midi, e.beat, min(e.dur_beats, span - e.beat), e.accent) for e in hook.notes if e.beat < span]
    return _loop(head, span, (bars - 1) * BEATS_PER_BAR)  # последний такт — пауза перед дропом


def _break(hook: Hook, bars: int, key: Key, register: Tuple[int, int]) -> List[PitchEvent]:
    half = bars * BEATS_PER_BAR / 2
    slow = [PitchEvent(e.midi, e.beat * 2, e.dur_beats * 2, max(1, e.accent - 1)) for e in hook.notes if e.beat < half]
    return _loop(slow, bars * BEATS_PER_BAR, bars * BEATS_PER_BAR)


def _drop(hook: Hook, bars: int, key: Key, register: Tuple[int, int]) -> List[PitchEvent]:
    return _loop(hook.notes, hook.bars * BEATS_PER_BAR, bars * BEATS_PER_BAR)


def _drop2(hook: Hook, bars: int, key: Key, register: Tuple[int, int]) -> List[PitchEvent]:
    """Хук в параллельных терциях лада: терция сверху, у верхнего края коридора — снизу (не ниже хука)."""
    base = _drop(hook, bars, key, register)
    floor = min(e.midi for e in hook.notes)
    voices = []
    for e in base:
        up, down = diatonic(e.midi, key, 2), diatonic(e.midi, key, -2)
        second = up if up <= register[1] else (down if down >= floor else None)
        if second is not None:
            voices.append(PitchEvent(second, e.beat, e.dur_beats, e.accent))
    return base + voices


def _theme(hook: Hook, bars: int, key: Key, register: Tuple[int, int]) -> List[PitchEvent]:
    """Тема целиком (ADR-0154 PR-7), остаток секции — хук по кругу."""
    span = hook.theme_bars * BEATS_PER_BAR
    rest = _loop(hook.notes, hook.bars * BEATS_PER_BAR, bars * BEATS_PER_BAR - span)
    return list(hook.theme) + [PitchEvent(e.midi, span + e.beat, e.dur_beats, e.accent) for e in rest]


def _rhythm(hook: Hook, bars: int, key: Key, register: Tuple[int, int]) -> List[PitchEvent]:
    """Ритм хука с контуром следующей фразы материала (``Hook.answer``, Н10: самый частый приём корпуса, 31 %)."""
    return _loop(hook.answer, hook.bars * BEATS_PER_BAR, bars * BEATS_PER_BAR)


#: Развитие хука по имени секции; ``build2``/``break2`` форм ``long64`` — то же развитие, что у ``build``/``break``.
#: ``rhythm`` — не секция, а вариант секций :data:`RHYTHM_SECTIONS` у хука с ответом материала.
DEVELOPMENT: Dict[str, object] = {"build": _build, "build2": _build, "drop": _drop, "break": _break,
                                  "break2": _break, "drop2": _drop2, "rhythm": _rhythm, "theme": _theme}
#: Секции, где хук с ответом (``Hook.answer``) звучит развитием ``rhythm`` вместо своего (ADR-0154 §3.3).
RHYTHM_SECTIONS: Tuple[str, ...] = ("drop2",)


def develop(hook: Hook, section: str, bars: int, key: Key, register: Tuple[int, int] = kn.REGISTERS["lead"]
            ) -> Tuple[PitchEvent, ...]:
    """Ноты мотива в секции ``section`` длиной ``bars`` тактов, доли от начала секции. Хук с темой
    (``Hook.theme``) в ``knowledge.THEME_SECTION`` не короче темы — развитие ``theme``. Секция песенной формы стиля
    (куплет, припев, бридж — ADR-0153 S5) развивается как секция клуба своего вида (``knowledge.section_kind``)."""
    section = kn.section_kind(section)
    if hook.theme and section == kn.THEME_SECTION and bars >= hook.theme_bars:
        section = "theme"
    op = DEVELOPMENT.get("rhythm" if hook.answer and section in RHYTHM_SECTIONS else section)
    if op is None:
        return ()
    return tuple(sorted(op(hook, bars, key, register), key=lambda e: (e.beat, e.midi)))  # type: ignore[operator]


# ── Хук из материала партитуры (ADR-0154 §3.3) ────────────────────────────────────────────────────────────────

#: Имена текстовых секций, где «живёт» тема: фраза оттуда предпочтительнее при любых повторах (Н12).
THEME_SECTIONS: Tuple[str, ...] = ("theme", "main")


def _in_theme_section(material: ScoreMaterial, phrase: Phrase) -> bool:
    return any(s.origin == "text" and s.name.strip().lower() in THEME_SECTIONS
               and s.bar <= phrase.bar < s.bar + s.bars for s in material.sections)


def pick_phrase(material: ScoreMaterial, anchor: Optional[int] = None) -> Phrase:
    """Фраза-хук. ``anchor`` — такт, где начинается главный мотив по RTTTL-эталону (:func:`for_theme`): фраза
    материала оттуда длиной из :data:`HOOK_BARS`, иначе ``HOOK_BARS[0]`` тактов от него. Без эталона — первое
    проведение (отзыв Шифу 07.10: самая повторяемая фраза ≠ главный мотив, «это не совсем горный король»): фраза
    длиной из :data:`HOOK_BARS` в секции темы, затем самая ранняя. Нет подходящих фраз — начало мелодии, как у RTTTL."""
    fit = [p for p in material.phrases if p.bars in HOOK_BARS]
    if anchor is not None:
        return next((p for p in fit if p.bar == anchor), Phrase(anchor, HOOK_BARS[0], "new", 1))
    if not fit:
        return Phrase(0, HOOK_BARS[0], "new", 1)
    return min(fit, key=lambda p: (not _in_theme_section(material, p), p.bar))


def voices(material: ScoreMaterial) -> Dict[str, Tuple[PitchEvent, ...]]:
    """Голоса материала, где может быть тема: мелодия (skyline) и бас (низший голос, по ноте на онсет) — тема бывает
    в низах (Григ: фаготы и виолончели). Внутренних голосов в ``ScoreMaterial`` нет (импортёр их не пишет)."""
    bass: Dict[float, PitchEvent] = {}
    for e in material.bass:
        if e.beat not in bass or e.midi < bass[e.beat].midi:
            bass[e.beat] = e
    return {"melody": material.melody, "bass": tuple(bass[b] for b in sorted(bass))}


@functools.lru_cache(maxsize=256)
def _ref_contour(rtttl: str) -> Optional[Tuple[int, ...]]:
    """Контур начала RTTTL (``knowledge.THEME_REF_NOTES`` нот); строки эталонов одни на весь сет — разбор один раз."""
    return contour(rtttl, kn.THEME_REF_NOTES)


def reference_contours(rtttls: Sequence[str]) -> List[Tuple[int, ...]]:
    """Контуры начала RTTTL-эталонов произведения (:func:`rtttl.contour`, ``knowledge.THEME_REF_NOTES`` нот)."""
    out = [_ref_contour(r) for r in rtttls]
    return [c for c in out if c is not None]


def _best_window(steps: Sequence[int], at: Mapping[int, Sequence[int]],
                 ref: Sequence[int]) -> Optional[Tuple[int, int]]:
    """``(совпавших интервалов, начало)`` лучшего окна ``ref`` в ``steps`` (ничья — раньше): то же, что
    :func:`contour_match` по каждому окну, но счёт идёт от мест каждого интервала ``at`` — план отбирает материалы
    на пути запроса (M7). Окон нет (голос короче эталона) — ``None``."""
    last = len(steps) - len(ref)
    if last < 0:
        return None
    hits = [0] * (last + 1)
    for k, step in enumerate(ref):
        for j in at.get(step, ()):
            if 0 <= j - k <= last:
                hits[j - k] += 1
    top = max(hits)
    return top, hits.index(top)


def theme_match(material: ScoreMaterial, refs: Sequence[Tuple[int, ...]]) -> Optional[Tuple[float, str, float]]:
    """Лучшее совпадение контура эталона в голосах материала: ``(доля совпавших интервалов, голос, доля начала)``;
    транспозиционно-инвариантно (:func:`contour_match`), ничья — мелодия, раньше в пьесе. Нет эталонов — ``None``."""
    best: Optional[Tuple[float, str, float]] = None
    for name, events in voices(material).items():
        pitches = [e.midi for e in events]
        steps = [b - a for a, b in zip(pitches, pitches[1:])]
        at: Dict[int, List[int]] = {}
        for j, step in enumerate(steps):
            at.setdefault(step, []).append(j)
        for ref in refs:
            window = _best_window(steps, at, ref)
            if window is None:
                continue
            score = window[0] / len(ref)
            if best is None or score > best[0] or (score == best[0] and name == "melody" != best[1]):
                best = (score, name, events[window[1]].beat)
    return best


def for_theme(material: ScoreMaterial, rtttls: Sequence[str]) -> Tuple[ScoreMaterial, Optional[int]]:
    """Материал и такт главного мотива по RTTTL-эталонам того же произведения (общий механизм, не под одну пьесу):
    голос и место, где контур эталона совпал не меньше ``knowledge.THEME_REF_MATCH_MIN``; тема в басу — бас
    становится мелодией материала (хук, тема, гармония и бас трека берут одно отображение). Эталонов нет — материал
    как есть, ``None`` (первое проведение, :func:`pick_phrase`). Эталон есть, а совпадения мало — главного мотива в
    голосах материала нет (тема во внутреннем голосе): :class:`HookError`, трек берёт RTTTL-хук темы, а не
    неузнаваемую фразу. Исход — в лог."""
    match = theme_match(material, reference_contours(rtttls))
    if match is None:
        return material, None
    score, voice, beat = match
    bar = int(beat // _meter_bar(material))
    ok = score >= kn.THEME_REF_MATCH_MIN
    _LOG.info("🎵 [music v2] material=%s эталон RTTTL: голос %s такт %d совпадение %.2f — %s", material.material_id,
              voice, bar, score, "главный мотив оттуда" if ok else "мало — материал не узнаётся")
    if not ok:
        raise HookError(f"главного мотива эталона нет в голосах материала: совпадение {score:.2f} < "
                        f"{kn.THEME_REF_MATCH_MIN}")
    return (replace(material, melody=voices(material)[voice]) if voice != "melody" else material), bar


def _meter_bar(material: ScoreMaterial) -> float:
    return material.meter[0] * 4 / material.meter[1]


def _meter(material: ScoreMaterial) -> MeterMap:
    """Перевод размера материала в 4/4 клуба в режиме ``knowledge.TRIPLE_METER_MODE`` (``material.meter_map``).
    Размер, который переводится только в других режимах, ждёт приёмки на слух (#3517); остальные (5/4, 6/4, 9/8 …)
    честно отвергаются (ADR-0154 §3.6)."""
    mm = meter_map(material.meter)
    if mm is not None:
        return mm
    name = f"{material.meter[0]}/{material.meter[1]}"
    if meter_pending(material.meter):
        raise HookError(f"размер {name}: перевод в 4/4 на приёмке (#3517)")
    raise HookError(f"размер {name} не переводится в 4/4 клуба (ADR-0154 §3.6)")


def material_scale(material: ScoreMaterial, bpm: int) -> float:
    """Множитель темпа материала в треке (:func:`time_scale`): темп материала — в долях клуба после перевода
    размера (растяжение 3/4 ×4/3 — музыка медленнее в долях клуба). Один на хук, гармонию и бас."""
    return time_scale((material.bpm or bpm) * _meter(material).tempo_ratio, bpm)


def _phrase_notes(material: ScoreMaterial, phrase: Phrase) -> List[Tuple[Optional[int], float]]:
    """Ноты фразы как ``(MIDI | None, длительность)`` в тактах 4/4 (перевод размера — :func:`_meter`); длительность
    не переходит за свой такт, тишина перевода (пауза 3/4) — паузой."""
    mm = _meter(material)
    first, last = phrase.bar * mm.bar, (phrase.bar + phrase.bars) * mm.bar
    placed = [mm.note(e.beat - first, e.dur_beats) + (e.midi,) for e in material.melody if first <= e.beat < last]
    out: List[Tuple[Optional[int], float]] = []
    cursor = 0.0
    for i, (start, dur, midi) in enumerate(placed):
        dur = min(dur, placed[i + 1][0] - start) if i + 1 < len(placed) else dur
        if start > cursor + 1e-9:
            out.append((None, start - cursor))
        out.append((midi, dur))
        cursor = start + dur
    return out


def rhythm_answer(material: ScoreMaterial, phrase: Phrase, hook: Hook, key: Key, bpm: int,
                  register: Tuple[int, int] = kn.REGISTERS["lead"]) -> Tuple[PitchEvent, ...]:
    """Ответ хуку (Н10 «rhythm»): ритм хука, высоты следующей фразы материала той же длины — нота, начавшаяся
    последней к доле каждой ноты хука; перенос в тонику трека и коридор — как у хука (:func:`_placement`), но не
    ниже низа хука (октавой выше), чтобы пэд под лидом не терял места.

    Нет следующей фразы, ответ вне лада трека (``HOOK_KEY_FIT_MIN``), ломается переносом
    (:data:`MAX_FOLDED_SHARE`), не встаёт не ниже хука, не мотив или совпал с хуком — ``()`` (секция звучит своим
    развитием); причина — в логе (I12)."""
    try:
        placed = _answer_pitches(material, phrase, hook, key, bpm, register)
    except HookError as exc:
        _LOG.info("🎵 [music v2] hook melody=%s ответ rhythm нет: %s", material.material_id, exc)
        return ()
    _LOG.info("🎵 [music v2] hook melody=%s ответ rhythm: %d нот", material.material_id, len(placed))
    return tuple(PitchEvent(m, e.beat, e.dur_beats, e.accent) for e, m in zip(hook.notes, placed))


def _contour(material: ScoreMaterial, phrase: Phrase, hook: Hook, bpm: int) -> List[int]:
    """Высоты следующей фразы материала на долях нот хука (нота, начавшаяся последней к доле)."""
    contour = _onsets(_phrase_notes(material, Phrase(phrase.bar + phrase.bars, phrase.bars, "new")),
                      material_scale(material, bpm), bars=True)
    if len(contour) < MIN_NOTES:
        raise HookError(f"в следующей фразе {len(contour)} нот — не мотив")
    starts = [b for b, _d, _m in contour]
    return [contour[max(0, bisect.bisect_right(starts, e.beat) - 1)][2] for e in hook.notes]


def _answer_pitches(material: ScoreMaterial, phrase: Phrase, hook: Hook, key: Key, bpm: int,
                    register: Tuple[int, int]) -> List[int]:
    pitches = _contour(material, phrase, hook, bpm)
    shift = (key.root - material.key.root + 6) % 12 - 6
    fit = key_fit([(m + shift, e.dur_beats) for m, e in zip(pitches, hook.notes)], kn.ROOTS[key.root], key.mode)
    if fit < kn.HOOK_KEY_FIT_MIN:
        raise HookError(f"ответ вне лада: key_fit {fit:.2f} < {kn.HOOK_KEY_FIT_MIN}")
    placed, folded = _placement(pitches, shift, register)
    if folded > MAX_FOLDED_SHARE * len(placed):
        raise HookError(f"ответ ломается: {folded} из {len(placed)} нот перенесены октавой")
    floor = min(e.midi for e in hook.notes)
    if min(placed) < floor:  # ниже хука — пэд под лидом потеряет место (compose: верх пэда от низа лида)
        placed = [m + 12 for m in placed]
        if max(placed) > register[1] or min(placed) < floor:
            raise HookError("ответ не встаёт в коридор не ниже хука")
    _check_musical([(e.beat, e.dur_beats, m) for e, m in zip(hook.notes, placed)])
    if placed == [e.midi for e in hook.notes]:
        raise HookError("следующая фраза — буквальный повтор")
    return placed


def theme_span(material: ScoreMaterial, phrase: Phrase) -> Phrase:
    """Тематическая секция материала (ADR-0154 PR-7) вокруг фразы-хука ``phrase``: секция партитуры, где лежит
    фраза (метка или повтор), иначе — от фразы до конца пьесы. Такты исходного размера."""
    sec = next((s for s in material.sections if s.bar <= phrase.bar < s.bar + s.bars), None)
    if sec is not None:
        return Phrase(sec.bar, sec.bars, "new")
    last = material.melody[-1]
    end = int(math.ceil((last.beat + last.dur_beats) / _meter(material).bar - 1e-9))
    return Phrase(phrase.bar, max(1, end - phrase.bar), "new")


def theme_cuts(material: ScoreMaterial, span: Phrase, bpm: int) -> List[float]:
    """Концы фраз темы в тактах клуба от начала ``span``: фразы материала подряд от начала секции (стоп на дыре);
    без разметки — каждые ``HOOK_BARS[-1]`` тактов материала. Такт материала — ``club × material_scale`` долей."""
    per_bar = _meter(material).club * material_scale(material, bpm) / BEATS_PER_BAR
    cuts: List[float] = []
    cursor = span.bar
    for p in material.phrases:
        if span.bar <= p.bar and p.bar + p.bars <= span.bar + span.bars:
            if p.bar != cursor:
                break
            cursor = p.bar + p.bars
            cuts.append((cursor - span.bar) * per_bar)
    step = HOOK_BARS[-1]
    return cuts or [c * per_bar for c in range(step, span.bars + step, step)]


def from_material(material: ScoreMaterial, bpm: int, root: int, mode: str,
                  register: Tuple[int, int] = kn.REGISTERS["lead"], theme_max: int = 0,
                  anchor: Optional[int] = None) -> Tuple[Hook, Key]:
    """Хук и тональность трека из материала партитуры: фраза по :func:`pick_phrase`, тональность — материала
    (Н2), остальное — общий путь :func:`from_notes`. ``Hook.source`` — ``material_id``; ``Hook.answer`` —
    :func:`rhythm_answer` (развитие ``rhythm`` в :data:`RHYTHM_SECTIONS`). ``theme_max`` > 0 — ещё и тема целиком:
    тематическая секция (:func:`theme_span`) фраза за фразой (:func:`theme_cuts`) до ``theme_max`` тактов клуба."""
    validate_material(material)
    phrase = pick_phrase(material, anchor)
    club_bpm = (material.bpm or bpm) * _meter(material).tempo_ratio
    theme = None
    if theme_max > 0:
        span = theme_span(material, phrase)
        theme = (_phrase_notes(material, span), theme_cuts(material, span, bpm))
    hook, key = from_notes(_phrase_notes(material, phrase), club_bpm, material.material_id, bpm, root, mode, register,
                           material.key, theme_max, theme, bars=True)
    return replace(hook, answer=rhythm_answer(material, phrase, hook, key, bpm, register)), key


def material_hook(material: ScoreMaterial, rtttls: Sequence[str], bpm: int, root: int, mode: str,
                  register: Tuple[int, int] = kn.REGISTERS["lead"], theme_max: int = 0
                  ) -> Tuple[ScoreMaterial, Optional[int], Hook, Key]:
    """``(материал, такт главного мотива, хук, тональность)`` трека из материала с RTTTL-эталонами темы ``rtttls``:
    главный мотив по эталону (:func:`for_theme`), затем хук (:func:`from_material`). Один путь у ``compose`` и у
    годности материала в плане (:func:`material_unfit`); не годится — :class:`HookError`/``MaterialError``."""
    material, anchor = for_theme(material, rtttls)
    hook, key = from_material(material, bpm, root, mode, register, theme_max, anchor)
    return material, anchor, hook, key


def material_unfit(material: ScoreMaterial, bpm: int, root: int, mode: str,
                   register: Tuple[int, int] = kn.REGISTERS["lead"], rtttls: Sequence[str] = ()) -> Optional[str]:
    """Причина, по которой материал не годится в хук трека (``None`` — годится): тот же :func:`material_hook` с теми
    же ``bpm/root/mode/register`` и эталонами темы ``rtttls``, что у ``compose`` — главный мотив по эталону, мотив
    (нот/высот), ``key_fit``, размер (§3.6), коридор (:func:`fit_register`); второго критерия нет (#3500, #3542).
    Годность зависит от темпа и тоники трека, поэтому план спрашивает её для каждого трека
    (``set_plan.plan_materials``): негодный материал уступает трек следующему годному, а не RTTTL-хуку."""
    try:
        material_hook(material, rtttls, bpm, root, mode, register)
    except (HookError, MaterialError) as exc:
        return str(exc)
    return None


__all__ = ["DEVELOPMENT", "HOOK_BARS", "HookError", "MAX_FOLDED_SHARE", "OCTAVE_SPANS", "RHYTHM_SECTIONS",
           "contour_match", "develop", "diatonic", "fit_register", "for_theme", "from_material", "from_notes",
           "from_rtttl", "material_hook", "material_scale", "material_unfit", "pick_phrase", "reference_contours",
           "rhythm_answer", "theme_cuts", "theme_match", "theme_span", "theme_version", "time_scale", "track_key",
           "voices", "with_theme"]
