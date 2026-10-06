"""Хук из RTTTL: начало мелодии (4–8 тактов) в тональности трека и его развитие по секциям (ADR-0149 §3.3).

Берутся **первые** такты мелодии (не случайное 2-тактовое окно старого ``club_fragments``): по ним мелодию и
узнают. Ритм ринг-тона подгоняется к темпу трека степенью двойки (чтобы «быстрая» тема не стала вдвое
медленнее), ноты встают на сетку 16-х. Тональность темы — :func:`rob_box_music.tonality.detect_key`; тема
переносится на тонику трека целиком (интервалы сохраняются), лад трека — лад темы (#3293: явная тональность
транспонирует тему, а не только аккомпанемент). Хроматика темы — проходящие: ``key_fit`` (доля длительности в
ладу трека) ≥ ``knowledge.HOOK_KEY_FIT_MIN`` (I12). Регистр клампится в коридор лида: тема встаёт целиком на
октаву с наименьшим числом нот вне коридора, оставшиеся переносятся октавой — ближе к соседней ноте (контур). Тема
чужого лада (``key_fit`` ниже порога) или мотив, который перенос ломает (больше :data:`MAX_FOLDED_SHARE` нот),
честно отвергается :class:`HookError` — вызывающий берёт следующую мелодию.

Развитие (:func:`develop`) — детерминированные операции над мотивом по имени секции: ``build`` — первые
2 такта мотива и пауза перед дропом; ``drop`` — мотив целиком; ``break`` — начало мотива вдвое медленнее
(увеличение); ``drop2`` — мотив в параллельных терциях лада; остальные секции — без хука.
"""

from __future__ import annotations

import logging
import math
from typing import Dict, List, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..material import Phrase, ScoreMaterial, bar_beats, club_beat, validate_material
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Hook, Key, PitchEvent
from ..rtttl import parse_rtttl
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


def _onsets(notes: Sequence[Tuple[Optional[int], float]], scale: float) -> List[Tuple[float, float, int]]:
    """Звучащие ноты ``(доля, длительность, MIDI)`` на сетке 16-х; начальные паузы срезаны."""
    out: List[Tuple[float, float, int]] = []
    t = 0.0
    start: Optional[float] = None
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
    """Ответ фразе: та же фраза на IV/V ступени (диатонический сдвиг — ноты остаются в ладу), через ``PHRASE_BARS``."""
    for steps in ANSWER_STEPS:
        moved = [(b + PHRASE_BARS * BEATS_PER_BAR, d, diatonic(m, key, steps)) for b, d, m in cut]
        if all(register[0] <= m <= register[1] for _b, _d, m in moved):
            return moved
    raise HookError("ответ фразе не помещается в коридор лида")


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


def _placement(pitches: Sequence[int], shift: int, register: Tuple[int, int]) -> Tuple[List[int], int]:
    """Тема в коридоре лида и число нот, перенесённых октавой.

    Тема целиком сдвигается на ``shift`` плюс октавы — там, где меньше всего нот вне коридора (при равенстве низ
    ≥ :data:`PREFERRED_LOW`, середина ближе к :data:`TARGET_CENTER`); оставшиеся вне коридора ноты переносятся
    октавой в коридор — на ту, что ближе к предыдущей ноте (контур мотива сохраняется где можно).
    """
    lo, hi = register
    center = (min(pitches) + max(pitches)) / 2

    def outside(s: int) -> int:
        return sum(1 for m in pitches if not lo <= m + s <= hi)

    best = min((shift + 12 * k for k in range(-6, 7)),
               key=lambda s: (outside(s), min(pitches) + s < PREFERRED_LOW, abs(center + s - TARGET_CENTER)))
    placed: List[int] = []
    for midi in pitches:
        m = midi + best
        if not lo <= m <= hi:
            near = placed[-1] if placed else TARGET_CENTER
            m = min((m + 12 * k for k in range(-9, 10) if lo <= m + 12 * k <= hi), key=lambda c: abs(c - near))
        placed.append(m)
    return placed, outside(best)


Notes = Sequence[Tuple[Optional[int], float]]


def from_rtttl(rtttl: str, melody_id: str, bpm: int, root: int, mode: str,
               register: Tuple[int, int] = kn.REGISTERS["lead"]) -> Tuple[Hook, Key]:
    """Хук и тональность трека из RTTTL. ``root``/``mode`` — тональность плана; лад трека берётся у темы.

    Разбор строки — здесь, всё остальное — :func:`from_notes` (общий путь с материалом партитуры, ADR-0154 §3.3).
    """
    try:
        _name, melody_bpm, notes = parse_rtttl(rtttl)
    except ValueError as exc:
        _LOG.info("🎵 [music v2] hook melody=%s отказ: RTTTL не разбирается: %s", melody_id, exc)
        raise HookError(f"RTTTL не разбирается: {exc}") from exc
    return from_notes(notes, melody_bpm, melody_id, bpm, root, mode, register)


def from_notes(notes: Notes, melody_bpm: float, melody_id: str, bpm: int, root: int, mode: str,
               register: Tuple[int, int] = kn.REGISTERS["lead"], known_key: Optional[Key] = None) -> Tuple[Hook, Key]:
    """Хук и тональность трека из нот ``(MIDI | None для паузы, длительность в четвертях)`` мелодии в темпе
    ``melody_bpm`` — единственный путь получения хука из нот для любого источника (RTTTL, партитура).

    ``known_key`` — тональность источника, если она известна (партитура, ADR-0154 Н2); иначе ``detect_key`` по
    нотам окна. Исход в логе (I12): принятый хук — с ``key_fit``, отказ — с причиной.
    """
    try:
        hook, key = _hook(notes, melody_bpm, melody_id, bpm, root, mode, register, known_key)
    except HookError as exc:
        _LOG.info("🎵 [music v2] hook melody=%s отказ: %s", melody_id, exc)
        raise
    _LOG.info("🎵 [music v2] hook melody=%s key=%s %s key_fit=%.2f bars=%d", melody_id, kn.ROOTS[key.root], key.mode,
              hook.key_fit, hook.bars)
    return hook, key


def _hook(notes: Notes, melody_bpm: float, melody_id: str, bpm: int, root: int, mode: str,
          register: Tuple[int, int], known_key: Optional[Key] = None) -> Tuple[Hook, Key]:
    bars, cut, answered = _window(_onsets(notes, time_scale(melody_bpm, bpm)))
    _check_musical(cut)
    pitches = [m for _b, _d, m in cut]
    tonic, hook_mode = ((kn.ROOTS[known_key.root], known_key.mode) if known_key
                        else detect_key(pitches, [d for _b, d, _m in cut]))
    key = track_key(kn.ROOTS.index(tonic), hook_mode, root, mode)
    shift = (key.root - kn.ROOTS.index(tonic) + 6) % 12 - 6
    fit = key_fit([(m + shift, d) for _b, d, m in cut], kn.ROOTS[key.root], key.mode)
    if fit < kn.HOOK_KEY_FIT_MIN:
        raise HookError(f"тема вне лада {kn.ROOTS[key.root]} {key.mode}: key_fit {fit:.2f} < {kn.HOOK_KEY_FIT_MIN}")
    placed, folded = _placement(pitches, shift, register)
    if folded > MAX_FOLDED_SHARE * len(placed):
        raise HookError(f"мотив ломается: {folded} из {len(placed)} нот не помещаются в коридор {register} "
                        f"без переноса октавой")
    moved = [(b, d, m) for (b, d, _m), m in zip(cut, placed)]
    _check_musical(moved)
    if answered:
        moved += _answer(moved, key, register)
    events = tuple(PitchEvent(m, b, d, 3 if b % BEATS_PER_BAR == 0 else 2) for b, d, m in moved)
    return Hook(events, bars, melody_id, fit), key


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


#: Развитие хука по имени секции; ``build2``/``break2`` форм ``long64`` — то же развитие, что у ``build``/``break``.
DEVELOPMENT: Dict[str, object] = {"build": _build, "build2": _build, "drop": _drop, "break": _break,
                                  "break2": _break, "drop2": _drop2}


def develop(hook: Hook, section: str, bars: int, key: Key, register: Tuple[int, int] = kn.REGISTERS["lead"]
            ) -> Tuple[PitchEvent, ...]:
    """Ноты мотива в секции ``section`` длиной ``bars`` тактов, доли от начала секции."""
    op = DEVELOPMENT.get(section)
    if op is None:
        return ()
    return tuple(sorted(op(hook, bars, key, register), key=lambda e: (e.beat, e.midi)))  # type: ignore[operator]


# ── Хук из материала партитуры (ADR-0154 §3.3) ────────────────────────────────────────────────────────────────

#: Имена текстовых секций, где «живёт» тема: фраза оттуда предпочтительнее при любых повторах (Н12).
THEME_SECTIONS: Tuple[str, ...] = ("theme", "main")


def _in_theme_section(material: ScoreMaterial, phrase: Phrase) -> bool:
    return any(s.origin == "text" and s.name.strip().lower() in THEME_SECTIONS
               and s.bar <= phrase.bar < s.bar + s.bars for s in material.sections)


def pick_phrase(material: ScoreMaterial) -> Phrase:
    """Фраза-хук: длиной из :data:`HOOK_BARS` в секции темы, затем с наибольшим ``repeats`` (Н9), ничья — раньше
    в пьесе. Нет подходящих фраз — начало мелодии, как у RTTTL (``HOOK_BARS[0]`` тактов)."""
    fit = [p for p in material.phrases if p.bars in HOOK_BARS]
    if not fit:
        return Phrase(0, HOOK_BARS[0], "new", 1)
    return min(fit, key=lambda p: (not _in_theme_section(material, p), -p.repeats, p.bar))


#: Размеры, которые хук переводит в 4/4 клуба: 4/4 и 2/2 как есть, 3/4 — паузой на 4-й доле (В5 (б)).
SUPPORTED_METERS: Tuple[Tuple[int, int], ...] = ((4, 4), (2, 2), (3, 4))


def _phrase_notes(material: ScoreMaterial, phrase: Phrase) -> List[Tuple[Optional[int], float]]:
    """Ноты фразы как ``(MIDI | None, длительность)`` в тактах 4/4; 3/4 дополняется паузой на 4-й доле, длительность
    не переходит за свой такт. Размер вне :data:`SUPPORTED_METERS` честно отвергается (ADR-0154 §3.6)."""
    if material.meter not in SUPPORTED_METERS:
        raise HookError(f"размер {material.meter[0]}/{material.meter[1]} не переводится в 4/4 клуба (ADR-0154 §3.6)")
    bar = bar_beats(material.meter)
    first, last = phrase.bar * bar, (phrase.bar + phrase.bars) * bar
    placed: List[Tuple[float, float, int]] = []  # (доля в 4/4, длительность до конца такта, MIDI)
    for e in material.melody:
        if first <= e.beat < last:
            offset = (e.beat - first) % bar
            placed.append((club_beat(material.meter, e.beat - first), min(e.dur_beats, bar - offset), e.midi))
    out: List[Tuple[Optional[int], float]] = []
    cursor = 0.0
    for i, (start, dur, midi) in enumerate(placed):
        dur = min(dur, placed[i + 1][0] - start) if i + 1 < len(placed) else dur
        if start > cursor + 1e-9:
            out.append((None, start - cursor))
        out.append((midi, dur))
        cursor = start + dur
    return out


def from_material(material: ScoreMaterial, bpm: int, root: int, mode: str,
                  register: Tuple[int, int] = kn.REGISTERS["lead"]) -> Tuple[Hook, Key]:
    """Хук и тональность трека из материала партитуры: фраза по :func:`pick_phrase`, тональность — материала
    (Н2), остальное — общий путь :func:`from_notes`. ``Hook.source`` — ``material_id``."""
    validate_material(material)
    notes = _phrase_notes(material, pick_phrase(material))
    return from_notes(notes, material.bpm or bpm, material.material_id, bpm, root, mode, register, material.key)


__all__ = ["DEVELOPMENT", "HOOK_BARS", "HookError", "MAX_FOLDED_SHARE", "develop", "diatonic", "from_material",
           "from_notes", "from_rtttl", "pick_phrase", "time_scale", "track_key"]
