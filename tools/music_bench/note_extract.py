"""``CompositionSpec`` -> плоский список нотных событий (issue #2977).

Не парсит сгенерированный Renardo-код: читает те же структуры, что
``core.arranger.render()`` превращает в текст (:class:`~core.arranger.
CompositionSpec`, :class:`~core.arranger.Layer`, план формы из
``resolve_form``) и переиспользует ЕГО ЖЕ функции огибающей громкости
(``_amp_envelope``, ``_section_intensity``) — тот же расчёт, что и в
реальном рендере, а не отдельная копия, которая может разойтись при
следующей правке аранжировщика.

Что намеренно упрощено (см. ``tools/music_bench/README`` в docstring
пакета и ``docs/music_bench.md``):

* Стенд извлекает ноты только для слоёв, где ``Layer.midi``/``Layer.durs``
  заданы абсолютно — это путь известной темы (``core.harmonize.
  Harmonization``: бас/пэд/лид/контрголос уже абсолютный MIDI + биты).
  Именно этим путём идёт вся выборка архива (:mod:`tools.music_bench.
  sampler` собирает её из RTTTL-библиотеки). Путь «сочинено с нуля»
  (ступени лада, ``Layer.degrees``) в эту версию стенда не включён —
  это самостоятельное расширение, а не часть плана P3 (там речь идёт
  именно про выборку архива).
* ``Pvar``-трансформации мотива лида (транспозиция/инверсия/ретроград,
  см. ``arranger._motif_variants``) здесь НЕ применяются: паттерн темы
  просто циклически повторяется на всей форме, огибающая громкости и
  плотности по секциям — применяется. На спектральный центроид и на сумму
  слоёв (наши целевые метрики) это не влияет — обе метрики держатся на
  регистре и громкости роли, не на порядке нот внутри него; RC5.1 в
  аудите прямо об этом говорит (транспозиция/инверсия сворачиваются
  обратно в тот же регистр).
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from . import _repo_paths  # noqa: F401  (sys.path побочный эффект)

from rob_box_mcp_tools.core.arranger import (  # noqa: E402
    BEATS_PER_BAR,
    COUNTER_OF_LEAD,
    DRUM_ROLES,
    FIXED_THEME_AMP_FLOOR,
    ROLE_PROFILE,
    CompositionSpec,
    Layer,
    _amp_envelope,
    _is_blank_drum_pattern,
    resolve_form,
)

#: Роль ударных -> (низ, верх) полосы шумового «удара» в Гц. Оценка на
#: глаз по типичному тембру роли (бочка/хэт/перкуссия), не измерение
#: конкретных сэмплов — сэмплов у стенда нет (см. docs/music_bench.md).
DRUM_NOISE_BAND_HZ = {
    "drums": (50.0, 180.0),
    "hats": (4000.0, 9500.0),
    "perc": (700.0, 3200.0),
}

#: Роль -> архетип волны для :mod:`tools.music_bench.synth`.
ROLE_WAVEFORM = {
    "bass": "sine",
    "pad": "soft",
    "lead": "saw",
    "counter": "triangle",
}


@dataclass(frozen=True)
class NoteEvent:
    """Одно событие: тон (мелодическая роль) или удар (ударная роль).

    Времена — в БИТАХ от начала формы (как в ``arranger`` — секунды
    считает вызывающий код через ``bpm``).
    """

    role: str
    start_beat: float
    dur_beat: float
    amp: float
    freqs_hz: Tuple[float, ...] = ()  # пусто -> шумовой удар
    noise_band_hz: Optional[Tuple[float, float]] = None


def midi_to_freq(midi: float) -> float:
    """Стандартная MIDI-шкала A4=69=440 Гц."""
    return 440.0 * (2.0 ** ((float(midi) - 69.0) / 12.0))


def _note_to_freqs(note: object) -> Tuple[float, ...]:
    if note is None:
        return ()
    if isinstance(note, (tuple, list)):
        return tuple(midi_to_freq(n) for n in note if n is not None)
    return (midi_to_freq(note),)


def _envelope_amp_at(amps: Sequence[float], durs: Sequence[int], t: float) -> float:
    """Значение time-varying огибающей в момент ``t`` (в битах)."""
    if not amps:
        return 0.0
    boundaries = []
    acc = 0.0
    for d in durs:
        acc += float(d)
        boundaries.append(acc)
    idx = bisect.bisect_right(boundaries, t)
    if idx >= len(amps):
        idx = len(amps) - 1
    return amps[idx]


def _tile(sequence: Sequence[Tuple[object, float]], total_beats: float) -> List[Tuple[float, float, object]]:
    """Зациклить ``(значение, длительность_бит)`` на весь ``total_beats``.

    Тот же приём, что и у реального плеера Renardo: паттерн (нота ИЛИ
    символ ударного шага) короче формы просто повторяется, независимо от
    границ секций формы — секции модулируют только громкость (огибающая
    считается отдельно, см. :func:`_envelope_amp_at`).
    """
    cycle = sum(float(d) for _v, d in sequence)
    if cycle <= 0 or not sequence:
        return []
    out: List[Tuple[float, float, object]] = []
    t = 0.0
    while t < total_beats - 1e-9:
        for value, dur in sequence:
            if t >= total_beats - 1e-9:
                break
            remaining = total_beats - t
            out.append((t, min(float(dur), remaining), value))
            t += float(dur)
    return out


def _drum_layer_events(
    layer: Layer,
    plan: Sequence[Tuple[str, int, dict]],
    base_amp: float,
) -> List[NoteEvent]:
    if not layer.pattern or _is_blank_drum_pattern(layer.pattern):
        return []
    amps, durs = _amp_envelope(layer.role, plan, base_amp)
    total_beats = sum(int(b) for _n, b, _i in plan) * BEATS_PER_BAR
    if not any(amps) or total_beats <= 0:
        return []
    n = len(layer.pattern)
    slot = BEATS_PER_BAR / n
    one_bar = [(ch, slot) for ch in layer.pattern]
    band = DRUM_NOISE_BAND_HZ.get(layer.role, (200.0, 4000.0))
    events: List[NoteEvent] = []
    for start, dur, ch in _tile(one_bar, total_beats):
        if ch in ". ":
            continue
        amp = _envelope_amp_at(amps, durs, start)
        if amp <= 0:
            continue
        # Удар короче слота сетки — иначе соседние хэты сливаются в шум.
        hit_dur = min(dur, 0.2)
        events.append(NoteEvent(
            role=layer.role, start_beat=start, dur_beat=hit_dur, amp=amp,
            freqs_hz=(), noise_band_hz=band,
        ))
    return events


def _melodic_layer_events(
    layer: Layer,
    plan: Sequence[Tuple[str, int, dict]],
    base_amp: float,
) -> List[NoteEvent]:
    if layer.midi is None or not layer.durs:
        return []
    total_beats = sum(int(b) for _n, b, _i in plan) * BEATS_PER_BAR
    if total_beats <= 0:
        return []
    floor = 0.0
    if layer.role in ("lead", "counter"):
        floor = FIXED_THEME_AMP_FLOOR
        if layer.role == "counter":
            floor *= COUNTER_OF_LEAD
    amps, durs = _amp_envelope(layer.role, plan, base_amp, floor=floor)
    if not any(amps):
        return []
    sequence = list(zip(layer.midi, (float(d) for d in layer.durs)))
    note_dur_cap = layer.sus  # пэд: остинато короче шага сетки
    events: List[NoteEvent] = []
    for start, dur, note in _tile(sequence, total_beats):
        freqs = _note_to_freqs(note)
        if not freqs:
            continue  # пауза
        amp = _envelope_amp_at(amps, durs, start)
        if amp <= 0:
            continue
        sounding = dur if note_dur_cap is None else min(dur, float(note_dur_cap))
        events.append(NoteEvent(
            role=layer.role, start_beat=start, dur_beat=sounding, amp=amp,
            freqs_hz=freqs, noise_band_hz=None,
        ))
    return events


def extract_events(spec: CompositionSpec) -> Tuple[List[NoteEvent], float]:
    """Спецификация -> (события, длительность формы в битах).

    Слои без ролей в :data:`~core.arranger.ROLE_PROFILE` (``loop``/``fx``)
    в этой версии стенда пропускаются — образцы из архива их не заводят
    (см. docstring модуля).
    """
    plan = resolve_form(spec.form, getattr(spec, "theme_bars", 0))
    total_beats = float(sum(int(b) for _n, b, _i in plan) * BEATS_PER_BAR)
    levels = dict(getattr(spec, "levels", None) or {})
    events: List[NoteEvent] = []
    for layer in spec.layers:
        profile = ROLE_PROFILE.get(layer.role)
        if profile is None:
            continue  # loop/fx — вне области этой версии стенда
        _player, _oct, base_amp = profile
        base_amp = base_amp * levels.get(layer.role, 1.0)
        if layer.role in DRUM_ROLES:
            events.extend(_drum_layer_events(layer, plan, base_amp))
        else:
            events.extend(_melodic_layer_events(layer, plan, base_amp))
    return events, total_beats
