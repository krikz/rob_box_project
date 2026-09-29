"""classic_loudness.py — модель громкости classic и калибровка секций формы (issue #3154).

Живой замер 29.09 после #3171 (jack_rec, ``r8_elise.wav``): club-сет ровный
−30…−36 dB RMS, а classic «К Элизе» −42…−47 — на ~13 dB тише. Club
откалиброван моделью по слоям (:mod:`core.club_loudness`); у classic слоёв
по палитре нет — синты выбирает LLM из ~60, мелодия любая, поэтому модель
считает НОТЫ.

Модель
======

1. Программа аранжировщика исполняется в заглушках FoxDot
   (:mod:`core.renardo_events`) — получаются все ноты и удары одной формы:
   доля, синт, высота, ``sus``, ``amp·amplify``, фильтры.
2. Энергия ноты берётся из таблицы :mod:`core._classic_loudness_table`:
   синт × фильтр × ``sus`` × высота, плюс наклон по ``amp``
   (``20·p·log10(amp/0.5)``); энергия удара ``play()`` — символ × ``sample``.
3. Уровень секции формы = Σ энергий её нот / длительность секции (dB,
   та же шкала RMS, что у club: masterfilter gain 0.5).

Таблица — офлайн-рендер ``scripts/music/club_loudness_nrt.py
--sweep-notes/--sweep-drums`` (scsynth 3.14.1 NRT 16 кГц, настоящие
SynthDef'ы renardo_lib 0.9.13 + патчи образа, сэмплы ``0_foxdot_default``,
``masterfilter.scd``). **Это модель, не замер на роботе.** Не
моделируются: ``echo``/``room`` (добавляют хвост), лупы и FX-сэмплы
(``loop``), ``Clock.latency``/свинг; синт вне таблицы — нота без энергии.

Калибровка (:func:`section_gains`)
==================================

* **Основной блок** classic — секции с ударными (``drums`` в плане формы);
  без ударных (``ambient``) — самая громкая секция. Общий множитель ставит
  его на :data:`core.club_loudness.TARGET_MAIN_DB` — тот же уровень, что у
  club (общий множитель стиля, club против classic ±0).
* **Тихие секции** (intro, break, outro, gap…) поднимаются до «основной −
  :data:`SECTION_FLOOR_DB`» — всеми слоями секции одинаково (баланс внутри
  секции не меняется).
* Каждый ``amp`` ≤ :data:`MAX_AMP` (0.85, ``max_amp`` санитайзера): где
  слой упёрся в потолок, модель это учитывает и остаток не прячет.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from . import _classic_loudness_table as table
from .club_loudness import TARGET_MAIN_DB
from .renardo_events import NoteEvent, ProgramError, program_events

#: Тихая секция не ниже основного блока больше чем на столько dB (приёмка
#: #3154 «~8–10 dB»; нижняя граница — запас на ошибку модели).
SECTION_FLOOR_DB = 8.0
#: Потолок одного ``amp`` (``music_max_amp`` санитайзера по умолчанию).
MAX_AMP = 0.85
#: Санитайзер переименовывает эти синты перед исполнением (``renardo_sanitizer``).
SYNTH_ALIASES: Dict[str, str] = {"pianovel": "rhpiano", "piano": "rhpiano"}
#: hpf ниже этой частоты считается ``hpf261``, выше — ``hpf523`` (середина октавы).
HPF_SPLIT_HZ = 392.0
#: Статичный lpf до этой частоты — ``lpf523``; свип ``gflt`` — ``lpf2000``.
LPF_LOW_HZ = 1000.0
LPF_OPEN_HZ = 3000.0
_SILENCE = -200.0
_BISECT_STEPS = 50


# ---------------------------------------------------------------------------
# Энергия ноты / удара
# ---------------------------------------------------------------------------


def _interp(xs: Sequence[float], ys: Sequence[float], x: float) -> float:
    """Линейная интерполяция по сетке; за краями — продолжение крайнего отрезка."""
    i = min(max(bisect_right(xs, x) - 1, 0), len(xs) - 2)
    x0, x1, y0, y1 = xs[i], xs[i + 1], ys[i], ys[i + 1]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def _row_db(rows: Sequence[Sequence[float]], midi: float, sus_s: float) -> float:
    """dB энергии по сетке высот (линейно, с клампом) и ``sus`` (по log2 sus).

    За пределами сетки ``sus`` наклон ограничен 0…3 dB на октаву: энергия
    держащей ноты растёт как её длина, ударной — не растёт.
    """
    pitches = table.NOTE_PITCHES
    midi = min(max(midi, pitches[0]), pitches[-1])
    by_sus = [_interp(pitches, row, midi) for row in rows]
    logs = [math.log2(s) for s in table.NOTE_SUS_S]
    x = math.log2(max(sus_s, 1e-3))
    if logs[0] <= x <= logs[-1]:
        return _interp(logs, by_sus, x)
    edge, other, log_edge, log_other = (
        (by_sus[0], by_sus[1], logs[0], logs[1]) if x < logs[0] else (by_sus[-1], by_sus[-2], logs[-1], logs[-2])
    )
    slope = min(max((edge - other) / (log_edge - log_other), 0.0), 3.0)
    return edge + slope * (x - log_edge)


def _filter_conds(fx: Mapping[str, object]) -> Tuple[str, List[str]]:
    """Базовое условие таблицы + условия-поправки (``dB(cond) − dB(none)``)."""
    hpf = float(fx.get("hpf", 0) or 0)
    lpf = float(fx.get("lpf", 0) or 0)
    base = "none" if not hpf else ("hpf261" if hpf < HPF_SPLIT_HZ else "hpf523")
    deltas: List[str] = []
    if lpf and fx.get("lpf_sweep"):
        deltas.append("lpf2000")
    elif lpf and lpf <= LPF_LOW_HZ:
        deltas.append("lpf523")
    elif lpf and lpf < LPF_OPEN_HZ:
        deltas.append("lpf2000")
    if base == "none" and deltas:
        return deltas[0], deltas[1:]
    return base, deltas


def _conds(fx: Mapping[str, object]) -> List[str]:
    base, deltas = _filter_conds(fx)
    return [base, *deltas]


def note_db(synth: str, midi: float, sus_s: float, amp: float, fx: Mapping[str, object]) -> Optional[float]:
    """dB энергии ноты синта по наклону (без учёта обрыва выше :func:`safe_amp`); ``None`` — синта нет."""
    synth = SYNTH_ALIASES.get(synth, synth)
    rows = table.NOTE_DB.get(synth)
    if rows is None or amp <= 0:
        return None
    base, deltas = _filter_conds(fx)
    db = _row_db(rows[base], midi, sus_s)
    for cond in deltas:
        db += _row_db(rows[cond], midi, sus_s) - _row_db(rows["none"], midi, sus_s)
    return db + 20.0 * table.EXPONENT.get(synth, 1) * math.log10(amp / table.NOTE_AMP)


def safe_amp(synth: str, fx: Mapping[str, object]) -> float:
    """Наибольший amp, до которого синт за этими фильтрами звучит по наклону (≤ :data:`MAX_AMP`).

    Офлайн-рендер: у ambi за hpf энергия при amp ≥ 0.15 обрывается до −110 dB
    — поднять такой слой выше значит выключить его.
    """
    safe = table.SAFE_AMP.get(SYNTH_ALIASES.get(synth, synth))
    if safe is None:
        return MAX_AMP
    return min([MAX_AMP] + [float(safe[cond]) for cond in _conds(fx)])


def drum_db(sample: str, amp: float) -> Optional[float]:
    """dB энергии удара ``play()`` (ключ ``символ+sample``); ``None`` — символа нет."""
    symbol, index = sample[0], int(sample[1:] or 0)
    values = table.DRUM_DB.get(symbol)
    if values is None or amp <= 0:
        return None
    db = values[index] if index < len(values) else 10 * math.log10(sum(10 ** (v / 10) for v in values) / len(values))
    return db + 20.0 * math.log10(amp / table.NOTE_AMP)


def event_db(event: NoteEvent, beat_s: float) -> Optional[float]:
    if event.synth == "play":
        return drum_db(event.sample or "", event.amp)
    return note_db(event.synth, float(event.midi or 0), event.sus_beats * beat_s, event.amp, event.fx)


def _event_cap(event: NoteEvent) -> float:
    return MAX_AMP if event.synth == "play" else safe_amp(event.synth, event.fx)


def _exponent(event: NoteEvent) -> float:
    if event.synth == "play":
        return 1.0
    return float(table.EXPONENT.get(SYNTH_ALIASES.get(event.synth, event.synth), 1))


# ---------------------------------------------------------------------------
# Секции формы
# ---------------------------------------------------------------------------

Key = Tuple[str, float]


@dataclass
class SectionModel:
    """Секция формы: по слою (слот, наклон) — мощность по наклону, гейт, потолок."""

    seconds: float
    power: Dict[Key, float] = field(default_factory=dict)
    actual: Dict[Key, float] = field(default_factory=dict)
    gate: Dict[Key, float] = field(default_factory=dict)
    cap: Dict[Key, float] = field(default_factory=dict)

    def add(self, key: Key, energy: float, gate: float, cap: float, amp: float) -> None:
        self.power[key] = self.power.get(key, 0.0) + energy
        if amp <= cap + 1e-9:
            self.actual[key] = self.actual.get(key, 0.0) + energy
        self.gate[key] = max(self.gate.get(key, 0.0), gate)
        self.cap[key] = min(self.cap.get(key, MAX_AMP), cap)

    def _db(self, total: float) -> float:
        return 10 * math.log10(total / self.seconds) if total > 0 else _SILENCE

    def actual_db(self) -> float:
        """Как звучит сейчас: ноты выше безопасного amp оборваны."""
        return self._db(sum(self.actual.values()))

    def level_db(self, gain: float = 1.0) -> float:
        """С множителем ``gain`` и потолком слоя ``min(gain·gate, cap)`` (как применит аранжировщик)."""
        total = 0.0
        for key, power in self.power.items():
            gate = self.gate[key]
            effective = min(gain, self.cap[key] / gate) if gate > 0 else gain
            total += power * effective ** (2 * key[1])
        return self._db(total)

    def full_cap_gain(self) -> float:
        """Множитель, при котором все слои секции на потолке (дальше громче не станет)."""
        gains = [self.cap[k] / g for k, g in self.gate.items() if g > 0]
        return max(gains) if gains else 1.0


@dataclass
class FormModel:
    sections: List[SectionModel]
    modeled: int
    unmodeled: int

    def slot_caps(self) -> Dict[str, float]:
        """Потолок amp по слоту плеера (минимум по секциям)."""
        caps: Dict[str, float] = {}
        for section in self.sections:
            for (slot, _p), cap in section.cap.items():
                caps[slot] = min(caps.get(slot, MAX_AMP), cap)
        return caps


def sounding_s(synth: str, sus_s: float) -> float:
    """Сколько секунд нота звучит (95 % энергии) — по ``TAIL_S`` таблицы, log-log по ``sus``.

    Держащие синты (moogbass, mhpad, supersawlead …) звучат до ``sus·8`` —
    пока ``makeSound`` renardo не освободит узел; колокола — фиксированный
    хвост; ударные — короче ``sus``. Синта нет в таблице — ``sus``.
    """
    tail = table.TAIL_S.get(SYNTH_ALIASES.get(synth, synth))
    if tail is None or min(tail) <= 0:
        return sus_s
    (s0, s1), (t0, t1) = (0.5, 2.0), tail
    slope = math.log(t1 / t0) / math.log(s1 / s0)
    return t0 * (max(sus_s, 1e-3) / s0) ** slope


def _spread(bounds: Sequence[float], start: float, length: float) -> List[Tuple[int, float]]:
    """Доли энергии ноты по секциям: ноту длиной ``length`` долей от ``start`` (с переходом через конец формы)."""
    total = bounds[-1]
    if length <= 0:
        return [(min(bisect_right(bounds, start) - 1, len(bounds) - 2), 1.0)]
    parts: List[Tuple[int, float]] = []
    left, cursor = min(length, total), start % total
    while left > 1e-9:
        index = min(bisect_right(bounds, cursor) - 1, len(bounds) - 2)
        take = min(left, bounds[index + 1] - cursor)
        parts.append((index, take / min(length, total)))
        left -= take
        cursor = (cursor + take) % total
    return parts


def form_model(code: str, bounds_beats: Sequence[float], bpm: float) -> FormModel:
    """Секции формы по программе: ``bounds_beats`` — границы секций в долях (0, …, F).

    Энергия ноты делится между секциями поровну по времени звучания
    (:func:`sounding_s`: у держащих синтов до ``sus·8``, через конец формы —
    в её начало, форма зациклена). Перетёкшая энергия масштабируется
    множителем секции, КУДА перетекла (приближение).

    Raises:
        ProgramError: программа не разворачивается в события.
    """
    beat_s = 60.0 / float(bpm)
    sections = [SectionModel(seconds=(b - a) * beat_s) for a, b in zip(bounds_beats, bounds_beats[1:])]
    _program, events = program_events(code, form_beats=bounds_beats[-1])
    modeled = unmodeled = 0
    for event in events:
        db = event_db(event, beat_s)
        if db is None:
            unmodeled += 1
            continue
        modeled += 1
        length = 0.0 if event.synth == "play" else sounding_s(event.synth, event.sus_beats * beat_s) / beat_s
        key, energy, cap = (event.slot, _exponent(event)), 10 ** (db / 10), _event_cap(event)
        for index, share in _spread(bounds_beats, event.beat, length):
            sections[index].add(key, energy * share, event.gate, cap, event.amp)
    return FormModel(sections, modeled, unmodeled)


def _mean(sections: Sequence[SectionModel], level) -> float:
    seconds = sum(s.seconds for s in sections)
    power = sum(10 ** (level(s) / 10) * s.seconds for s in sections if level(s) > _SILENCE)
    return 10 * math.log10(power / seconds) if power > 0 else _SILENCE


def _solve_gain(fn, target: float, lo: float, hi: float) -> float:
    """Наименьший множитель в [lo, hi], где монотонная ``fn(g) >= target`` (иначе hi)."""
    if fn(hi) < target:
        return hi
    for _ in range(_BISECT_STEPS):
        mid = math.sqrt(lo * hi)
        if fn(mid) >= target:
            hi = mid
        else:
            lo = mid
    return hi


@dataclass
class Calibration:
    """Итог калибровки: множители ``amp`` по секциям, потолки по слотам, уровни до/после."""

    gains: List[float]
    slot_caps: Dict[str, float]
    main_sections: List[int]
    before_db: List[float]
    after_db: List[float]
    main_before_db: float
    main_after_db: float
    unmodeled_events: int


def calibrate(model: FormModel, main_sections: Sequence[int],
              target_db: float = TARGET_MAIN_DB, floor_db: float = SECTION_FLOOR_DB) -> Optional[Calibration]:
    """Множители ``amp`` по секциям: основной блок на ``target_db``, тихие не ниже ``target − floor``.

    ``None`` — модели нечего сказать (ни одной ноты из таблицы).
    """
    sections = model.sections
    if model.modeled == 0:
        return None
    before = [round(s.actual_db(), 1) for s in sections]
    main = [i for i in main_sections if sections[i].power] or [max(range(len(sections)), key=lambda i: before[i])]
    main_secs = [sections[i] for i in main]
    main_gain = _solve_gain(lambda g: _mean(main_secs, lambda s: s.level_db(g)), target_db, 1e-3,
                            max(s.full_cap_gain() for s in main_secs))
    floor = target_db - floor_db
    gains: List[float] = []
    for section in sections:
        gain = main_gain
        if section.power and section.level_db(gain) < floor:
            top = max(main_gain, section.full_cap_gain())
            gain = _solve_gain(section.level_db, floor, main_gain, top)
        gains.append(gain)
    return Calibration(
        gains=gains, slot_caps=model.slot_caps(), main_sections=main, before_db=before,
        after_db=[round(s.level_db(g), 1) for s, g in zip(sections, gains)],
        main_before_db=round(_mean(main_secs, SectionModel.actual_db), 1),
        main_after_db=round(_mean(main_secs, lambda s: s.level_db(main_gain)), 1),
        unmodeled_events=model.unmodeled,
    )


def section_gains(code: str, bounds_beats: Sequence[float], bpm: float,
                  main_sections: Sequence[int]) -> Optional[Calibration]:
    """Калибровка по программе; ``None`` — программа не моделируется (без калибровки).

    Модель смотрит на код ПОСЛЕ санитайзера — его играет робот. Это не
    мелочь: санитайзер капает числа внутри ``amplify=var(...)`` вместе с
    длительностями (0.875 → 0.85), цикл дакинга баса становится 3.9 доли
    вместо 4 и уезжает от бочки — в офлайн-рендере бас звучит на ~4 dB
    громче, чем по исходному коду аранжировщика.
    """
    try:
        model = form_model(robot_code(code), bounds_beats, bpm)
    except ProgramError:
        return None
    return calibrate(model, main_sections)


def robot_code(code: str) -> str:
    """Код, как его исполнит робот: через ``sanitize_renando`` (без каталога синтов)."""
    from .renardo_sanitizer import sanitize_renando

    result = sanitize_renando(code, MAX_AMP, pack1_loops_enabled=True)
    return result.code or code


__all__ = [
    "Calibration",
    "FormModel",
    "MAX_AMP",
    "SECTION_FLOOR_DB",
    "SectionModel",
    "calibrate",
    "drum_db",
    "event_db",
    "form_model",
    "note_db",
    "safe_amp",
    "sounding_s",
    "robot_code",
    "section_gains",
]
