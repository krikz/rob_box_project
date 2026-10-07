"""Пэд: фигуры аккомпанемента — генераторы ``(style, key, bar_chords, synth, register) -> Part`` (ADR-0153 §2.2).

Рисунки — ``knowledge.PAD_FIGURES`` (ADR-0152 §3.2), генератор выбирается по ключу из реестра
``arrange.compose.PAD_GENERATORS``:

* ``pumped16`` — аккорд на каждой 16-й (``sus`` — шаг): сайдчейн-огибающая рендера живёт только на событиях, поэтому
  пэд под ней звучит каждой 16-й (ADR-0149 §3.6, §3.8).
* ``held`` — аккорд держится до смены (аккорд петли — 2 такта, ``sus`` 8 долей), без сайдчейна: в ~30 раз меньше
  событий, синты с собственным хвостом (``warmpad``, ``strangerpulsepad``) звучат как задуманы.
* ``stabs`` — аккорд на «и» каждой доли под сайдчейном, звучит до следующего «и» (четверть): короче — окно тишины
  ≥ 50 мс там, где пэд единственный тональный слой (интро и хвост блэнда, A4/I8 ``test_no_silent_window``); отрезок
  пэда начинается подхватом на первой доле.
* ``arp`` — арпеджио (ADR-0153 S2): один тон аккорда такта на каждой 16-й, порядок голосов — ``Style.arp_order``;
  без сайдчейна. Тоны — голоса того же обращения, что держал бы пэд: в регистре и в ладу по построению.
* ``comping`` — comping (ADR-0153 S4): 2–3 коротких аккорда на такт по ритмам ``Style.comp_rhythms`` (такт за тактом по
  кругу), септаккорды при ``Style.chord_size`` 4; на сильных долях 1 и 3 — никогда (там хук), без сайдчейна.

Обращения аккордов — ``harmony.pad_chords``; уровень ставит ``arrange.mix``.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, Part, PitchEvent
from . import rhythm

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
#: «И» каждой доли (оффбит-восьмые); стэб звучит до следующего «и».
STAB_STEPS = (2, 6, 10, 14)
STAB_SUS_STEPS = 4


def pumped16(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
             register: Tuple[int, int]) -> Part:
    """Аккорд такта на каждой 16-й по тактам формы ``bar_chords`` — (такт, аккорд)."""
    events = tuple(
        PitchEvent(m, bar * BEATS_PER_BAR + step * STEP_BEATS, STEP_BEATS, 3)
        for bar, chord in bar_chords for step in range(STEPS_PER_BAR) for m in chord.voicing
    )
    return Part("pad", synth, rhythm.grid(range(STEPS_PER_BAR)), events, 0.0, register)


def stabs(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
          register: Tuple[int, int]) -> Part:
    """Аккорд такта на «и» каждой доли, до следующего «и»; в начале отрезка, где пэд звучит, — подхват на первой доле
    (до «и»), в конце — последний стэб до конца такта: без окна тишины на стыке формы и за её пределы."""
    bars = {bar for bar, _chord in bar_chords}
    events = []
    for bar, chord in bar_chords:
        hits = [(0, STAB_STEPS[0])] if bar - 1 not in bars else []
        hits += [(step, min(STAB_SUS_STEPS, STEPS_PER_BAR - step) if bar + 1 not in bars else STAB_SUS_STEPS)
                 for step in STAB_STEPS]
        events += [PitchEvent(m, bar * BEATS_PER_BAR + step * STEP_BEATS, sus * STEP_BEATS, 3)
                   for step, sus in hits for m in chord.voicing]
    return Part("pad", synth, rhythm.grid(STAB_STEPS), tuple(events), 0.0, register)


def _runs(bar_chords: Sequence[Tuple[int, Chord]]) -> List[Tuple[int, int, Chord]]:
    """(первый такт, тактов, аккорд): подряд идущие такты с одним аккордом — одна нота."""
    runs: List[List] = []
    for bar, chord in bar_chords:
        if runs and runs[-1][0] + runs[-1][1] == bar and runs[-1][2] == chord:
            runs[-1][1] += 1
        else:
            runs.append([bar, 1, chord])
    return [(b, n, c) for b, n, c in runs]


def held(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
         register: Tuple[int, int]) -> Part:
    """Аккорд держится, пока он не сменится и пэд звучит (``sus`` = длина отрезка)."""
    events = tuple(PitchEvent(m, bar * BEATS_PER_BAR, bars * BEATS_PER_BAR, 3)
                   for bar, bars, chord in _runs(bar_chords) for m in chord.voicing)
    return Part("pad", synth, rhythm.grid((0,)), events, 0.0, register)


def arp(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
        register: Tuple[int, int]) -> Part:
    """Арпеджио по тактам ``bar_chords``: на 16-й ``k`` такта звучит голос ``style.arp_order[k % len]`` аккорда
    (голоса снизу вверх; индекс за числом голосов — по кругу), до следующей 16-й; первая 16-я доли — акцент 3."""
    spans = [(bar * BEATS_PER_BAR, float(BEATS_PER_BAR), chord.voicing) for bar, chord in bar_chords]
    return Part("pad", synth, rhythm.grid(range(STEPS_PER_BAR)), arp_events(style.arp_order, spans), 0.0, register)


def arp_events(order: Sequence[int], spans: Sequence[Tuple[float, float, Tuple[int, ...]]]) -> Tuple[PitchEvent, ...]:
    """Арпеджио по отрезкам ``(доля начала, долей, обращение)``: на 16-й ``k`` такта — голос ``order[k % len]``
    обращения (снизу вверх, по кругу), до следующей 16-й; первая 16-я доли — акцент 3. Отрезок — такт (``arp``) или
    аккорд автора любой длины на сетке 16-х (песня из материала, ADR-0154 PR-7B)."""
    events = []
    for start, beats, voicing in spans:
        tones = sorted(voicing)
        for k in range(int(round(beats / STEP_BEATS))):
            beat = start + k * STEP_BEATS
            step = int(round(beat / STEP_BEATS)) % STEPS_PER_BAR
            events.append(PitchEvent(tones[order[step % len(order)] % len(tones)], beat, STEP_BEATS,
                                     3 if step % 4 == 0 else 2))
    return tuple(events)


#: Сильные доли такта (шаги 16-х), где comping не бьёт: там звучит хук (комплементарность, ADR-0149 §3.4).
STRONG_STEPS = (0, 8)


def comping(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
            register: Tuple[int, int]) -> Part:
    """Comping по тактам ``bar_chords``: в такте ``bar`` — удары ритма ``style.comp_rhythms[bar % n]``, аккорд такта
    звучит свою длину (короткий); удар на сильной доле в таблице — ``ValueError`` (таблица врёт)."""
    rhythms = style.comp_rhythms
    if not rhythms or any(step in STRONG_STEPS for r in rhythms for step, _n in r):
        raise ValueError(f"ритмы comping {rhythms!r}: пусто или удар на сильной доле {STRONG_STEPS}")
    events = [PitchEvent(m, bar * BEATS_PER_BAR + step * STEP_BEATS, length * STEP_BEATS, 3 if step % 4 == 0 else 2)
              for bar, chord in bar_chords for step, length in rhythms[bar % len(rhythms)] for m in chord.voicing]
    steps = sorted({step for r in rhythms for step, _n in r})
    return Part("pad", synth, rhythm.grid(steps), tuple(events), 0.0, register)


__all__ = ["STAB_STEPS", "STRONG_STEPS", "arp", "arp_events", "comping", "held", "pumped16", "stabs"]
