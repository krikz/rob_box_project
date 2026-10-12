"""Инварианты сета H1–H6 по модели ``Track`` (ADR-0157 §2.6, §3.5, §5): числа вместо ушей судьи.

Офлайн за миллисекунды, как ``scripts/music/research/arranger_theory_audit.py``: трек и пара соседей сета (уходящий
A, входящий B) → метрики. Гард — сид-серия в CI (``test/test_set_invariants.py``), отчёт до/после —
``scripts/music/set_invariants_report.py``.

* **H1** пульс (рантайм — ``model.validate``, здесь — та же :func:`model.pulse_faults`): такт дропа без
  kick/clap/bass; пауза пульса вне брейка и окон блэнда > ``PULSE_GAP_MAX_BEATS``.
* **H2** окно без темы на стыке, доли (решение В5 — оставить 8 тактов: метрика отчётная, не гард).
* **H3** конфликт тональностей на стыке: в первых тактах блэнда B — авторский хроматический аккорд (тон вне лада B)
  с тоном вне лада A или нота темы вне лада A.
* **H4** фразировка лида: доля пауз в секциях с лидом и самая длинная цепочка 16-х подряд (решение В6 — порога нет,
  только печать).
* **H6** сустейн длиннее ``LONG_SUS_BEATS`` не у ``LONG_SUS_ROLES``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from . import knowledge as kn
from .model import BEATS_PER_BAR, Chord, Track, blend_bars, pulse_faults

STEP = 0.25  # шестнадцатая в долях


@dataclass(frozen=True)
class TrackInvariants:
    """Метрики одного трека: ``pulse`` — нарушения H1 (секция, причина), ``long_sus`` — роли нот H6."""

    pulse: Tuple[Tuple[int, str], ...]
    long_sus: Tuple[str, ...]
    lead_rest: Optional[float]  # H4: доля 16-х без звука лида в секциях с лидом; ``None`` — лида нет
    lead_run16: int  # H4: самая длинная цепочка 16-х подряд у лида


@dataclass(frozen=True)
class JunctionInvariants:
    """Метрики стыка A → B: ``theme_gap`` — доли без темы (H2), ``key_conflicts`` — что не сходится по ладу (H3)."""

    blend_bars: int
    theme_gap: Optional[float]
    key_conflicts: Tuple[str, ...]
    author_chords: int = 0  # аккордов с качеством в окне блэнда B — сколько H3 проверил


def _theme_notes(track: Track):
    return [ev for role in kn.THEME_ROLES if role in track.parts for ev in track.parts[role].pitches or ()]


def long_sustains(track: Track) -> Tuple[str, ...]:
    """H6: роль каждой ноты длиннее ``LONG_SUS_BEATS`` у роли не из ``LONG_SUS_ROLES``."""
    return tuple(role for role, part in sorted(track.parts.items()) if role not in kn.LONG_SUS_ROLES
                 for ev in part.pitches or () if ev.dur_beats > kn.LONG_SUS_BEATS)


def _lead_steps(track: Track) -> Tuple[int, int]:
    """(16-х в секциях с лидом, из них звучащих)."""
    covered = set()
    for ev in track.parts["lead"].pitches or ():
        first = int(round(ev.beat / STEP))
        covered.update(range(first, first + max(1, int(round(ev.dur_beats / STEP)))))
    total = sounding = start = 0
    for sec in track.form.sections:
        steps = sec.bars * BEATS_PER_BAR * 4
        if "lead" in sec.roles:
            total += steps
            sounding += sum(1 for s in range(start, start + steps) if s in covered)
        start += steps
    return total, sounding


def lead_phrasing(track: Track) -> Tuple[Optional[float], int]:
    """H4: (доля пауз лида в его секциях, самая длинная цепочка 16-х подряд) — печать, порога нет (В6)."""
    if "lead" not in track.parts:
        return None, 0
    total, sounding = _lead_steps(track)
    onsets = sorted({ev.beat for ev in track.parts["lead"].pitches or () if ev.dur_beats <= STEP + 1e-9})
    best = run = 0
    for i, beat in enumerate(onsets):
        run = run + 1 if i and abs(beat - onsets[i - 1] - STEP) < 1e-9 else 1
        best = max(best, run)
    return (round(1.0 - sounding / total, 3) if total else None), best


def track_invariants(track: Track) -> TrackInvariants:
    rest, run = lead_phrasing(track)
    return TrackInvariants(tuple(pulse_faults(track)), long_sustains(track), rest, run)


def theme_gap(leaving: Track, incoming: Track, bars: int) -> Optional[float]:
    """H2: доли от конца последней ноты темы A до первой ноты темы B на общей оси (B встаёт за ``bars`` тактов до
    конца формы A); ``None`` — у кого-то темы нет."""
    a, b = _theme_notes(leaving), _theme_notes(incoming)
    if not a or not b:
        return None
    entry = leaving.form.bars_total * BEATS_PER_BAR - bars * BEATS_PER_BAR
    return round(entry + min(ev.beat for ev in b) - max(ev.beat + ev.dur_beats for ev in a), 3)


def window_chords(incoming: Track, bars: int) -> List[Tuple[int, Chord]]:
    """Авторские аккорды (``Chord.quality``) первых ``bars`` тактов B: (такт, аккорд)."""
    out, bar = [], 0
    for sec in incoming.form.sections:
        out += [(bar + k, chord) for k, chord in enumerate(incoming.harmony.progression.get(sec.name, ())[:sec.bars])
                if bar + k < bars and chord.quality]
        bar += sec.bars
    return out


def key_conflicts(leaving: Track, incoming: Track, bars: int) -> Tuple[str, ...]:
    """H3: в первых ``bars`` тактах B (нахлёст с хвостом A) — авторский аккорд с тоном вне лада B и вне лада A, нота
    темы вне лада A. Соседи по кругу квинт (``set_plan``) — не конфликт: тоны лада B разрешены."""
    scale_a = set(kn.scale_pitch_classes(leaving.key.root, leaving.key.mode))
    scale_b = set(kn.scale_pitch_classes(incoming.key.root, incoming.key.mode))
    size = kn.STYLES[incoming.style].chord_size
    out: List[str] = []
    for bar, chord in window_chords(incoming, bars):
        tones = set(kn.chord_pitch_classes(incoming.key.root, incoming.key.mode, chord.degree, chord.quality, size))
        if tones - scale_b and tones - scale_a:
            out.append(f"такт {bar}: аккорд {chord.degree}{chord.quality} вне ладов A и B")
    window = bars * BEATS_PER_BAR
    out += [f"доля {ev.beat:g}: тема {ev.midi} вне лада A" for ev in _theme_notes(incoming)
            if ev.beat < window and ev.midi % 12 not in scale_a]
    return tuple(out)


def junction_invariants(leaving: Track, incoming: Track) -> JunctionInvariants:
    bars = blend_bars(leaving, incoming)
    return JunctionInvariants(bars, theme_gap(leaving, incoming, bars), key_conflicts(leaving, incoming, bars),
                              len(window_chords(incoming, bars)))


def ducked(track: Track) -> bool:
    """H7: дакинг трека слышен — есть роли под сайдчейном и глубина хоть одной секции > 0."""
    return bool(track.mix.duck_roles) and any(d.depth > 0 for d in track.mix.duck)


def _median(values: List[float]) -> float:
    ordered = sorted(values)
    return ordered[len(ordered) // 2] if ordered else float("nan")


def _faults(per: List[TrackInvariants], joints: List[JunctionInvariants]) -> Dict[str, float]:
    """Число нарушений гардов H1/H3/H6 и сколько аккордов окна проверил H3."""
    return {"h1": sum(len(p.pulse) for p in per), "h3": sum(len(j.key_conflicts) for j in joints),
            "h3_checked": sum(j.author_chords for j in joints), "h6": sum(len(p.long_sus) for p in per)}


def style_row(runs: List[List[Track]]) -> Dict[str, float]:
    """Сводка стиля по сетам (в сете треки подряд): число нарушений H1/H3/H6, H2 и H4 — медиана/максимум, H7 — треков
    с бочкой и из них под дакингом."""
    seq = [t for run in runs for t in run]
    per = [track_invariants(t) for t in seq]
    joints = [junction_invariants(a, b) for run in runs for a, b in zip(run, run[1:])]
    kicked = [t for t in seq if "kick" in t.parts]
    return {
        **_faults(per, joints), "tracks": len(seq),
        "h2_median": _median([j.theme_gap for j in joints if j.theme_gap is not None]),
        "h4_rest_median": _median([p.lead_rest for p in per if p.lead_rest is not None]),
        "h4_run16_max": max((p.lead_run16 for p in per), default=0),
        "joints": len(joints), "blends": sum(bool(j.blend_bars) for j in joints),
        "h7_ducked": sum(map(ducked, kicked)), "with_kick": len(kicked),
    }


def summary(sets: Dict[str, List[List[Track]]]) -> Dict[str, Dict[str, float]]:
    """Сводка по стилям: ``style → сеты`` → :func:`style_row`."""
    return {style: style_row(runs) for style, runs in sets.items()}


__all__ = ["JunctionInvariants", "TrackInvariants", "ducked", "junction_invariants", "key_conflicts", "lead_phrasing",
           "long_sustains", "style_row", "summary", "theme_gap", "track_invariants", "window_chords"]
