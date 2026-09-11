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
from .rtttl import parse_rtttl

__all__ = ["RtttlMelody", "rtttl_to_melody", "detect_key", "melody_to_compose_params"]

#: Лады для детекции тональности, в порядке приоритета при равном счёте.
_KEY_SCALES = (
    "major",
    "minor",
    "harmonicMinor",
    "dorian",
    "mixolydian",
    "lydian",
    "phrygian",
)


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


def detect_key(midi_notes: Sequence[Optional[int]]) -> Tuple[str, str]:
    """Определить ``(тоника, лад)`` по набору абсолютных MIDI-нот.

    Скор каждой пары (тоника, лад) — сколько высотных классов мелодии
    лежит в ладу. Выбирается пара с максимумом покрытия; при равенстве —
    лад из :data:`_KEY_SCALES`, тоника по кругу от C. Паузы (``None``)
    игнорируются. Без нот — ``("C", "major")``.

    Тональность нужна не для самой темы (она играется абсолютным MIDI),
    а для баса и подклада, которые аранжировщик достраивает вокруг неё.
    """
    pcs = {m % 12 for m in midi_notes if m is not None}
    if not pcs:
        return "C", "major"
    best: Optional[Tuple[str, str]] = None
    best_score = -1
    for scale_name in _KEY_SCALES:
        in_scale = {i % 12 for i in SCALE_INTERVALS[scale_name]}
        for root_idx, root in enumerate(VALID_ROOTS):
            shifted = {(pc - root_idx) % 12 for pc in pcs}
            score = len(shifted & in_scale)
            if score > best_score:
                best_score = score
                best = (root, scale_name)
    return best if best is not None else ("C", "major")


def melody_to_compose_params(melody: RtttlMelody) -> Dict[str, object]:
    """RTTTL-мелодия → плоские параметры ``compose_music``.

    Возвращает dict с ключами:
      * ``bpm`` — темп из RTTTL (``compose_music.bpm``);
      * ``root`` / ``scale`` — определённая тональность (для баса/подклада);
      * ``lead_midi`` — строка абсолютных MIDI через запятую (``None`` = пауза);
      * ``lead_dur`` — ритм в битах, той же длины.

    Мелодия выравнивается по такту (хвостовая пауза доводит луп до целого
    числа тактов) — иначе луп плывёт относительно ударной сетки и тема
    звучит «не в тайминг». Синт мелодии (``lead_synth``) и аккомпанемент
    (drums/bass/pad/form) сюда НЕ входят — их даёт модель обычными
    параметрами ``compose_music``.
    """
    melody = _snap_to_bar(melody)
    root, scale = detect_key([m for m, _ in melody.notes])
    midi: List[str] = ["None" if m is None else str(int(m)) for m, _ in melody.notes]
    dur: List[str] = [f"{d:g}" for _, d in melody.notes]
    return {
        "bpm": melody.bpm,
        "root": root,
        "scale": scale,
        "lead_midi": ", ".join(midi),
        "lead_dur": ", ".join(dur),
    }


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
