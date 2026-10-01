"""club_timbre.py — тембр лида/пэда club от модели под тему (issue #3268).

Живой лог 01.10 (азиатский и славянский сеты): модель передавала в
``compose_music(style="club")`` ``lead_synth="brass"``, ``pad_synth="strings"``,
а club отвечал «Проигнорировано: lead_synth, pad_synth» и играл тембр из
пула сида (``club_arranger.ROLE_SYNTHS``) — тема на звук не влияла.

Здесь:

* :data:`TIMBRE_EXTRAS` — тембры, которые модель может выбрать ЯВНО поверх
  пула сида. Сид из них не выбирает (разнообразие ADR-0146 без темы —
  прежнее). Критерии — те же, что у пула (``club_arranger.ROLE_SYNTHS``):
  синт из ``CRITICAL_SYNTHS`` (грузится на роботе), есть в таблице
  классик-громкости (:mod:`core._classic_loudness_table`), не ``held``
  (:mod:`core.synth_traits`), у лида хвост короткой ноты не длиннее
  :data:`LEAD_TAIL_MAX_S` — иначе 16-е арпеджио слипаются; у пэда остаток
  просадки тихих секций по модели громкости не хуже самого тихого пэда
  пула (sinepad, ≤ :data:`PAD_DROP_MAX_DB`) — :data:`PAD_TOO_QUIET`;
* :func:`timbre_request` — что из ``lead_synth``/``pad_synth`` вызова
  применить, а что нет и ПОЧЕМУ (причина идёт в ответ тула, тембр тогда —
  из пула, трек не падает);
* :func:`estimated_unit_db` — громкость нового тембра для модели
  :mod:`core.club_loudness`. NRT-замера club для этих синтов нет;
  оценка — энергия ноты из таблицы классик-громкости в условиях club-слоя
  плюс поправка, снятая на синтах пула, которые замерены и там, и там
  (медиана; расхождения на пуле — в :data:`ESTIMATE_NOTE`). Это МОДЕЛЬ,
  не замер на роботе.

Модуль чистый, без ROS/Renardo.
"""

from __future__ import annotations

from functools import lru_cache
from statistics import median
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from . import _classic_loudness_table as table
from .classic_loudness import note_db
from .club_loudness import _MEASURED_DB
from .synth_traits import traits_of

__all__ = [
    "ESTIMATE_NOTE",
    "LEAD_TAIL_MAX_S",
    "PAD_DROP_MAX_DB",
    "PAD_TOO_QUIET",
    "TIMBRE_EXTRAS",
    "estimated_unit_db",
    "rejection_reason",
    "timbre_refusals",
    "timbre_request",
    "timbre_sentence",
]

#: Тембры поверх пула сида (роль → синты). Порядок — как в подсказке модели.
TIMBRE_EXTRAS: Dict[str, Tuple[str, ...]] = {
    "lead": ("sitar", "epiano", "brass", "orient", "viola"),
    "pad": ("ambi", "strangerpulsepad"),
}

#: Остаток просадки тихих секций (dB к основному блоку, модель
#: ``club_loudness.kit_report``), который допускает пул: sinepad ~10.5.
PAD_DROP_MAX_DB = 11.0
#: Пэды из ``CRITICAL_SYNTHS``, которые модель просит под тему, но в club
#: они тише порога: худший остаток по шаблонам club (пересчитывается тестом).
PAD_TOO_QUIET: Dict[str, float] = {"strings": 13.2, "pads": 19.3}

#: Хвост короткой ноты (``TAIL_S[синт][0]``, с) самого длинного синта пула
#: лида — marimba 0.31 — плюс запас. Длиннее — ноты 16-х накладываются.
LEAD_TAIL_MAX_S = 0.35

#: Параметр вызова → роль club.
_PARAM_ROLES: Tuple[Tuple[str, str], ...] = (("lead_synth", "lead"), ("pad_synth", "pad"))

#: Условия club-слоя для оценки по таблице: (MIDI середины регистра, длина
#: ноты в долях при 124 BPM, фильтры как в ``render_club_kit``).
_LANE_NOTE: Dict[str, Tuple[float, float, Mapping[str, object]]] = {
    "lead": (71.0, 0.15, {"lpf": 2000, "lpf_sweep": True}),  # LEAD_LOW..LEAD_TOP_LIMIT, sus=0.15, lpf linvar
    "pad": (59.0, 8.0, {}),  # PAD_LOW..68, аккорд на 2 такта
}
_REF_BPM = 124.0

ESTIMATE_NOTE = (
    "оценка по таблице классик-громкости (NRT 16 кГц, amp=0.1) + медианная поправка по синтам пула, "
    "замеренным в club (29.09); расхождение на пуле: лид ≤ 4.1 dB (karp), пэд ≤ 0.4 dB; не замер на роботе"
)


def rejection_reason(role: str, synth: str, palette: Sequence[str]) -> Optional[str]:
    """Почему ``synth`` не годится для роли club (``None`` — годится)."""
    if synth in palette:
        return None
    traits = traits_of(synth)
    if traits is not None and traits.tail == "held":
        return f"{synth} держит ноту (held), в club слои идут 16-ми и аккордами — ноты слипнутся"
    if role == "pad" and synth in PAD_TOO_QUIET:
        return (
            f"{synth} в club слишком тихий пэд: тихие секции без бочки просели бы на {PAD_TOO_QUIET[synth]:.0f} dB "
            f"(допустимо ≤ {PAD_DROP_MAX_DB:.0f})"
        )
    tail = table.TAIL_S.get(synth)
    if role == "lead" and tail is not None and tail[0] > LEAD_TAIL_MAX_S:
        return f"хвост {synth} {tail[0]:.2f} с длиннее 16-й — арпеджио club слипнется"
    return f"{synth} нет в палитре club для роли {role}"


def timbre_request(
    kwargs: Mapping[str, Any], palette: Mapping[str, Sequence[str]],
) -> Tuple[Dict[str, str], List[str]]:
    """Тембры вызова для club: ``(роль → синт, причины отказа)``.

    Пустое/``none`` — не задано (тембр выбирает сид). Отказ не ломает трек:
    роль играет тембр из пула, а причина идёт в ответ тула.
    """
    chosen: Dict[str, str] = {}
    refused: List[str] = []
    for param, role in _PARAM_ROLES:
        synth = str(kwargs.get(param) or "").strip().lower()
        if not synth or synth == "none":
            continue
        reason = rejection_reason(role, synth, palette[role])
        if reason is None:
            chosen[role] = synth
        else:
            refused.append(
                f"{param}={synth!r} не применён: {reason}; играет тембр из пула "
                f"(допустимо: {', '.join(palette[role])})."
            )
    return chosen, refused


def _table_db(lane: str, synth: str, amp: float) -> float:
    midi, beats, fx = _LANE_NOTE[lane]
    db = note_db(synth, midi, beats * 60.0 / _REF_BPM, amp, fx)
    if db is None:
        raise ValueError(f"Синта {synth!r} нет в таблице классик-громкости — оценить громкость в club нечем")
    return db


@lru_cache(maxsize=64)
def estimated_unit_db(lane: str, synth: str) -> Tuple[float, float]:
    """``(dB при amp-гейте 1.0, показатель amp)`` синта вне замера club (оценка).

    Поправка «таблица → club» — медиана по синтам пула, замеренным в club
    (``club_loudness._MEASURED_DB``) при их уровне замера.

    Raises:
        ValueError: синта нет в таблице классик-громкости или роль без мелодии.
    """
    if lane not in _LANE_NOTE:
        raise ValueError(f"Слой {lane!r}: оценки громкости по таблице нет")
    level, measured = _MEASURED_DB[lane]
    offset = median(db - _table_db(lane, anchor, level) for anchor, db in measured.items())
    exponent = float(table.EXPONENT.get(synth, 1))
    return round(_table_db(lane, synth, 1.0) + offset, 2), exponent


def timbre_refusals(club_timbre: Optional[Mapping[str, Any]]) -> List[str]:
    """Причины отказа в тембрах вызова — для предупреждения в начале ответа тула."""
    return list((club_timbre or {}).get("refused") or ())


def timbre_sentence(club_timbre: Optional[Mapping[str, Any]]) -> str:
    """Фраза ответа тула о применённых тембрах из вызова (``""`` — не задавались)."""
    applied = (club_timbre or {}).get("applied") or {}
    if not applied:
        return ""
    roles = ", ".join(f"{role} {synth}" for role, synth in applied.items())
    return f" Тембр из вызова: {roles} (громкость слоя — по модели, оценка по классик-таблице)."
