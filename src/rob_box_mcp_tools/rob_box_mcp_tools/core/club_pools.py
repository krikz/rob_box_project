"""club_pools.py — пулы вариаций club-трека: клэп, баланс слоёв, lpf-огибающие.

Issue #3226 (карточка (в) umbrella #3223, ADR-0146). Клэп, уровни слоёв и
lpf-огибающие лида/баса были ОДНИМИ на все треки клуба (эталон «By
Design»): менялись только каркас (#3224) и прогрессия.

Выбор — :func:`core.music_diversity.weighted_pick` со штрафом за недавнее,
как у каркаса. Правила совместимости (снимок seed=0 и sha-тесты #3224):

* ``seed=0`` и/или пустая история — всегда вариант ``reference``, то есть
  байты прежнего рендера;
* профили баланса только ПРИТЕНЯЮТ слой (множитель ≤ 1 к уже
  откалиброванному ``club_loudness`` уровню): громкость не уезжает вверх,
  калибровка (#3154) остаётся источником истины.

Модуль чистый, без ROS/Renardo. Имена вариантов пишутся в ``music_history``
(колонки ``clap``, ``lpf``, ``balance``).
"""

from __future__ import annotations

import random
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .music_diversity import weighted_pick

__all__ = [
    "CLAP_PATTERNS",
    "LEVEL_PROFILES",
    "LPF_PROFILES",
    "REFERENCE_VARIANT",
    "VARIANT_ROLES",
    "club_variant",
    "variant_levels",
]

#: Клэп ``*`` + открытый хэт ``=`` — один плеер, 16 шагов. Эталон — клэп на
#: 2 и 4, открытый хэт на слабые 8-е.
CLAP_PATTERNS: Dict[str, str] = {
    "reference": "..=.*.=...=.*.=.",
    # то же + призрачный клэп на последнем шаге такта
    "ghost": "..=.*.=...=.*.=*",
    # открытый хэт сдвинут на «и» доли
    "offhat": "...=*..=...=*..=",
    # только клэп, без открытых хэтов (суше)
    "dry": "....*.......*...",
    # клэп-синкопа: лишний удар на «и» 3-й доли
    "double": "..=.*.=..*.*.=..",
}

#: Профили баланса: множители 0..1 к откалиброванным уровням слоёв
#: (только тише; ``reference`` — без изменений). Сдвиги ≤ 15% (~1.4 дБ).
LEVEL_PROFILES: Dict[str, Dict[str, float]] = {
    "reference": {},
    "soft_lead": {"lead": 0.9},
    "airy": {"hats": 0.85, "clap": 0.92},
    "deep": {"pad": 0.9, "hats": 0.9},
    "punchy": {"pad": 0.85, "lead": 0.93, "hats": 0.9},
}

#: lpf-огибающие (лид, бас): аргумент ``lpf=`` целиком.
LPF_PROFILES: Dict[str, Tuple[str, str]] = {
    "reference": ("linvar([900, 4000], 31)", "linvar([500, 2500], 61)"),
    "slow": ("linvar([700, 4000], 47)", "linvar([400, 2200], 89)"),
    "bright": ("linvar([1200, 4000], 23)", "linvar([600, 2800], 47)"),
    "narrow": ("linvar([1500, 3200], 31)", "linvar([500, 2000], 61)"),
    "wide": ("linvar([600, 4000], 61)", "linvar([350, 2600], 61)"),
}

#: Роль варианта → (пул, колонка истории).
_POOLS: Dict[str, Tuple[Sequence[str], str]] = {
    "clap": (tuple(CLAP_PATTERNS), "clap"),
    "lpf": (tuple(LPF_PROFILES), "lpf"),
    "balance": (tuple(LEVEL_PROFILES), "balance"),
}
VARIANT_ROLES: Tuple[str, ...] = tuple(_POOLS)

REFERENCE_VARIANT: Dict[str, str] = {role: "reference" for role in _POOLS}


def club_variant(seed: int, recent: Optional[Sequence[Mapping[str, Any]]] = None) -> Dict[str, str]:
    """Имена клэпа, lpf-профиля и баланса трека.

    ``seed=0`` или пустая история — эталон (:data:`REFERENCE_VARIANT`).
    Иначе каждая роль — ``weighted_pick`` со штрафом за недавнее своим ГСЧ
    ``club-var:<seed>:<роль>`` (не трогает потоки каркаса и рифа).
    """
    if seed == 0 or not recent:
        return dict(REFERENCE_VARIANT)
    return {
        role: weighted_pick(list(pool), [r.get(column) for r in recent], random.Random(f"club-var:{seed}:{role}"))
        for role, (pool, column) in _POOLS.items()
    }


def variant_levels(
    variant: Optional[Mapping[str, str]], levels: Optional[Mapping[str, float]] = None,
) -> Optional[Mapping[str, float]]:
    """Множители слоёв: профиль баланса варианта × явные ``levels`` (0..1).

    Без профиля (``reference``/``None``) возвращает ``levels`` как есть —
    в том числе ``None``, рендер тогда побайтно прежний.
    """
    profile = LEVEL_PROFILES[(variant or {}).get("balance", "reference")]
    if not profile:
        return levels
    merged: Dict[str, float] = dict(profile)
    for lane, factor in (levels or {}).items():
        merged[lane] = merged.get(lane, 1.0) * factor
    return merged
