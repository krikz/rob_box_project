"""Identity seam package («Знакомый», issue #2440).

Экспортирует интерфейс шва идентичности: value-объект
:class:`Acquaintance`, абстрактный :class:`IdentitySeam` и памятный
:class:`MemoryIdentitySeam` (без биометрии). Голосовой адаптер живёт в
``rob_box_voice.utils.identity_seam``, лицевой (ADR-0089 Phase 2) — в
``rob_box_perception``, оба реализуют один и тот же контракт.
"""

from __future__ import annotations

from rob_box_harness.identity.base import (
    Acquaintance,
    IdentitySeam,
    MemoryIdentitySeam,
)

__all__ = [
    "Acquaintance",
    "IdentitySeam",
    "MemoryIdentitySeam",
]
