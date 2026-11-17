"""Re-export shim for VoiceFloorCache / FloorState / FloorHolder.

ADR-0051 §2.2 (issue #1999) — ``FloorHolder``, ``FloorState`` и
``VoiceFloorCache`` переехали в :mod:`rob_box_quest.core.floor`,
чтобы :class:`rob_box_quest.core.avatar_arbiter.LocalAvatarArbiterClient`
мог импортировать ``FloorHolder`` без циркулярной зависимости
через ``server/__init__.py`` → ``ws_server`` →
``core.avatar_arbiter`` → ``server.voice_floor``.

Этот модуль — тонкий re-export для backward-compat с уже написанным
кодом и тестами (``from rob_box_quest.server.voice_floor import
VoiceFloorCache`` остаётся рабочим путём). Никакой логики здесь
больше нет — она вся в ``core/floor.py``.
"""

from __future__ import annotations

from ..core.floor import FloorHolder, FloorState, VoiceFloorCache

__all__ = ["FloorHolder", "FloorState", "VoiceFloorCache"]