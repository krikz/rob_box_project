"""Encounter seam package («Встреча», issue #2442).

Экспортирует интерфейс шва: value-объект :class:`Encounter`, множество
:class:`EncounterChannel` и сам шов :class:`EncounterSeam` (интерфейс —
одна операция чтения, :meth:`EncounterSeam.current`, и одна операция
записи для адаптеров, :meth:`EncounterSeam.observe`). Голосовой адаптер —
:class:`VoiceEncounterAdapter` в ``voice_adapter.py``; зрительный (issue
#2531/#2583) — следующий шаг, вне скоупа первого инкремента.

Размещение — тот же принцип, что и у шва идентичности
(``rob_box_harness.identity``, issue #2440): оба потребителя,
``dialogue_node`` (``rob_box_voice``) и ``mcp_server``
(``rob_box_mcp_tools``), уже зависят от ``rob_box_harness`` (memory,
identity) — общий шов присутствия живёт там же, а не в одном из
потребителей, иначе второй был бы вынужден зависеть от первого либо
дублировать логику слияния каналов.

Соотношение с ADR-0096 (``docs/adr/0096-encounter-seam.md``, статус
**Proposed**, не принят): этот пакет — первый инкремент issue #2442, а
не реализация ADR-0096 целиком. ADR-0096 проектирует распределённую
схему (``encounter_node`` как отдельный ROS-процесс, ``EncounterState.msg``,
реестр НЕСКОЛЬКИХ одновременных встреч, ``кто: str`` без прямой
зависимости от identity). Этот PR — заметно меньше: один in-process шов
по образцу уже принятого ``IdentitySeam`` (issue #2440), одна текущая
Встреча, ``who: Acquaintance | None`` напрямую (не строка) — потому что
идентичность (issue #2440) в итоге легла в тот же пакет
``rob_box_harness``, а не в отдельный ``rob_box_identity``, как
предполагал ADR-0096 §2.1 на момент написания; кросс-пакетной проблемы,
которую там решали строковым id, на практике не возникло. Открытые
вопросы ADR-0096 (§10 — merge_window, несколько людей одновременно,
config-fix топика, face-продюсер) этот PR не решает и не обязан —
голосовой адаптер и мерж-логика шва работают независимо от исхода
ревью ADR-0096 товарищем Шифу.
"""

from __future__ import annotations

from rob_box_harness.encounter.base import (
    DEFAULT_PRESENCE_TIMEOUT_SEC,
    Encounter,
    EncounterChannel,
    EncounterSeam,
)
from rob_box_harness.encounter.voice_adapter import (
    VoiceEncounterAdapter,
    acquaintance_from_speaker_result,
)

__all__ = [
    "DEFAULT_PRESENCE_TIMEOUT_SEC",
    "Encounter",
    "EncounterChannel",
    "EncounterSeam",
    "VoiceEncounterAdapter",
    "acquaintance_from_speaker_result",
]
