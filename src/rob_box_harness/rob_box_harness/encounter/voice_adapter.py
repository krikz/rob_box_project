"""Voice adapter for the encounter seam (issue #2442).

Сегодня сигнал ``/voice/speaker/result`` (результат голосовой биометрии,
speaker_id_node) независимо парсится дважды — ``dialogue_node.
_on_speaker_result`` и ``mcp_server._on_speaker_result`` — в два не
знающих друг о друге состояния (``dialogue_node._current_speaker`` и
``mcp_server.current_speaker_id``, issue #2442 «текущее состояние»).
:class:`VoiceEncounterAdapter` — единственное место, которое переводит
этот сырой JSON-сигнал в :meth:`EncounterSeam.observe`.

Подключение обоих потребителей к этому адаптеру (замена
``_current_speaker``/``current_speaker_id`` на ``EncounterSeam.current()``)
— отдельный PR (issue #2442, «следующие шаги»): в этом PR адаптер вводится
и тестируется автономно, чтобы не смешивать введение шва с миграцией
потребителей.
"""

from __future__ import annotations

from typing import Any, Mapping, Optional

from rob_box_harness.identity import Acquaintance

from .base import Encounter, EncounterChannel, EncounterSeam

#: Значения ``name``, которые resemblyzer/спикер-БД иногда кладёт вместо
#: реального имени (issue #1077/#1101) — не имя, а мусор.
_JUNK_NAMES = frozenset({"null", "none", "undefined", ""})


def _clean_str(value: Any) -> Optional[str]:
    text = str(value).strip() if value is not None else ""
    if text.lower() in _JUNK_NAMES:
        return None
    return text or None


def acquaintance_from_speaker_result(payload: Mapping[str, Any]) -> Optional[Acquaintance]:
    """Собрать ``Acquaintance`` из payload ``/voice/speaker/result``.

    Формат payload (speaker_id_node): ``{"is_known": true, "speaker_id":
    "<uuid>", "name": "...", "confidence": 0.93}`` или ``{"is_known":
    false}``. Возвращает ``None``, если спикер не опознан или в payload
    нет стабильного ``speaker_id`` — под per-session данные профиль
    заводить нельзя (issue #2440).
    """
    if not payload.get("is_known"):
        return None
    speaker_id = _clean_str(payload.get("speaker_id"))
    if speaker_id is None:
        return None
    try:
        confidence = float(payload.get("confidence") or 0.0)
    except (TypeError, ValueError):
        confidence = None
    return Acquaintance(
        id=speaker_id,
        name=_clean_str(payload.get("name")),
        epithet=_clean_str(payload.get("epithet")),
        confidence=confidence,
    )


class VoiceEncounterAdapter:
    """Голосовой адаптер шва «Встреча».

    :param seam: :class:`EncounterSeam`, в который кормится сигнал.
    """

    def __init__(self, seam: EncounterSeam) -> None:
        self._seam = seam

    async def on_speaker_result(
        self,
        payload: Mapping[str, Any],
        *,
        now: Optional[float] = None,
    ) -> Optional[Encounter]:
        """Обработать один payload ``/voice/speaker/result``.

        Registration-ack (``{"event": "registered", ...}``) — не сигнал
        присутствия (то же самое исключение уже делают оба текущих
        потребителя); возвращает текущую Встречу без изменений.
        """
        if payload.get("event") == "registered":
            return self._seam.current(now=now)

        who = acquaintance_from_speaker_result(payload)
        if who is not None:
            confidence = who.confidence if who.confidence is not None else 0.0
        else:
            confidence = 0.0
        return await self._seam.observe(
            EncounterChannel.VOICE, who, confidence, now=now
        )


__all__ = [
    "VoiceEncounterAdapter",
    "acquaintance_from_speaker_result",
]
