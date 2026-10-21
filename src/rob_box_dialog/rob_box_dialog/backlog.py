"""Бэклог неадресованных фраз (ADR-0150 §7.3, ревизия 5, решение В12): помнит, но LLM не отдаёт.

Фраза без вейка роботу не адресована (В2) и копится здесь как ``{text, t, doa_deg, person_ref}`` на
:data:`~rob_box_dialog.knowledge.BACKLOG_TTL_S`. Наружу она выходит только двумя путями, оба — решение кода:

* :meth:`Backlog.pickup` — голое «робот» сразу после неуслышанной просьбы: самая свежая запись не старше
  :data:`~rob_box_dialog.knowledge.BACKLOG_PICKUP_S` из того же сектора DOA (±``DOA_SECTOR_DEG``) или от того же
  ``person_ref`` становится текстом хода и из бэклога уходит;
* :meth:`Backlog.recent_unaddressed` — в снимок LLM не больше одной такой записи, структурой
  ``{text, age_s, same_direction}``, без инструкции.

Всё остальное (телевизор, чужие разговоры) живёт только для счётчика «фоновая речь» и умирает по TTL.
Раньше 30 записей за 180 с уходили LLM блоком-инструкцией, и что с ними делать, решала модель (живой лог 20.10:
24 фразы ТВ за 40 минут, один и тот же вход давал то исполнение, то «Слушаю тебя»).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from rob_box_dialog.doa import angle_diff_deg
from rob_box_dialog.knowledge import BACKLOG_MAX_ENTRIES, BACKLOG_PICKUP_S, BACKLOG_TTL_S, DOA_SECTOR_DEG


@dataclass(frozen=True)
class Heard:
    """Реплика в момент приёма: текст, источник, когда прозвучала, откуда и от кого (если известно)."""

    text: str
    t: float
    source: str = "respeaker"
    doa_deg: Optional[float] = None
    person_ref: str = ""


def same_direction(a: Optional[float], b: Optional[float]) -> bool:
    """Оба направления известны и лежат в одном секторе ±``DOA_SECTOR_DEG``."""
    return a is not None and b is not None and angle_diff_deg(a, b) <= DOA_SECTOR_DEG


def same_source(entry: Heard, heard: Heard) -> bool:
    """Тот же человек (оба ``person_ref`` известны и равны) или то же направление."""
    if entry.person_ref and heard.person_ref:
        return entry.person_ref == heard.person_ref
    return same_direction(entry.doa_deg, heard.doa_deg)


class Backlog:
    """Неадресованные фразы на TTL; наружу — только свежая фраза того же источника."""

    def __init__(self, ttl_s: float = BACKLOG_TTL_S, max_entries: int = BACKLOG_MAX_ENTRIES) -> None:
        self.ttl_s, self.max_entries = ttl_s, max_entries
        self._entries: List[Heard] = []

    def add(self, heard: Heard) -> None:
        if not heard.text.strip():
            return
        self._prune(heard.t)
        self._entries.append(heard)
        del self._entries[:-self.max_entries]

    def clear(self) -> None:
        self._entries.clear()

    def background_phrases(self, now: float) -> int:
        """Сколько фраз фоновой речи лежит в бэклоге (метрика ``backlog.background_phrases``)."""
        self._prune(now)
        return len(self._entries)

    def _fresh(self, heard: Heard) -> Optional[Heard]:
        self._prune(heard.t)
        for entry in reversed(self._entries):
            if heard.t - entry.t <= BACKLOG_PICKUP_S and same_source(entry, heard):
                return entry
        return None

    def pickup(self, heard: Heard) -> Optional[Heard]:
        """Забрать свежую фразу того же источника как текст хода (В12); нет такой — ``None``."""
        entry = self._fresh(heard)
        if entry is not None:
            self._entries.remove(entry)
        return entry

    def recent_unaddressed(self, heard: Heard) -> Optional[Dict[str, object]]:
        """Не больше одной записи для снимка LLM: ``{text, age_s, same_direction}``; чужого не отдаёт."""
        entry = self._fresh(heard)
        if entry is None:
            return None
        return {"text": entry.text, "age_s": round(heard.t - entry.t, 1),
                "same_direction": same_direction(entry.doa_deg, heard.doa_deg)}

    def _prune(self, now: float) -> None:
        self._entries = [e for e in self._entries if now - e.t <= self.ttl_s]
