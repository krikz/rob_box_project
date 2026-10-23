"""«Робот помнит, с кем говорит» в диалоге v2 (#3919, решение Шифу 23.10, паритет с v1 ``_handle_speaker_turn``).

В начале адресованного хода при подтверждённом собеседнике (``memory.profiled_person``) код ведёт его профиль и
собирает раздел снимка ``speaker_memory`` (``memory.speaker_memory_block``): без правил в промпте (ADR-0148).

Две БД, у каждой своя роль (выбор описан в PR):

* профиль (``first_seen/last_seen/dialog_count``) - тем же швом, что вёл v1: ``MemoryIdentitySeam.note_seen`` в
  ``harness_voice.db`` под ``speaker_scope(person_ref)``; счётчики v1 продолжаются, а не обнуляются. Диалог - ход
  с адресованной репликой (как у v1: ``touch_speaker`` на каждый ход);
* факты - читаются из ``voice_facts`` (``voice_memory.db``), куда пишет ``memory_save`` и откуда читают
  ``memory_search``/``memory_context``: одна память на ``person_ref`` (PR-11), вторую копию фактов не заводим.

В режиме e2e боевые файлы не трогаются: профиль идёт в память процесса (на диск ни байта), факты читаются из e2e-БД
``voice_memory.e2e.db``. Сбой памяти не валит ход: раздела (или его части) просто нет.
"""

from __future__ import annotations

import asyncio
import sqlite3
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.memory import profiled_person, speaker_memory_block
from rob_box_harness.identity import Acquaintance, MemoryIdentitySeam
from rob_box_harness.memory import InMemoryStore


def read_voice_facts(db_path: str, speaker_id: str, limit: int) -> List[str]:
    """Факты человека из ``voice_facts`` - те, что пишет ``memory_save``: свежие первыми, только его
    (``speaker_id = ?``; безымянные общие строки #1770 - не его факты). БД только на чтение; файла/таблицы нет или
    она занята - пусто.
    Схему ``voice_facts`` знает ещё ``utils.legacy_voice_facts`` (склейка); тот пакет тянет pyaudio, v2 - нет."""
    if not db_path or not speaker_id:
        return []
    try:
        conn = sqlite3.connect(f"{Path(db_path).resolve().as_uri()}?mode=ro", uri=True, timeout=2.0)
    except (sqlite3.Error, ValueError):
        return []
    try:
        rows = conn.execute("SELECT fact FROM voice_facts WHERE speaker_id = ? ORDER BY updated_at DESC LIMIT ?",
                            (speaker_id, int(limit))).fetchall()
    except sqlite3.Error:
        return []
    finally:
        conn.close()
    return [str(row[0]) for row in rows]


class SpeakerMemory:
    """Профиль и факты собеседника хода. Путь пустой - этой части нет (а не молчаливый ``/data`` по умолчанию)."""

    def __init__(self, profile_db: str, facts_db: str, *, e2e_facts_db: str = "",
                 is_e2e: Callable[[], bool] = lambda: False, log: Any = None) -> None:
        self._profile_db, self._facts_db, self._e2e_facts_db = profile_db, facts_db, e2e_facts_db
        self._is_e2e = is_e2e
        self._log = log  # логгер с .info/.warning (ROS-логгер ноды)
        self._seams: Dict[bool, MemoryIdentitySeam] = {}

    async def _seam(self, e2e: bool) -> Optional[MemoryIdentitySeam]:
        if e2e not in self._seams:
            if e2e:
                store: Any = InMemoryStore()
            elif self._profile_db:
                from rob_box_harness.memory import SQLiteVoiceMemory  # noqa: PLC0415 - sqlite только когда нужен

                store = SQLiteVoiceMemory(db_path=self._profile_db)
            else:
                return None
            await store.init()
            self._seams[e2e] = MemoryIdentitySeam(store)
        return self._seams[e2e]

    def reset_e2e(self) -> None:
        """Вход в e2e: профили e2e-собеседников прошлого акта забыты (память процесса, не диск)."""
        self._seams.pop(True, None)

    async def _profile(self, person: str, name: Optional[str], e2e: bool) -> Optional[Mapping[str, Any]]:
        seam = await self._seam(e2e)
        return await seam.note_seen(Acquaintance(id=person, name=name)) if seam is not None else None

    async def section(self, speaker: Any) -> Optional[Mapping[str, Any]]:
        """Раздел ``speaker_memory`` для собеседника на начало хода (``None`` - не уверен/нечего показать)."""
        person = profiled_person(speaker)
        if person is None:
            return None
        e2e = bool(self._is_e2e())
        profile: Optional[Mapping[str, Any]] = None
        facts: Any = ()
        try:
            profile = await self._profile(person, speaker.name, e2e)
        except Exception as exc:  # noqa: BLE001 - память не валит ход
            self._warn("профиль", exc)
        try:
            facts = await asyncio.to_thread(read_voice_facts, self._e2e_facts_db if e2e else self._facts_db,
                                            person, kn.SPEAKER_FACTS_MAX)
        except Exception as exc:  # noqa: BLE001
            self._warn("факты", exc)
        block = speaker_memory_block(profile, facts)
        if self._log is not None:
            shown = block or {}
            self._log.info(f"🧠 [speaker-memory] person={person[:8]} dialog_count={shown.get('dialog_count', '-')} "
                           f"facts={len(shown.get('facts', ()))} e2e={e2e}")
        return block

    def _warn(self, what: str, exc: Exception) -> None:
        if self._log is not None:
            self._log.warning(f"⚠️ [speaker-memory] {what} не прочитан(ы): {type(exc).__name__}: {exc}")


__all__ = ["SpeakerMemory", "read_voice_facts"]
