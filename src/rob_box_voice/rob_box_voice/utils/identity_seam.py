"""Voice adapter for the identity seam (issue #2440).

``VoiceIdentitySeam`` — первый адаптер к шву ``rob_box_harness.identity``.
Оборачивает :class:`SpeakerDatabase` (голосовая биометрия resemblyzer,
``speaker_embeddings.py``) и ``MemoryStore`` (факты/профили знакомых) в
один контракт ``resolve`` / ``note_seen`` / ``since_last_seen`` / ``merge``.

Сигнал для голосового адаптера — d-vector resemblyzer (numpy-вектор),
полученный через ``SpeakerDatabase.embed_audio``. ``resolve`` возвращает
``Acquaintance`` с ``id = speaker_id`` (биометрический UUID), а НЕ Yandex
``speaker_tag`` — именно так шов делает идентичность стабильной между
сессиями.
"""

from __future__ import annotations

import asyncio
from typing import Any, Optional

from rob_box_harness.identity import Acquaintance, IdentitySeam
from rob_box_harness.memory import MemoryStore, merge_speaker_facts

from .legacy_voice_facts import merge_legacy_voice_facts
from .speaker_embeddings import SpeakerDatabase


class VoiceIdentitySeam(IdentitySeam):
    """Голосовой адаптер шва идентичности.

    :param speaker_db: ``SpeakerDatabase`` (``/data/speakers.db``) — слой
        биометрии. Используется «как есть», без изменения самого класса.
    :param memory: ``MemoryStore`` — памятный слой (профили/факты знакомых
        под ``speaker_scope(<id>)``). На роботе это ``harness_voice.db``.
    :param legacy_facts_db_path: путь к ``voice_memory.db`` — второй, более
        старый писатель фактов (MCP-инструмент ``memory_save``, таблица
        ``voice_facts``). Issue #2751: пока обе БД живы (миграция #2000
        не доведена до конца — см. ADR-0128), ``merge()`` без этого пути
        переносил бы только факты из ``harness_voice.db`` и молча пропускал
        бы живые факты из ``voice_facts``. ``None`` (по умолчанию)
        отключает перенос — используется в тестах, которые не поднимают
        вторую БД.
    """

    def __init__(
        self,
        speaker_db: SpeakerDatabase,
        memory: MemoryStore,
        *,
        legacy_facts_db_path: Optional[str] = None,
    ) -> None:
        super().__init__(memory)
        self._db = speaker_db
        self._legacy_facts_db_path = legacy_facts_db_path

    def resolve(self, signal: Any) -> Acquaintance | None:
        """Опознать голосовой сигнал → знакомый с биометрическим id."""
        match = self._db.identify(signal)
        if match is None:
            return None
        return Acquaintance(
            id=match.speaker_id,
            name=match.name,
            epithet=match.epithet,
            confidence=match.confidence,
        )

    def register(self, signal: Any, name: str) -> Acquaintance:
        """Зарегистрировать голос (или дописать в существующий профиль).

        Обёртка над ``SpeakerDatabase.register_or_merge`` (issue W5-4):
        голос, похожий на уже известного спикера, дописывается в его
        профиль вместо создания дубля. Возвращает знакомого со стабильным id.
        """
        sid, _reused = self._db.register_or_merge(name, signal)
        profile = self._db.get_speaker_profile(sid) or {}
        return Acquaintance(
            id=sid,
            name=profile.get("name") or name,
            epithet=profile.get("epithet"),
        )

    async def merge(self, src_id: str, dst_id: str) -> tuple[int, int]:
        """Склеить две записи одного человека (эмбеддинги + факты).

        Возвращает ``(embeddings_moved, facts_moved)``. Обе операции идут
        по единому биометрическому ключу, поэтому ручной маппинг
        tag↔uuid (которого раньше не существовало) больше не нужен —
        дефект C из issue #2440 закрыт архитектурно.

        ``facts_moved`` — сумма по ОБОИМ писателям фактов (issue #2751):
        ``harness_voice.db`` (``self._memory``, через ``merge_speaker_facts``)
        и, если задан ``legacy_facts_db_path``, ``voice_facts`` в
        ``voice_memory.db`` (через ``merge_legacy_voice_facts``). Без
        второго слагаемого склейка двух профилей одного человека переносила
        бы только facts из harness-БД (в проде на 22.09.2026 — 10 старых
        строк) и молча теряла бы живые факты MCP-инструментов (100 строк).
        """
        embeddings_moved = self._db.merge_speakers(src_id, dst_id)
        facts_moved = await merge_speaker_facts(self._memory, src_id, dst_id)
        if self._legacy_facts_db_path:
            legacy_moved = await asyncio.to_thread(
                merge_legacy_voice_facts,
                self._legacy_facts_db_path,
                src_id,
                dst_id,
            )
            facts_moved += legacy_moved
        return embeddings_moved, facts_moved


__all__ = ["VoiceIdentitySeam"]
