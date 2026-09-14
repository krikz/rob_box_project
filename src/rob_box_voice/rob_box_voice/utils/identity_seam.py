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

from typing import Any

from rob_box_harness.identity import Acquaintance, IdentitySeam
from rob_box_harness.memory import MemoryStore, merge_speaker_facts

from .speaker_embeddings import SpeakerDatabase


class VoiceIdentitySeam(IdentitySeam):
    """Голосовой адаптер шва идентичности.

    :param speaker_db: ``SpeakerDatabase`` (``/data/speakers.db``) — слой
        биометрии. Используется «как есть», без изменения самого класса.
    :param memory: ``MemoryStore`` — памятный слой (профили/факты знакомых
        под ``speaker_scope(<id>)``).
    """

    def __init__(self, speaker_db: SpeakerDatabase, memory: MemoryStore) -> None:
        super().__init__(memory)
        self._db = speaker_db

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
        """
        embeddings_moved = self._db.merge_speakers(src_id, dst_id)
        facts_moved = await merge_speaker_facts(self._memory, src_id, dst_id)
        return embeddings_moved, facts_moved


__all__ = ["VoiceIdentitySeam"]
