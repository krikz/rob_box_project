"""legacy_voice_facts.py — перенос строк ``voice_facts`` при склейке спикеров.

Issue #2751 (инвентаризация ``/data`` на Vision Pi), продолжение issue #2440
(шов идентичности).

Контекст
--------
``VoiceIdentitySeam.merge()`` (``identity_seam.py``) переносит факты профиля
через ``rob_box_harness.memory.merge_speaker_facts()`` — но та функция знает
только про ``MemoryStore``-факты (таблица ``facts`` в ``harness_voice.db``,
модель scope/key/value). Живые пользовательские факты («не ем лук», «пью
чай без сахара» и т. п.) копит СОВСЕМ ДРУГОЙ, более старый писатель —
``rob_box_voice.core.voice_memory.VoiceMemory`` (через MCP-инструмент
``memory_save``), пишущий в таблицу ``voice_facts`` файла
``/data/voice_memory.db`` (колонки ``speaker_id``, ``fact``, ``category`` —
никакого отношения к scope/key модели харнесса).

Замер на роботе 22.09.2026 (issue #2751): ``harness_voice.db`` — 10 фактов,
последняя запись 15.09 (см. ``docs/adr/0128-...``); ``voice_memory.db`` —
100 фактов, записи сегодняшние. ``VoiceIdentitySeam.merge()`` до этого
модуля видел только первую БД — склейка двух профилей одного человека
переносила 10 старых фактов и НЕ трогала 100 живых.

Это НЕ дублирует ``merge_speaker_facts``: та функция сохраняет
scope/key-семантику (конфликт ключей → выигрывает dst), эта — плоское
переприсвоение владельца строки, потому что ``voice_facts`` не знает
понятия «ключ», там просто последовательность произвольных фактов на
спикера без уникальности.
"""

from __future__ import annotations

import logging
import sqlite3
import time

logger = logging.getLogger(__name__)

__all__ = ["merge_legacy_voice_facts"]


def merge_legacy_voice_facts(db_path: str, src_speaker_id: str, dst_speaker_id: str) -> int:
    """Переприсвоить ``voice_facts.speaker_id`` от ``src`` к ``dst``.

    :param db_path: путь к ``voice_memory.db`` (легаси-БД MCP-инструментов,
        ``VOICE_MEMORY_DB_PATH`` / ``/data/voice_memory.db`` по умолчанию).
    :param src_speaker_id: биометрический id профиля-источника (будет
        удалён вызывающим кодом после склейки — ``SpeakerDatabase.merge_speakers``).
    :param dst_speaker_id: биометрический id профиля-получателя.
    :returns: число ФАКТИЧЕСКИ перенесённых строк. ``0``, если пусто, файла
        нет или таблицы ``voice_facts`` ещё не существует (легаси-писатель
        мог ни разу не запуститься — это не ошибка, а «нечего переносить»).

    Синхронная функция (обычный ``sqlite3``), а не async ``MemoryStore`` —
    ``voice_facts`` не участвует в контракте шва, это прямой доступ к чужой
    схеме, изолированный в одном месте, чтобы не размазывать знание о ней
    по ``identity_seam.py``.
    """
    if not src_speaker_id or not dst_speaker_id or src_speaker_id == dst_speaker_id:
        return 0

    try:
        conn = sqlite3.connect(db_path, timeout=5.0)
    except sqlite3.Error as exc:
        logger.warning(
            "merge_legacy_voice_facts: не удалось открыть %s (%s: %s) — "
            "перенос живых фактов пропущен",
            db_path,
            type(exc).__name__,
            exc,
        )
        return 0

    try:
        # Таблицы может не быть: легаси-писатель (mcp_server.py) создаёт её
        # лениво при первом save_fact. Пустая/отсутствующая — легальный
        # исход ("нечего переносить"), не ошибка.
        exists = conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='voice_facts'"
        ).fetchone()
        if not exists:
            logger.info(
                "merge_legacy_voice_facts: таблица voice_facts отсутствует в %s — "
                "перенос пропущен (легаси-писатель ещё не создавал записей)",
                db_path,
            )
            return 0

        now = time.time()
        with conn:
            cur = conn.execute(
                "UPDATE voice_facts SET speaker_id = ?, updated_at = ? "
                "WHERE speaker_id = ?",
                (dst_speaker_id, now, src_speaker_id),
            )
        moved = cur.rowcount if cur.rowcount and cur.rowcount > 0 else 0
        if moved:
            logger.info(
                "merge_legacy_voice_facts: %d факт(ов) перенесено %s → %s в %s",
                moved,
                src_speaker_id[:8],
                dst_speaker_id[:8],
                db_path,
            )
        return moved
    except sqlite3.Error as exc:
        logger.warning(
            "merge_legacy_voice_facts: UPDATE в %s провалился (%s: %s) — "
            "перенос живых фактов пропущен",
            db_path,
            type(exc).__name__,
            exc,
        )
        return 0
    finally:
        conn.close()
