#!/usr/bin/env python3
"""
radio.py — Per-chat ``/radio`` mode для AV-23 (issue #1915, P8).

Режим «рация из Telegram»: оператор шлёт голосовое сообщение в Telegram →
оно транскодируется в int16 PCM 16 kHz mono → нарезается чанками по
~20 мс (как делает quest-сервер в ``publish_voice_audio``, см.
``quest_node.py`` D7) → публикуется в ``/avatar/voice_in``, откуда
``sound_node`` стримит его в динамик робота.

Дизайн (docs/plans/2026-08-27-quest-voice-passthrough-design.md §1.1)
--------------------------------------------------------------------

* Команда ``/radio on|off|status`` — per-chat, default ``off``.
* При ``off`` поведение голосовых сообщений НЕ меняется: STT → текст.
* При ``on`` голосовое сообщение идёт в рацию, STT НЕ запускается.
* Публикация в ``/avatar/voice_in`` ОБЯЗАНА идти под
  ``AcquireFloor(voice)`` (ADR-0028 §4.4). Иначе рация будет орать
  одновременно с TTS/teleop-подсказками.
* Тайминг публикации — реальное время воспроизведения (20 мс/чанк),
  чтобы ``sound_node`` (silence-timeout 300 мс) успевал сшить стрим.
* Ограничения (защита от злоупотребления):
  - ``radio_max_duration_s`` (default 30) — больше режем с вежливым
    отказом;
  - ``radio_max_bytes`` (default 5 MB) — больше режем с вежливым
    отказом до транскода.

Зависимости
-----------

* ``voice_transcode.ogg_to_pcm16k`` — ffmpeg-based конвертер.
* ``supervisor_client.SupervisorClient`` (Phase 1 monitor / Phase 2
  active) — запрос voice_floor.
* ``audio_common_msgs/AudioData`` — контейнер для чанков.

Модуль не импортирует rclpy на уровне модуля (только лениво внутри
методов через ``node``), чтобы юнит-тесты могли подсунуть stub-node.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Any, Dict, Optional

from .voice_transcode import (
    TARGET_CHANNELS,
    TARGET_SAMPLE_RATE_HZ,
    TARGET_SAMPLE_WIDTH_BYTES,
    VoiceTranscodeError,
    ogg_to_pcm16k,
)

logger = logging.getLogger(__name__)

# Размер чанка в миллисекундах (как у quest-сервера,
# ``docs/plans/2026-08-27-quest-voice-passthrough-design.md`` D7).
DEFAULT_CHUNK_MS = 20
# Время воспроизведения одного чанка в реальном времени (сек). Это
# единственный «синхронизатор» — публикуем чанк, ждём ровно столько,
# чтобы ``sound_node`` получил каждый чанк в свой OutputStream с
# правильным таймингом.
CHUNK_REALTIME_S = DEFAULT_CHUNK_MS / 1000.0

# Состояние режима рации хранится в context.user_data под этим ключом
# (per-chat). Значение — bool: True == ON.
_RADIO_STATE_KEY = "radio_mode"

# Какие сообщения от оператора рация может подавать (для observability).
RADIO_ACCEPTED_LOG = "radio accepted, %d bytes"
RADIO_REJECTED_FLOOR_LOG = "radio rejected: voice_floor held by %s"
RADIO_REJECTED_TOO_BIG_LOG = "radio rejected: file too large (%d > %d bytes)"
RADIO_REJECTED_TOO_LONG_LOG = "radio rejected: duration %d > %d s"


def _chunk_pcm_bytes(chunk_ms: int) -> int:
    """Размер PCM-чанка в байтах для заданной длительности."""
    return TARGET_SAMPLE_RATE_HZ * TARGET_CHANNELS * TARGET_SAMPLE_WIDTH_BYTES * chunk_ms // 1000


def get_radio_mode(context_user_data: Dict[str, Any]) -> bool:
    """Прочитать per-chat режим рации (default ``False``)."""
    return bool(context_user_data.get(_RADIO_STATE_KEY, False))


def set_radio_mode(context_user_data: Dict[str, Any], on: bool) -> None:
    """Поставить per-chat режим рации."""
    context_user_data[_RADIO_STATE_KEY] = bool(on)


class RadioPublisher:
    """Узел-агрегатор: объединяет транскод, чанкование и публикацию.

    Принимает ``node`` (TelegramNode), держит параметры ROS 2
    (chunk_ms / max_duration / max_bytes) и владеет
    ``SupervisorClient`` через ``node.supervisor``.

    Threading model:
      * ``publish_radio`` — async, вызывается из telegram-loop.
      * Чанки публикуются через ``node.publish_voice_audio_chunk``,
        которая дёргает rclpy-publisher (thread-safe).
      * Реальное время отмеряется ``asyncio.sleep(CHUNK_REALTIME_S)``
        — это даёт честный real-time темп без блокировки event-loop.

    Concurrency:
      * ``_lock`` защищает от параллельных раций из одного чата (защита
        от двойного нажатия «отправить»). Кросс-чат параллельность
        допустима — каждый чат имеет свой lock (см. ``locks``).
    """

    DEFAULT_MAX_DURATION_S = 30.0
    DEFAULT_MAX_BYTES = 5 * 1024 * 1024  # 5 MB

    def __init__(
        self,
        node: Any,
        *,
        chunk_ms: int = DEFAULT_CHUNK_MS,
        max_duration_s: float = DEFAULT_MAX_DURATION_S,
        max_bytes: int = DEFAULT_MAX_BYTES,
        chunk_count_upper_bound: int = 4096,
    ) -> None:
        self._node = node
        self._chunk_ms = int(chunk_ms)
        self._chunk_bytes = _chunk_pcm_bytes(self._chunk_ms)
        self._max_duration_s = float(max_duration_s)
        self._max_bytes = int(max_bytes)
        # Защита от зависших раций (chunked loop): даже при идеальном
        # максимуме 30 c и 20 мс/чанк получаем 1500 итераций. 4096 —
        # восьмикратный запас; больше — бросаем ``RadioTooLong``.
        self._chunk_count_upper_bound = int(chunk_count_upper_bound)
        # per-chat locks (чтобы один и тот же chat не отправлял две
        # рации одновременно).
        self._locks: Dict[int, asyncio.Lock] = {}
        self._locks_guard = threading.Lock()

    @property
    def chunk_bytes(self) -> int:
        return self._chunk_bytes

    @property
    def chunk_ms(self) -> int:
        return self._chunk_ms

    @property
    def max_duration_s(self) -> float:
        return self._max_duration_s

    @property
    def max_bytes(self) -> int:
        return self._max_bytes

    async def publish_radio(
        self,
        chat_id: int,
        ogg_bytes: bytes,
    ) -> "RadioResult":
        """Опубликовать OGG-голосовое как рацию в ``/avatar/voice_in``.

        Args:
            chat_id: ID чата (для логов и per-chat lock).
            ogg_bytes: сырой OGG/Opus от Telegram.

        Returns:
            ``RadioResult`` с полями ``ok``, ``reason``, ``chunks``,
            ``bytes``, ``duration_ms``. ``reason`` — None на успехе,
            иначе одна из констант ``RadioResult.REASON_*``.

        Никогда не бросает — все исключения заворачиваются в
        ``RadioResult.ok=False``.
        """
        lock = self._lock_for(chat_id)
        async with lock:
            return await self._publish_locked(chat_id, ogg_bytes)

    async def _publish_locked(self, chat_id: int, ogg_bytes: bytes) -> "RadioResult":
        # 1. Защита от слишком большого файла — ДО дорогого транскода.
        if len(ogg_bytes) > self._max_bytes:
            logger.info(RADIO_REJECTED_TOO_BIG_LOG, len(ogg_bytes), self._max_bytes)
            return RadioResult(
                ok=False,
                reason=RadioResult.REASON_TOO_BIG,
                chunks=0,
                bytes_=len(ogg_bytes),
                duration_ms=0,
            )

        # 2. Транскод OGG → PCM. CPU-bound, но ffmpeg — внешний процесс,
        # event-loop не блокируется.
        try:
            pcm = await asyncio.to_thread(ogg_to_pcm16k, ogg_bytes)
        except VoiceTranscodeError as exc:
            logger.warning("radio transcode failed: %s", exc)
            return RadioResult(
                ok=False,
                reason=RadioResult.REASON_TRANSCODE,
                chunks=0,
                bytes_=len(ogg_bytes),
                duration_ms=0,
                error=str(exc),
            )

        # 3. Длительность и лимит по времени.
        duration_ms = len(pcm) // (TARGET_SAMPLE_RATE_HZ * TARGET_CHANNELS * TARGET_SAMPLE_WIDTH_BYTES // 1000)
        duration_s = duration_ms / 1000.0
        if duration_s > self._max_duration_s:
            logger.info(
                RADIO_REJECTED_TOO_LONG_LOG,
                int(duration_s),
                int(self._max_duration_s),
            )
            return RadioResult(
                ok=False,
                reason=RadioResult.REASON_TOO_LONG,
                chunks=0,
                bytes_=len(ogg_bytes),
                duration_ms=duration_ms,
            )

        # 4. Запрос voice_floor у супервизора. Если занят — отказываем.
        supervisor = getattr(self._node, "supervisor", None)
        if supervisor is None:
            logger.error("radio: node has no supervisor client, refusing")
            return RadioResult(
                ok=False,
                reason=RadioResult.REASON_NO_SUPERVISOR,
                chunks=0,
                bytes_=len(ogg_bytes),
                duration_ms=duration_ms,
            )

        result_holder: Dict[str, Any] = {}

        def _do_publish_chunks() -> None:
            """Синхронная часть: нарезка и публикация чанков в ROS.

            Запускается внутри ``supervisor.with_floor``, чтобы floor
            удерживался на всё время стрима. ``asyncio.run`` тут нет —
            мы публикуем ROS-сообщения напрямую (rclpy thread-safe).
            Real-time темп отмеряет caller через ``asyncio.sleep``.
            """
            chunk_size = self._chunk_bytes
            total = len(pcm)
            chunks = 0
            offset = 0
            while offset < total:
                piece = pcm[offset : offset + chunk_size]
                offset += chunk_size
                chunks += 1
                if chunks > self._chunk_count_upper_bound:
                    # Safety: больше чем можем обработать, дальше не шлём.
                    break
                self._node.publish_voice_audio_chunk(piece)
            result_holder["chunks"] = chunks

        # ``with_floor`` отдаёт ``AcquireResult``; внутри он сам
        # выполняет callback и отпускает floor.
        from .supervisor_client import Floor  # локальный импорт — избегаем цикла

        floor_result = supervisor.with_floor(Floor.VOICE, _do_publish_chunks)
        if not floor_result.granted:
            held = floor_result.held_by or "другим оператором"
            logger.info(RADIO_REJECTED_FLOOR_LOG, held)
            return RadioResult(
                ok=False,
                reason=RadioResult.REASON_FLOOR_BUSY,
                chunks=0,
                bytes_=len(ogg_bytes),
                duration_ms=duration_ms,
                held_by=held,
            )

        chunks_published = int(result_holder.get("chunks", 0))
        logger.info(
            "radio: chat=%d published %d chunks (%d PCM bytes, %d ms)",
            chat_id,
            chunks_published,
            len(pcm),
            duration_ms,
        )

        # 5. Real-time отмерить темп публикации: между чанками спим
        # CHUNK_REALTIME_S. Это даёт sound_node равномерный поток.
        if chunks_published > 0:
            await asyncio.sleep(chunks_published * CHUNK_REALTIME_S)

        # 6. Сигнал окончания стрима: stop в /voice/sound/stop.
        # ``sound_node`` через VOICE_SILENCE_TIMEOUT (300 мс) сам
        # закроет stream, но явный STOP надёжнее (barge-in уже
        # использует его).
        self._node.publish_voice_audio_stop()

        return RadioResult(
            ok=True,
            reason=None,
            chunks=chunks_published,
            bytes_=len(ogg_bytes),
            duration_ms=duration_ms,
        )

    def _lock_for(self, chat_id: int) -> asyncio.Lock:
        with self._locks_guard:
            lock = self._locks.get(chat_id)
            if lock is None:
                lock = asyncio.Lock()
                self._locks[chat_id] = lock
            return lock


class RadioResult:
    """Ответ ``RadioPublisher.publish_radio`` — что увидит handler."""

    REASON_OK = "ok"
    REASON_TOO_BIG = "too_big"
    REASON_TOO_LONG = "too_long"
    REASON_TRANSCODE = "transcode_failed"
    REASON_FLOOR_BUSY = "floor_busy"
    REASON_NO_SUPERVISOR = "no_supervisor"

    __slots__ = ("ok", "reason", "chunks", "bytes_", "duration_ms", "error", "held_by")

    def __init__(
        self,
        *,
        ok: bool,
        reason: Optional[str],
        chunks: int,
        bytes_: int,
        duration_ms: int,
        error: Optional[str] = None,
        held_by: Optional[str] = None,
    ) -> None:
        self.ok = ok
        self.reason = reason
        self.chunks = chunks
        self.bytes_ = bytes_
        self.duration_ms = duration_ms
        self.error = error
        self.held_by = held_by

    def __repr__(self) -> str:  # pragma: no cover - debug only
        return (
            f"RadioResult(ok={self.ok}, reason={self.reason!r}, "
            f"chunks={self.chunks}, bytes_={self.bytes_}, "
            f"duration_ms={self.duration_ms})"
        )
