#!/usr/bin/env python3
"""
voice_transcode.py — Convert Telegram voice messages (OGG/Opus) to int16 PCM 16 kHz mono.

AV-23 (issue #1915, P8): передача голоса оператора из Telegram в
``/avatar/voice_in`` через рацию (voice passthrough). До этого фичи
telegram-бот транскрибировал голос в текст, но PCM-пути к динамику
робота не было. Принимающая сторона (``sound_node`` на
``/avatar/voice_in``) ждёт именно ``AudioData`` int16 LE PCM, 16 kHz, mono
(см. ``docs/plans/2026-08-27-quest-voice-passthrough-design.md`` D1).

Решение по инструменту
----------------------

ffmpeg (есть в образе ``rob_box_base:ros2-zenoh-humble``) — единственный
надёжный декодер OGG/Opus, поддерживающий произвольные параметры
Telegram (битрейт, mono/stereo, sample rate). ``pydub`` без ffmpeg не
работает; ``soundfile`` OGG не читает; ``av`` (PyAV) ставит
дополнительный бинарь и тянет тяжёлую зависимость.

ffmpeg даёт нам:

* декодер OGG/Opus/anything-Telegram,
* resample в 16 kHz одним ключом (``-ar``),
* downmix в mono одним ключом (``-ac 1``),
* s16le на выходе (``-f s16le``),
* читаем stdin → stdout, не плодит временные файлы.

Это чистый модуль — без ROS, без asyncio, без сети. Импортируется
и из телеграм-ноды, и из юнит-тестов на фикстуре.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
from typing import Optional

logger = logging.getLogger(__name__)

# Выход PCM: 16 kHz, mono, signed 16-bit little-endian — точное совпадение
# с контрактом ``audio_common_msgs/AudioData`` в sound_node.
TARGET_SAMPLE_RATE_HZ = 16000
TARGET_CHANNELS = 1
TARGET_SAMPLE_WIDTH_BYTES = 2  # int16

# Максимальный размер исходного OGG, который мы готовы обработать за один
# вызов. Это защита от того, чтобы кто-то не отправил 50-мегабайтный
# голосовой файл (Telegram режет на 50 МБ, но мы режем жёстче — рация не
# про бесконечные записи). 20 МБ ≈ 2.5 часа OGG/Opus 8 kbps.
MAX_INPUT_BYTES = 20 * 1024 * 1024

# Допуск по длительности между исходным OGG (если есть probe) и PCM.
# ffmpeg resample может накинуть 1-2 мс padding, не считаем это ошибкой.
DURATION_TOLERANCE_MS = 200


class VoiceTranscodeError(Exception):
    """Не удалось транскодировать OGG → PCM (битый формат, нет ffmpeg, …)."""


def _find_ffmpeg() -> str:
    """Найти ffmpeg в PATH. Бросает VoiceTranscodeError если не нашли."""
    path = shutil.which("ffmpeg")
    if not path:
        raise VoiceTranscodeError(
            "ffmpeg not found in PATH — required for OGG/Opus → PCM transcode "
            "(install ffmpeg in the telegram-bot image)"
        )
    return path


def ogg_to_pcm16k(ogg: bytes, *, probe_duration_ms: Optional[int] = None) -> bytes:
    """Транскодировать OGG/Opus → int16 PCM 16 kHz mono.

    Args:
        ogg: сырые байты OGG/Opus (как приходят из Telegram).
        probe_duration_ms: опциональная длительность исходника в мс
            (для sanity-check через ffprobe — если задана и расходится с
            PCM больше чем на ``DURATION_TOLERANCE_MS``, бросаем ошибку).
            Если ``None`` — sanity-check пропускается.

    Returns:
        bytes int16 LE PCM 16 kHz mono.

    Raises:
        VoiceTranscodeError: пустой/битый вход, ffmpeg вернул код != 0,
            результат не парсится как PCM, расхождение длительности
            больше допуска.

    Examples:
        >>> pcm = ogg_to_pcm16k(b'...')
        >>> len(pcm) % 2 == 0  # int16 — чётное число байт
        True
    """
    if not ogg:
        raise VoiceTranscodeError("empty input: ogg bytes are empty")

    if len(ogg) > MAX_INPUT_BYTES:
        raise VoiceTranscodeError(
            f"input too large: {len(ogg)} bytes > {MAX_INPUT_BYTES} limit " "(likely a corrupt or malicious upload)"
        )

    ffmpeg = _find_ffmpeg()

    cmd = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        "pipe:0",  # stdin
        "-f",
        "s16le",  # raw int16 LE
        "-acodec",
        "pcm_s16le",
        "-ac",
        str(TARGET_CHANNELS),
        "-ar",
        str(TARGET_SAMPLE_RATE_HZ),
        "pipe:1",  # stdout
    ]

    try:
        proc = subprocess.run(
            cmd,
            input=ogg,
            capture_output=True,
            check=False,
            timeout=30,
        )
    except subprocess.TimeoutExpired as exc:
        raise VoiceTranscodeError(f"ffmpeg timeout after 30s (input size {len(ogg)} bytes)") from exc
    except OSError as exc:
        # Например, ffmpeg бинарь есть, но не запускается (нет прав).
        raise VoiceTranscodeError(f"ffmpeg failed to start: {exc}") from exc

    if proc.returncode != 0:
        stderr = (proc.stderr or b"").decode("utf-8", errors="replace").strip()
        raise VoiceTranscodeError(
            f"ffmpeg returned {proc.returncode} for input of {len(ogg)} bytes: "
            f"{stderr[-400:] if stderr else '<no stderr>'}"
        )

    pcm = proc.stdout
    if not pcm:
        raise VoiceTranscodeError(
            f"ffmpeg produced empty output for input of {len(ogg)} bytes " "(likely corrupt OGG/Opus stream)"
        )

    if len(pcm) % TARGET_SAMPLE_WIDTH_BYTES != 0:
        # По идее ffmpeg всегда выдаёт целое число сэмплов s16le.
        # Если нет — что-то очень не так с бинарем.
        raise VoiceTranscodeError(
            f"ffmpeg output length {len(pcm)} is not aligned to "
            f"{TARGET_SAMPLE_WIDTH_BYTES} bytes (s16le framing broken)"
        )

    pcm_duration_ms = len(pcm) // (TARGET_SAMPLE_RATE_HZ * TARGET_CHANNELS * TARGET_SAMPLE_WIDTH_BYTES // 1000)

    if probe_duration_ms is not None and abs(pcm_duration_ms - probe_duration_ms) > DURATION_TOLERANCE_MS:
        # Не фатально — ffmpeg resample может сильно разойтись на коротких
        # файлах или при нестандартных частотах. Логируем warning.
        logger.warning(
            "ogg_to_pcm16k: duration probe mismatch — probed %s ms, " "pcm %s ms (delta %s ms, tolerance %s ms)",
            probe_duration_ms,
            pcm_duration_ms,
            pcm_duration_ms - probe_duration_ms,
            DURATION_TOLERANCE_MS,
        )

    return pcm


def probe_ogg_duration_ms(ogg: bytes) -> Optional[int]:
    """Узнать длительность OGG через ffprobe (опционально).

    Используется только в sanity-check (см. ``ogg_to_pcm16k``). Возвращает
    ``None``, если ffprobe недоступен или не смог распарсить.

    Не бросает — это best-effort метрика. На старых ffprobe, которые
    не умеют ``-i pipe:0``, пишем во временный файл и пробим его.
    """
    ffprobe = shutil.which("ffprobe")
    if not ffprobe:
        return None

    # Сначала пробуем stdin-probe (быстро, без I/O).
    try:
        proc = subprocess.run(
            [
                ffprobe,
                "-hide_banner",
                "-loglevel",
                "error",
                "-i",
                "pipe:0",
                "-show_entries",
                "format=duration",
                "-of",
                "default=noprint_wrappers=1:nokey=1",
            ],
            input=ogg,
            capture_output=True,
            check=False,
            timeout=10,
        )
        if proc.returncode == 0:
            try:
                seconds = float((proc.stdout or b"").decode("utf-8", "replace").strip())
                return int(seconds * 1000)
            except ValueError:
                pass
    except (subprocess.TimeoutExpired, OSError) as exc:  # noqa: BLE001
        logger.debug("ffprobe stdin-probe failed: %s", exc)

    # Fallback: временный файл (нужно для старых ffprobe).
    import tempfile

    try:
        with tempfile.NamedTemporaryFile(suffix=".ogg", delete=True) as fp:
            fp.write(ogg)
            fp.flush()
            proc = subprocess.run(
                [
                    ffprobe,
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-i",
                    fp.name,
                    "-show_entries",
                    "format=duration",
                    "-of",
                    "default=noprint_wrappers=1:nokey=1",
                ],
                capture_output=True,
                check=False,
                timeout=10,
            )
            if proc.returncode != 0:
                return None
            try:
                seconds = float((proc.stdout or b"").decode("utf-8", "replace").strip())
                return int(seconds * 1000)
            except ValueError:
                return None
    except (subprocess.TimeoutExpired, OSError) as exc:  # noqa: BLE001
        logger.debug("ffprobe file-probe failed: %s", exc)
        return None
