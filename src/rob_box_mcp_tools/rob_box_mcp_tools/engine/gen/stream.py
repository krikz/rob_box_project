"""Ранний звук генерации: ``stream_audio_url`` до ``success`` (ADR-0158 §2.1, ускорение кавера/песни/инструментала).

Замер воркера PR-8 14.10 (task ``f5aa62a5…``, V6 120 с): на стадии ``text`` (≈ 32 с от заказа) ``stream_audio_url``
отдал весь трек 2.55 МБ за 6 с, ``success`` — на 50 с; после ``success`` стрим — те же байты, что ``audio_url``.
Но полнота стрима не обещана: он мог бы отдать только сгенерированную часть. Поэтому стрим принимается **только**
проверенным: декод в wav частоты сервера (``audio.server_wav`` — одна реализация декода) и длина не меньше
ожидаемой минус допуск. Иначе — причина в лог и честное ожидание ``success``, как без стрима.

Здесь только знание «полон ли ранний звук»; заказ, кэш и учёт — ``pipeline.generate_into_cache``.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from .audio import SERVER_WAV, AudioUnavailable, server_wav, wav_seconds
from .provider import GenJob, GenTrack

#: Допуск длины, когда длина известна точно (``duration`` ответа или заказа): Suno отдаёт 119.48–120.0 на 120,
#: 179.16 на 180, 89 на 90 (ADR-0158 §1.3).
STREAM_TOLERANCE_S = 3.0
#: Песня на вокал (``add-instrumental``): длину задаёт вокал, Suno укорачивает — 74.8 с на 82 с вокала (§1.3, № 2).
SONG_MIN_FRACTION = 0.85
#: Потолок скачивания стрима: целиком он пришёл за 6 с; дольше — значит идёт вровень с генерацией, ждём ``success``.
STREAM_DOWNLOAD_TIMEOUT_S = 20.0
#: Стадии, на которых пробуется ранний звук (по попытке на стадию): стрим на ``text``, ``audio_url`` 1-го на ``first``;
#: на ``complete`` — первый вариант мосту до скачивания второго.
EARLY_STAGES = ("text", "first", "complete")


class StreamIncomplete(Exception):
    """Ранний звук не принят: ``reason`` — для лога (``short``/``decode``/``unknown_length``/``download``)."""

    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason


@dataclass(frozen=True)
class EarlyAudio:
    """Проверенный ранний звук одного варианта: mp3 как пришёл и уже декодированный wav сервера (во временном)."""

    track: GenTrack
    url: str
    blob: bytes
    wav_path: str
    seconds: float


def media_seconds(path: str, run: Callable[..., Any] = subprocess.run) -> Optional[float]:
    """Длина медиафайла по ``ffprobe``; ``None`` — не узнать."""
    if not path:
        return None
    try:
        out = run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path],
                  check=True, timeout=10.0, capture_output=True)
        return float((out.stdout or b"").decode("ascii", "replace").strip())
    except (OSError, subprocess.SubprocessError, ValueError):
        return None


def minimum_seconds(job: GenJob, track: GenTrack, run: Callable[..., Any] = subprocess.run) -> float:
    """Нижняя граница длины полного трека: ``duration`` ответа → заказанная длина → длина вокала песни."""
    if track.duration_s > 0:
        return track.duration_s - STREAM_TOLERANCE_S
    if job.kind in ("instrumental", "cover"):
        return float(job.duration_s) - STREAM_TOLERANCE_S
    vocal = media_seconds(job.audio_path, run)
    if vocal:
        return vocal * SONG_MIN_FRACTION
    raise StreamIncomplete("unknown_length", f"{job.kind}: длина полного трека неизвестна")


def candidate(tracks: "tuple[GenTrack, ...]", tried: "set[str]") -> Optional["tuple[GenTrack, str]"]:
    """Первый вариант с ещё не пробованным URL: готовый ``audio_url`` важнее стрима."""
    for track in tracks:
        for url in (track.audio_url, track.stream_url):
            if url and url not in tried:
                return track, url
    return None


def verify(blob: bytes, minimum_s: float, run: Callable[..., Any] = subprocess.run) -> "tuple[str, float]":
    """mp3 → wav сервера во временном каталоге; длина ≥ ``minimum_s`` → ``(путь wav, длина)``, иначе исключение."""
    tmp = Path(tempfile.mkdtemp(prefix="rbx-stream-"))
    mp3 = tmp / "track.mp3"
    mp3.write_bytes(blob)
    try:
        wav = server_wav(str(mp3), run=run)
        seconds = wav_seconds(wav)
    except AudioUnavailable as exc:
        shutil.rmtree(tmp, ignore_errors=True)
        raise StreamIncomplete("decode", str(exc)) from exc
    if seconds < minimum_s:
        shutil.rmtree(tmp, ignore_errors=True)
        raise StreamIncomplete("short", f"{seconds:.1f} с < {minimum_s:.1f} с ({len(blob)} байт)")
    return wav, seconds


def adopt_wav(wav_path: str, mp3_path: str) -> None:
    """Декодированный wav — рядом с mp3 кэша, не старше его (``server_wav`` не декодирует второй раз)."""
    target = Path(mp3_path).with_name(SERVER_WAV)
    shutil.move(wav_path, target)
    os.utime(target, None)
    shutil.rmtree(Path(wav_path).parent, ignore_errors=True)


def discard(early: EarlyAudio) -> None:
    shutil.rmtree(Path(early.wav_path).parent, ignore_errors=True)


__all__ = ["EARLY_STAGES", "EarlyAudio", "SONG_MIN_FRACTION", "STREAM_DOWNLOAD_TIMEOUT_S", "STREAM_TOLERANCE_S",
           "StreamIncomplete", "adopt_wav", "candidate", "discard", "media_seconds", "minimum_seconds", "verify"]
