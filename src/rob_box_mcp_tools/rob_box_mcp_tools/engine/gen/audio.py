"""Трек кэша → ``AudioTrack`` деки (ADR-0158 §2.4, PR-2).

mp3 кэша один раз декодируется ``ffmpeg`` в ``track.16k.wav`` рядом с ним — частота и каналы сервера
(``knowledge.SC_SERVER_RATE_HZ``/``AUDIO_CHANNELS``): спайк PR-2 на Vision Pi — декод 0.7–0.9 с на 3–4 мин,
буфер в scsynth 31 МБ вместо 85 МБ у mp3 и загрузка 0.10 с вместо 0.41 с. Каталог кэша scsynth видит по тому же
пути (``docker-compose.yaml``: ``/data/music_library:ro`` у ``supercollider``). Длительность — из самого wav,
а не из строки библиотеки. Удаление трека из библиотеки уносит каталог вместе с wav.
"""

from __future__ import annotations

import os
import subprocess
import wave
from pathlib import Path
from typing import Any, Callable, Mapping

from rob_box_music import knowledge as kn
from rob_box_music.model import AudioTrack

#: Имя декодированного файла рядом с ``track.mp3``.
SERVER_WAV = f"track.{kn.SC_SERVER_RATE_HZ // 1000}k.wav"
#: Потолок декода: 4 мин на Pi — меньше секунды (спайк PR-2); дольше — что-то сломано.
DECODE_TIMEOUT_S = 60.0


class AudioUnavailable(RuntimeError):
    """Файл трека не превратился в звук деки: ``reason`` — код для лога и фразы."""

    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


def server_wav(mp3_path: str, run: Callable[..., Any] = subprocess.run) -> str:
    """Путь к wav частоты сервера; нет или старше mp3 — декодировать (во временный файл, потом переименовать)."""
    mp3 = Path(mp3_path)
    wav = mp3.with_name(SERVER_WAV)
    if not mp3.exists():
        raise AudioUnavailable("no_file", f"нет файла {mp3}")
    if wav.exists() and wav.stat().st_mtime >= mp3.stat().st_mtime:
        return str(wav)
    tmp = wav.with_name(wav.name + ".part")
    cmd = ["ffmpeg", "-v", "error", "-y", "-i", str(mp3), "-ar", str(kn.SC_SERVER_RATE_HZ),
           "-ac", str(kn.AUDIO_CHANNELS), "-c:a", "pcm_s16le", "-f", "wav", str(tmp)]
    try:
        run(cmd, check=True, timeout=DECODE_TIMEOUT_S, capture_output=True)
        os.replace(tmp, wav)
    except (OSError, subprocess.SubprocessError) as exc:
        stderr = getattr(exc, "stderr", b"") or b""
        detail = stderr.decode("utf-8", "replace").strip()[-200:] if isinstance(stderr, bytes) else str(stderr)
        raise AudioUnavailable("decode_failed", f"{type(exc).__name__}: {detail or exc}") from exc
    return str(wav)


def wav_seconds(path: str) -> float:
    """Длительность wav по заголовку."""
    try:
        with wave.open(path, "rb") as handle:
            return handle.getnframes() / float(handle.getframerate())
    except (OSError, wave.Error, ZeroDivisionError) as exc:
        raise AudioUnavailable("bad_wav", f"{path}: {exc}") from exc


def audio_track(info: Mapping[str, Any], track_id: str, run: Callable[..., Any] = subprocess.run) -> AudioTrack:
    """Строка библиотеки (``get_track_info``) → ``AudioTrack`` на wav сервера. ``track_id`` — id запуска на деке
    (свой на каждый запуск: ``started`` ждут по нему, повтор того же трека не должен поймать старое событие)."""
    wav = server_wav(str(info["path"]), run=run)
    bpm, lufs = info.get("bpm_nominal"), info.get("loudness_lufs")
    return AudioTrack(track_id=track_id, path=wav, duration_s=round(wav_seconds(wav), 3),
                      bpm_nominal=int(bpm) if bpm else None, loudness_lufs=float(lufs) if lufs is not None else None)


__all__ = ["AudioUnavailable", "DECODE_TIMEOUT_S", "SERVER_WAV", "audio_track", "server_wav", "wav_seconds"]
