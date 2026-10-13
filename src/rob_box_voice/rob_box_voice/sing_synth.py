#!/usr/bin/env python3
"""sing_synth.py — синтез текста песни в ФАЙЛ любым системным голосом (ADR-0158 §2.3).

Общий интерфейс для PR-7 («текущий голос + пресет → файл вокала → Suno
add-instrumental»)::

    vocal = synth_vocal_to_file("minimax", "Russian_ReliableMan", text, "vocal.mp3")

Три бэкенда (MiniMax / Yandex SpeechKit / Silero) реализуют один протокол
``SingBackend``. Пресет подачи берётся из ``tts_voice_registry.SING_PRESETS``
(единая таблица), неизвестный голос -> ``SingPresetUnknown``, а не молчаливый
откат на другой голос. Любой сбой синтеза -> ``SingSynthError(reason)``;
пустой файл не создаётся.

Модуль не зависит от ROS. Тяжёлые зависимости (grpc, torch, httpx) импортируются
лениво внутри бэкендов. Ключи читаются из окружения (``MINIMAX_API_KEY``,
``YANDEX_API_KEY``), не логируются.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import io
import logging
import os
from pathlib import Path
import shutil
import subprocess
from typing import Any, Callable, Mapping, Protocol
import wave

from .tts_chunking import CHUNK_LIMITS, split_text
from .tts_engines import (
    SILERO_MODEL_PATHS,
    build_yandex_request,
    find_silero_model_path,
    load_silero_package,
    make_yandex_stub,
    silero_apply_tts,
    yandex_collect_audio,
)
from .tts_voice_registry import SingPreset, sing_preset_for

_log = logging.getLogger(__name__)

#: Частота дискретизации вокала для Yandex/Silero (Suno нужен широкий спектр,
#: а не 16 кГц канала динамика).
VOCAL_SAMPLE_RATE = 48000
MINIMAX_SAMPLE_RATE = 44100
MINIMAX_MODEL = "speech-02-hd"


class SingSynthError(RuntimeError):
    """Синтез вокала не удался. ``reason`` — короткий код для фразы отказа."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


@dataclass(frozen=True)
class SungVocal:
    """Результат: файл вокала и его измеренные свойства."""

    path: str
    provider: str
    voice: str
    fmt: str                    # "mp3" | "wav"
    sample_rate: int
    size_bytes: int
    duration_s: float | None    # None — если измерить нечем (честно, не 0)
    preset: SingPreset


class SingBackend(Protocol):
    """Один провайдер: текст + пресет -> байты файла."""

    provider: str
    fmt: str

    def synth(self, text: str, preset: SingPreset) -> tuple[bytes, int, float | None]:
        """Вернуть ``(байты_файла, sample_rate, duration_s|None)``."""


def _wav_bytes(pcm: bytes, rate: int) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)
    return buf.getvalue()


def _run_coro(coro: Any) -> Any:
    """Выполнить корутину из синхронного кода, даже если луп уже запущен."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(coro)
    with ThreadPoolExecutor(max_workers=1) as ex:
        return ex.submit(asyncio.run, coro).result()


def probe_duration_s(data: bytes, suffix: str) -> float | None:
    """Длительность через ffprobe; None, если ffprobe нет или файл не разобран."""
    if suffix == ".wav":
        try:
            with wave.open(io.BytesIO(data), "rb") as w:
                return w.getnframes() / float(w.getframerate())
        except (wave.Error, EOFError):
            return None
    exe = shutil.which("ffprobe")
    if not exe:
        return None
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp.write(data)
        name = tmp.name
    try:
        out = subprocess.run(
            [exe, "-v", "error", "-show_entries", "format=duration",
             "-of", "default=nw=1:nk=1", name],
            capture_output=True, text=True, timeout=20, check=False,
        ).stdout.strip()
        return float(out) if out else None
    except (ValueError, subprocess.SubprocessError, OSError):
        return None
    finally:
        try:
            os.unlink(name)
        except OSError:
            pass


class MiniMaxSingBackend:
    """MiniMax T2A v2: speed/volume/pitch/emotion из пресета, mp3 44.1 кГц."""

    provider = "minimax"
    fmt = "mp3"

    def __init__(self, provider: Any = None, *, api_key: str | None = None,
                 group_id: str | None = None) -> None:
        self._provider = provider
        self._api_key = api_key if api_key is not None else os.getenv("MINIMAX_API_KEY", "")
        self._group_id = group_id if group_id is not None else os.getenv("MINIMAX_GROUP_ID", "")

    def _client(self) -> Any:
        if self._provider is not None:
            return self._provider
        if not self._api_key:
            raise SingSynthError("no_api_key", "MINIMAX_API_KEY не задан")
        from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider

        self._provider = MiniMaxTTSProvider(api_key=self._api_key, group_id=self._group_id)
        return self._provider

    def synth(self, text: str, preset: SingPreset) -> tuple[bytes, int, float | None]:
        from rob_box_llm.tts import TTSFormat, TTSSettings

        settings = TTSSettings(
            voice=preset.voice,
            model=MINIMAX_MODEL,
            language=preset.language or None,
            speed=preset.speed,
            volume=preset.volume,
            pitch=preset.pitch,
            emotion=preset.emotion,
            sample_rate=MINIMAX_SAMPLE_RATE,
            format=TTSFormat.MP3,
        )
        client = self._client()
        audio = _run_coro(client.synthesize(text, settings=settings))
        if not audio.samples:
            raise SingSynthError("empty_audio", "MiniMax вернул пустое аудио")
        raw = audio.raw if isinstance(audio.raw, dict) else {}
        length_ms = (raw.get("extra_info") or {}).get("audio_length")
        duration = float(length_ms) / 1000.0 if length_ms else None
        # provider._decode_audio подставляет 32000, если API не прислал
        # audio_sample_rate; реальная частота — запрошенная (ffprobe 13.10: 44100).
        rate = int((raw.get("data") or {}).get("audio_sample_rate") or MINIMAX_SAMPLE_RATE)
        return bytes(audio.samples), rate, duration


class YandexSingBackend:
    """Yandex SpeechKit v3: speed + role (если голос принимает), LINEAR16 48 кГц."""

    provider = "yandex"
    fmt = "wav"

    def __init__(self, stub: Any = None, *, api_key: str | None = None) -> None:
        self._stub = stub
        self._api_key = api_key if api_key is not None else os.getenv("YANDEX_API_KEY", "")

    def _get_stub(self) -> Any:
        if self._stub is not None:
            return self._stub
        if not self._api_key:
            raise SingSynthError("no_api_key", "YANDEX_API_KEY не задан")
        _channel, self._stub = make_yandex_stub()
        return self._stub

    def _one(self, text: str, preset: SingPreset) -> bytes:
        request = build_yandex_request(
            text,
            preset.voice,
            preset.speed,
            pitch_hz=float(preset.pitch) if preset.pitch else None,
            role=preset.emotion,
            raw_sample_rate=VOCAL_SAMPLE_RATE,
        )
        stub = self._get_stub()
        try:
            return yandex_collect_audio(stub, self._api_key, request)
        except Exception as exc:  # grpc.RpcError и пр. — наружу как честная причина
            code = getattr(exc, "code", None)
            detail = getattr(exc, "details", None)
            raise SingSynthError(
                "provider_error",
                f"{code() if callable(code) else type(exc).__name__}: "
                f"{detail() if callable(detail) else exc}",
            ) from exc

    def synth(self, text: str, preset: SingPreset) -> tuple[bytes, int, float | None]:
        chunks = split_text(text, CHUNK_LIMITS["yandex_grpc_v3"])
        pcm = b"".join(self._one(c, preset) for c in chunks if c.strip())
        if not pcm:
            raise SingSynthError("empty_audio", "Yandex вернул пустое аудио")
        return _wav_bytes(pcm, VOCAL_SAMPLE_RATE), VOCAL_SAMPLE_RATE, len(pcm) / 2 / VOCAL_SAMPLE_RATE


class SileroSingBackend:
    """Silero v5: только выбор голоса; speed/pitch/volume/emotion не применяются."""

    provider = "silero"
    fmt = "wav"

    def __init__(self, model: Any = None, *, model_loader: Callable[[], Any] | None = None) -> None:
        self._model = model
        self._loader = model_loader or self._load_from_disk

    @staticmethod
    def _load_from_disk() -> Any:
        import torch

        path = find_silero_model_path()
        if path is None:
            raise SingSynthError("model_missing", f"Silero v5 не найден: {SILERO_MODEL_PATHS}")
        torch.set_grad_enabled(False)
        return load_silero_package(path, torch.device("cpu"))

    def synth(self, text: str, preset: SingPreset) -> tuple[bytes, int, float | None]:
        if self._model is None:
            self._model = self._loader()
        segments = []
        for chunk in split_text(text, CHUNK_LIMITS["silero_v5"]):
            if not chunk.strip():
                continue
            try:
                segments.append(
                    silero_apply_tts(self._model, preset.voice, VOCAL_SAMPLE_RATE, text=chunk)
                )
            except Exception as exc:
                raise SingSynthError("provider_error", f"silero: {exc}") from exc
        if not segments:
            raise SingSynthError("empty_audio", "Silero не вернул аудио")
        import numpy as np

        samples = np.concatenate(segments)
        pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype("<i2").tobytes()
        return _wav_bytes(pcm, VOCAL_SAMPLE_RATE), VOCAL_SAMPLE_RATE, len(samples) / VOCAL_SAMPLE_RATE


def default_backends() -> dict[str, SingBackend]:
    """Бэкенды по умолчанию (ключи — из окружения, ленивая загрузка тяжёлого)."""
    return {
        "minimax": MiniMaxSingBackend(),
        "yandex": YandexSingBackend(),
        "silero": SileroSingBackend(),
    }


def synth_vocal_to_file(
    provider: str,
    voice: str,
    text: str,
    out_path: str | os.PathLike[str],
    *,
    backends: Mapping[str, SingBackend] | None = None,
) -> SungVocal:
    """Синтезировать ``text`` голосом ``provider:voice`` с пресетом пения в файл.

    Расширение файла подменяется на фактический формат бэкенда (mp3/wav) —
    итоговый путь в ``SungVocal.path``.

    Raises:
        SingPresetUnknown: у голоса нет пресета (никакого тихого фолбека).
        SingSynthError: пустой текст, нет бэкенда/ключа, сбой провайдера.
    """
    preset = sing_preset_for(provider, voice)
    if not text or not text.strip():
        raise SingSynthError("empty_text", "нечего петь")
    table = backends if backends is not None else default_backends()
    backend = table.get(provider)
    if backend is None:
        raise SingSynthError("no_backend", f"нет бэкенда синтеза для {provider}")
    data, rate, duration = backend.synth(text, preset)
    if not data:
        raise SingSynthError("empty_audio", f"{provider} вернул пустой файл")
    path = Path(out_path).with_suffix(f".{backend.fmt}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    if duration is None:
        duration = probe_duration_s(data, path.suffix)
    _log.info("sing_synth: %s:%s -> %s (%d bytes, %s s)", provider, voice, path.name,
              len(data), duration)
    return SungVocal(
        path=str(path), provider=provider, voice=voice, fmt=backend.fmt,
        sample_rate=rate, size_bytes=len(data), duration_s=duration, preset=preset,
    )
