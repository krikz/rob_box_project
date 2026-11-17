"""Адаптер Yandex SpeechKit v3 (gRPC) на ``BaseTTSProvider`` — ADR-0160 §2.5, #4240.

Раньше синтез Yandex жил методами ``tts_node`` (``_synthesize_yandex``, ``_synthesize_yandex_single``,
``_synthesize_yandex_single_with_latency``, ``_chunk_text``). Здесь — единственная реализация: нарезка под лимит
v3, retry-halve на «Too long text», gRPC-вызов, склейка аудио.

Флаги сервиса (``capabilities``): цифры читает сам; ударения не ставит надёжно (разметка «+» — PR-7 по замеру);
паузы такта метка ``sil<[мс]>`` умеет, но адаптер их не ставит: рэп на Yandex — без такта, строкой в логе (решение
Шифу 30.10, В3; ``pause_markup=False`` честно, :mod:`.beat`).
"""

from __future__ import annotations

import asyncio
import io
import time
import wave
from typing import Any, AsyncIterator, Optional

import grpc
import numpy as np

from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat, TTSSettings
from rob_box_llm.tts_provider_base import BaseTTSProvider, TTSCapabilities, TTSVoice

from ..tts_chunking import (
    CHUNK_LIMITS,
    DEFAULT_MAX_RETRIES,
    MIN_CHUNK_CHARS,
    SENTENCE_SENTINELS,
    split_text,
    synthesize_with_retry,
)
from ..tts_engines import build_yandex_request, make_yandex_stub, yandex_collect_audio
from .beat import without_beat

#: Частота, с которой нода играет ответ Yandex (контейнер WAV v3; так было в ``tts_node``).
YANDEX_SAMPLE_RATE: int = 22050


def chunk_text(
    text: str,
    max_chars: int = CHUNK_LIMITS["yandex_grpc_v3"],
    sentence_separators: str = SENTENCE_SENTINELS,
) -> list[str]:
    """Нарезка под лимит Yandex v3 — общий чанкер :func:`tts_chunking.split_text` с лимитом Yandex по умолчанию.

    >>> chunk_text("Короткий текст.")
    ['Короткий текст.']
    """
    return split_text(text, max_chars, sentence_separators=sentence_separators)


class YandexTTSProvider(BaseTTSProvider):
    """Yandex SpeechKit v3: ``prepare`` + синхронный :meth:`synthesize_np` (поток синтеза ноды) и async-контракт."""

    name = "yandex"
    sample_rate = YANDEX_SAMPLE_RATE

    def __init__(
        self,
        api_key: str,
        stub: Any,
        *,
        logger: Any,
        channel: Any = None,
        default_voice: str = "anton",
        chunk_max_chars: int = CHUNK_LIMITS["yandex_grpc_v3"],
        chunk_max_retries: int = DEFAULT_MAX_RETRIES,
        chunk_min_chars: int = MIN_CHUNK_CHARS,
    ) -> None:
        self._api_key = api_key
        self._stub = stub
        self._channel = channel
        self._logger = logger
        self._default_voice = default_voice
        self._chunk_max_chars = max(1, int(chunk_max_chars))
        self._chunk_max_retries = max(1, int(chunk_max_retries))
        self._chunk_min_chars = max(1, int(chunk_min_chars))

    @classmethod
    def connect(cls, api_key: str, *, logger: Any, **kwargs: Any) -> "YandexTTSProvider":
        """Открыть канал gRPC и собрать адаптер; ошибка канала не глотается."""
        channel, stub = make_yandex_stub()
        return cls(api_key, stub, logger=logger, channel=channel, **kwargs)

    # ---- флаги и подготовка текста ----

    def capabilities(self) -> TTSCapabilities:
        """Контейнер WAV, без SSML; цифры читает сам (ADR-0160 §2.5)."""
        return TTSCapabilities(audio_format_pcm=True, reads_digits=True)

    def prepare(self, text: str, beat_s: float = 0.0) -> str:
        """Текст как есть: цифры и ударения — сервису; такт рэпа ``beat_s`` не размечается (строка лога)."""
        return without_beat(self.name, text, beat_s, self._logger.info)

    def _build_request_payload(
        self, text: str, settings: TTSSettings, voice_meta: Optional[TTSVoice]
    ) -> dict[str, Any]:
        """Поля ``UtteranceSynthesisRequest`` (сам запрос строит ``tts_engines.build_yandex_request``)."""
        return {
            "text": text,
            "voice": settings.voice or self._default_voice,
            "speed": settings.speed if settings.speed is not None else 1.0,
        }

    # ---- синтез ----

    def synthesize_np(self, text: str, *, voice: Optional[str] = None, speed: float = 1.0) -> np.ndarray:
        """Синтез → float32 mono −1..1 (все чанки склеены без пауз, #931).

        Длинный текст (≥ 2 лимита) режется на чанки без retry: упал любой — ошибка наверх, нода идёт к следующему
        провайдеру цепочки (#929). Короткий — retry-halve на «Too long text» (#937).
        """
        if not self._stub:
            raise Exception("Yandex gRPC stub не инициализирован")
        voice = voice or self._default_voice
        if len(text) >= max(1, 2 * self._chunk_max_chars):
            audio_results = []
            chunks = chunk_text(text, max_chars=self._chunk_max_chars)
            for idx, chunk in enumerate(chunks, start=1):
                t0 = time.monotonic()
                segment, rate = self._synthesize_one(chunk, speed, voice)
                self._logger.info(
                    f"⏱️ Yandex chunk {idx}/{len(chunks)}: {len(chunk)} chars → "
                    f"{(time.monotonic() - t0) * 1000.0:.0f} ms ({len(segment)} samples)"
                )
                audio_results.append((segment, rate))
        else:
            try:
                audio_results = synthesize_with_retry(
                    text,
                    "yandex_grpc_v3",
                    lambda chunk: self._synthesize_one_logged(chunk, speed, voice),
                    max_chars=self._chunk_max_chars,
                    max_retries=self._chunk_max_retries,
                    min_chunk_chars=self._chunk_min_chars,
                    is_too_long=lambda exc: "invalid_argument" in str(exc).lower() and "too long" in str(exc).lower(),
                )
            except Exception as exc:
                if "too long" in str(exc).lower() or "yandex" in str(exc).lower():
                    raise
                raise Exception(f"Yandex gRPC error: {exc}") from exc
            if not audio_results:
                raise Exception("Yandex gRPC error: ни один chunk не вернул аудио")
        segments = [segment for segment, _ in audio_results]
        audio_np = np.concatenate(segments) if len(segments) > 1 else segments[0]
        self._logger.info(
            f"✅ Yandex gRPC v3 (ROBBOX original!): {len(audio_np)} samples, "
            f"source {audio_results[-1][1]} Hz, speed={speed}, {len(segments)} chunk(s)"
        )
        return audio_np

    def _synthesize_one(self, text: str, speed: float, voice: str) -> tuple[np.ndarray, int]:
        """Один gRPC ``UtteranceSynthesis`` → ``(audio_np, sample_rate из заголовка WAV)``."""
        try:
            audio_data = yandex_collect_audio(self._stub, self._api_key, build_yandex_request(text, voice, speed))
            if not audio_data:
                raise Exception("Пустой ответ от Yandex TTS")
            # Сырые байты (включая заголовок WAV) как PCM — оригинальный звук ROBBOX.
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            try:
                with io.BytesIO(audio_data) as wav_file, wave.open(wav_file, "rb") as wav:
                    rate = wav.getframerate()
            except Exception:  # noqa: BLE001 — заголовок не разобрался
                rate = YANDEX_SAMPLE_RATE
            return audio_np, rate
        except grpc.RpcError as e:
            raise Exception(f"Yandex gRPC error: {e.code()} - {e.details()}")
        except Exception as e:
            raise Exception(f"Yandex synthesis error: {e}")

    def _synthesize_one_logged(self, text: str, speed: float, voice: str) -> tuple[np.ndarray, int]:
        """:meth:`_synthesize_one` + латентность в лог (#931: время синтеза каждого чанка)."""
        t0 = time.monotonic()
        segment, rate = self._synthesize_one(text, speed, voice)
        self._logger.info(
            f"⏱️ Yandex synth: {len(text)} chars → {(time.monotonic() - t0) * 1000.0:.0f} ms ({len(segment)} samples)"
        )
        return segment, rate

    # ---- async-контракт TTSProvider ----

    async def synthesize(self, text: str, *, settings: TTSSettings | None = None) -> TTSAudio:
        """:meth:`prepare` + :meth:`synthesize_np` в пуле потоков → PCM int16."""
        settings = settings or TTSSettings()
        audio_np = await asyncio.to_thread(
            self.synthesize_np,
            self.prepare(text),
            voice=settings.voice,
            speed=settings.speed if settings.speed is not None else 1.0,
        )
        pcm = (np.clip(audio_np, -1.0, 1.0) * 32767.0).astype("<i2").tobytes()
        return TTSAudio(samples=pcm, sample_rate=self.sample_rate, format=TTSFormat.PCM)

    async def stream(self, text: str, *, settings: TTSSettings | None = None) -> AsyncIterator[TTSChunk]:
        """Потока у v3-обёртки нет: один терминальный кусок со всем аудио."""
        audio = await self.synthesize(text, settings=settings)
        yield TTSChunk(samples=audio.samples, sample_rate=audio.sample_rate, finish_reason="stop")

    async def aclose(self) -> None:
        """Закрыть канал gRPC (идемпотентно)."""
        channel, self._channel = self._channel, None
        if channel is not None:
            channel.close()


__all__ = ["YANDEX_SAMPLE_RATE", "YandexTTSProvider", "chunk_text"]
