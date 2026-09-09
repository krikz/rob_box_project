#!/usr/bin/env python3
"""
voice_processor.py — Convert Telegram voice messages (OGG/Opus) to text.

Supports Yandex SpeechKit REST API (primary), a local Vosk model
(fallback — same offline safety net as ``stt_node.py`` uses for the
dialogue pipeline, issue found 2026-09-03: Yandex balance ran out and
telegram voice went dead instead of degrading), and a Whisper-compatible
HTTP endpoint (opt-in via ``method="whisper"``). The Whisper endpoint URL
is configurable via the ``WHISPER_URL`` environment variable — the
default points to a generic reconcilable host so deployments can plug in
their own self-hosted or third-party STT service without any LLM
coupling.
"""

import asyncio
import io
import json
import logging
import os
from typing import Optional

import aiohttp

from .voice_transcode import VoiceTranscodeError, ogg_to_pcm16k

logger = logging.getLogger(__name__)

# Yandex STT REST endpoint (short audio, synchronous)
YANDEX_STT_URL = "https://stt.api.cloud.yandex.net/speech/v1/stt:recognize"

# Whisper-compatible STT endpoint. Override via the WHISPER_URL env var to
# point at any deployment that exposes the multipart upload contract.
_DEFAULT_WHISPER_URL = "https://whisper.local/v1/audio/transcriptions"
WHISPER_URL = os.getenv("WHISPER_URL", _DEFAULT_WHISPER_URL)

# Same default as stt_node.py (rob_box_voice) — bundled by the image build
# (see docker/vision/telegram_bot/Dockerfile) so this fallback works fully
# offline, no Yandex balance/quota/network dependency.
VOSK_MODEL_PATH = os.getenv("VOSK_MODEL_PATH", "/models/vosk-model-small-ru-0.22")
VOSK_SAMPLE_RATE_HZ = 16000

_vosk_model = None  # lazy singleton — loading is ~1s, do it once per process


async def transcribe_voice(ogg_bytes: bytes, method: str = "yandex", language: str = "ru-RU") -> Optional[str]:
    """Transcribe OGG/Opus voice message to text.

    Args:
        ogg_bytes: Raw OGG audio bytes from Telegram voice message.
        method: STT backend — "yandex" or "whisper".
        language: Language code for recognition.

    Returns:
        Transcribed text or None on failure.
    """
    if method == "yandex":
        text = await _transcribe_yandex(ogg_bytes, language)
        if text:
            return text
        # Same contract as stt_node.py: Yandex down/quota-exhausted/timeout
        # must degrade to the local Vosk model, not go silent — voice ops
        # over Telegram are the same operator-facing feature as the mic.
        logger.warning("Yandex STT unavailable, falling back to local Vosk model")
        return await _transcribe_vosk(ogg_bytes)
    elif method == "whisper":
        return await _transcribe_whisper(ogg_bytes, language)
    else:
        logger.error("Unknown STT method: %s", method)
        return None


async def _transcribe_yandex(ogg_bytes: bytes, language: str) -> Optional[str]:
    """Transcribe using Yandex SpeechKit REST API.

    Requires YANDEX_API_KEY or YANDEX_IAM_TOKEN env var.
    OGG/Opus is natively supported — no conversion needed.
    """
    api_key = os.getenv("YANDEX_API_KEY", "")
    folder_id = os.getenv("YANDEX_FOLDER_ID", "")

    if not api_key:
        logger.error("YANDEX_API_KEY not set, cannot transcribe voice")
        return None

    params = {
        "lang": language,
        "format": "oggopus",
        "folderId": folder_id,
    }
    headers = {
        "Authorization": f"Api-Key {api_key}",
    }

    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                YANDEX_STT_URL,
                params=params,
                headers=headers,
                data=ogg_bytes,
                timeout=aiohttp.ClientTimeout(total=15),
            ) as resp:
                if resp.status == 200:
                    result = await resp.json()
                    text = result.get("result", "")
                    if text:
                        logger.info("Yandex STT: '%s'", text)
                        return text
                    logger.warning("Yandex STT returned empty result")
                    return None
                else:
                    body = await resp.text()
                    logger.error("Yandex STT error %d: %s", resp.status, body)
                    return None
    except Exception as e:
        logger.error("Yandex STT request failed: %s", e)
        return None


def _load_vosk_model():
    """Load (once) and cache the local Vosk model. Returns None if unavailable.

    Mirrors stt_node.py's ``initialize_vosk`` fail-fast-with-a-clear-message
    behaviour instead of the opaque Vosk C++ "Folder does not contain model
    files" error. Runs in a worker thread (see ``_transcribe_vosk``) since
    loading is blocking I/O + CPU.
    """
    global _vosk_model
    if _vosk_model is not None:
        return _vosk_model

    if not os.path.isdir(VOSK_MODEL_PATH):
        logger.error(
            'Vosk model not found at "%s" — bundle it in the telegram-bot '
            "image (see docker/vision/telegram_bot/Dockerfile) or set "
            "VOSK_MODEL_PATH. Local STT fallback unavailable.",
            VOSK_MODEL_PATH,
        )
        return None

    try:
        from vosk import Model, SetLogLevel

        SetLogLevel(-1)  # silence Kaldi's stderr spam
        _vosk_model = Model(VOSK_MODEL_PATH)
        logger.info("Vosk model loaded from %s (STT fallback ready)", VOSK_MODEL_PATH)
        return _vosk_model
    except Exception as e:
        logger.error("Failed to load Vosk model from %s: %s", VOSK_MODEL_PATH, e)
        return None


def _recognize_vosk_sync(pcm: bytes) -> Optional[str]:
    """Blocking Vosk recognition — run via asyncio.to_thread."""
    model = _load_vosk_model()
    if model is None:
        return None

    from vosk import KaldiRecognizer

    recognizer = KaldiRecognizer(model, VOSK_SAMPLE_RATE_HZ)
    recognizer.SetWords(True)

    chunk_size = 4096
    for i in range(0, len(pcm), chunk_size):
        recognizer.AcceptWaveform(pcm[i : i + chunk_size])

    result = json.loads(recognizer.FinalResult())
    return result.get("text", "").strip() or None


async def _transcribe_vosk(ogg_bytes: bytes) -> Optional[str]:
    """Transcribe using the local Vosk model (offline fallback).

    Only Russian is supported by the bundled ``vosk-model-small-ru-0.22``
    (same model as ``stt_node.py``) — ``language`` isn't threaded through
    since Vosk models are per-language, not parametrizable at recognize time.
    """
    try:
        pcm = ogg_to_pcm16k(ogg_bytes)
    except VoiceTranscodeError as e:
        logger.error("Vosk fallback: OGG→PCM transcode failed: %s", e)
        return None

    try:
        text = await asyncio.to_thread(_recognize_vosk_sync, pcm)
    except Exception as e:
        logger.error("Vosk recognition failed: %s", e)
        return None

    if text:
        logger.info("Vosk STT (fallback): '%s'", text)
    else:
        logger.warning("Vosk STT returned empty result")
    return text


async def _transcribe_whisper(ogg_bytes: bytes, language: str) -> Optional[str]:
    """Transcribe using a Whisper-compatible HTTP endpoint.

    Requires the ``WHISPER_API_KEY`` env var (placeholder naming — set
    whatever the upstream service expects). The endpoint URL is read
    from ``WHISPER_URL`` at import time.
    """
    api_key = os.getenv("WHISPER_API_KEY", "")
    if not api_key:
        logger.error("WHISPER_API_KEY not set, cannot use Whisper STT")
        return None

    # Whisper expects a file upload
    form = aiohttp.FormData()
    form.add_field("file", ogg_bytes, filename="voice.ogg", content_type="audio/ogg")
    form.add_field("model", "whisper-1")
    # Map ru-RU -> ru for Whisper
    form.add_field("language", language.split("-")[0])

    headers = {
        "Authorization": f"Bearer {api_key}",
    }

    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                WHISPER_URL,
                headers=headers,
                data=form,
                timeout=aiohttp.ClientTimeout(total=30),
            ) as resp:
                if resp.status == 200:
                    result = await resp.json()
                    text = result.get("text", "")
                    if text:
                        logger.info("Whisper STT: '%s'", text)
                        return text
                    return None
                else:
                    body = await resp.text()
                    logger.error("Whisper API error %d: %s", resp.status, body)
                    return None
    except Exception as e:
        logger.error("Whisper API request failed: %s", e)
        return None
