#!/usr/bin/env python3
"""
voice_processor.py — Convert Telegram voice messages (OGG/Opus) to text.

Supports Yandex SpeechKit REST API (primary) and OpenAI Whisper API (fallback).
"""

import io
import logging
import os
from typing import Optional

import aiohttp

logger = logging.getLogger(__name__)

# Yandex STT REST endpoint (short audio, synchronous)
YANDEX_STT_URL = "https://stt.api.cloud.yandex.net/speech/v1/stt:recognize"

# OpenAI Whisper endpoint
WHISPER_URL = "https://api.openai.com/v1/audio/transcriptions"


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
        return await _transcribe_yandex(ogg_bytes, language)
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


async def _transcribe_whisper(ogg_bytes: bytes, language: str) -> Optional[str]:
    """Transcribe using OpenAI Whisper API.

    Requires OPENAI_API_KEY env var.
    """
    api_key = os.getenv("OPENAI_API_KEY", "")
    if not api_key:
        logger.error("OPENAI_API_KEY not set, cannot use Whisper")
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
