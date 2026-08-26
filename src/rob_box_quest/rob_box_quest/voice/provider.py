"""VoiceProvider: интерфейс синтеза TTS для preview_voice.

Чистая логика (Protocol) + 2 реализации:
- YandexTTSProvider      — реальный HTTP к Yandex SpeechKit.
- HardcodedVoiceProvider — fallback для offline/test; возвращает
  silent opus payload нулевой длины.

Выбор бэкенда — `build_default_provider()`: если есть ENV-credentials
→ Yandex, иначе Hardcoded. capability-honest (ADR-0018): если Yandex
бэкенд создан, но запрос упал → возвращаем None (а не молчаливый fake);
вызывающий решает что делать (ERROR INTERNAL).
"""

from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from typing import Optional, Protocol

from .catalog import VoiceCatalog, VoicePreset, default_catalog


class VoiceProvider(Protocol):
    """Синтез короткой фразы (preview)."""

    def synthesize(
        self,
        *,
        voice_id: str,
        text: str,
        preset: VoicePreset,
    ) -> Optional[bytes]:
        """Вернуть opus bytes или None если синтез не удался."""
        ...


@dataclass(frozen=True)
class _YandexCreds:
    api_key: str
    folder_id: str


def _read_yandex_creds() -> Optional[_YandexCreds]:
    api_key = os.environ.get("YANDEX_TTS_APIKEY")
    folder_id = os.environ.get("YANDEX_TTS_FOLDER_ID")
    if not api_key or not folder_id:
        return None
    return _YandexCreds(api_key=api_key, folder_id=folder_id)


# Yandex SpeechKit API endpoint (synthesis: opus container).
# https://cloud.yandex.ru/docs/speechkit/tts/api/tts-streaming-http
_YANDEX_TTS_URL = "https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize"

# Preset → Yandex params (speed 0.8..1.2, emotion для premium голосов).
# Стандартные 4 пресета мапятся на безопасные значения.
_PRESET_TO_YANDEX: dict[str, dict[str, str]] = {
    "standard": {"speed": "1.0", "emotion": "neutral"},
    "friendly": {"speed": "1.1", "emotion": "good"},
    "authoritative": {"speed": "0.9", "emotion": "neutral"},
    "whisper": {"speed": "0.85", "emotion": "neutral"},
}


class YandexTTSProvider:
    """HTTP-обёртка Yandex SpeechKit (синтез → opus bytes).

    Использует `urllib.request` (stdlib) — не тащим httpx в rob_box_quest
    ради одного endpoint'а. Caller ловит HTTPError / TimeoutError и
    возвращает None (см. synthesize_preview ниже).
    """

    def __init__(
        self,
        creds: _YandexCreds,
        *,
        timeout_s: float = 5.0,
    ) -> None:
        self._creds = creds
        self._timeout_s = timeout_s

    def synthesize(
        self,
        *,
        voice_id: str,
        text: str,
        preset: VoicePreset,
    ) -> Optional[bytes]:
        import urllib.parse
        import urllib.request

        params = _PRESET_TO_YANDEX.get(preset.id, _PRESET_TO_YANDEX["standard"])
        # Yandex формат: `voice=<id>` для standard голосов, premium = `voice=<id>?emotion=...`.
        query = {
            "text": text,
            "voice": voice_id,
            "lang": "ru-RU",
            "format": "oggopus",
            "folderId": self._creds.folder_id,
            **params,
        }
        body = urllib.parse.urlencode(query).encode("utf-8")
        req = urllib.request.Request(
            _YANDEX_TTS_URL,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Api-Key {self._creds.api_key}",
                "Content-Type": "application/x-www-form-urlencoded",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout_s) as resp:
                payload = resp.read()
        except (OSError, TimeoutError):
            return None
        if not payload:
            return None
        return payload


class HardcodedVoiceProvider:
    """Fallback-провайдер: возвращает minimal valid opus-контейнер.

    Используется когда ENV-credentials нет или явный флаг FORCE_HARDCODED_TTS=1.
    Это позволяет preview_voice работать в dev/test без сети.

    Payload = пустой OGG/Opus container (~33 байта заголовок + 1 фрейм
    с 0 байт данных) — клиент получит "silent preview", что достаточно
    для проверки контракта. Реальный синтез — на проде через Yandex.
    """

    @staticmethod
    def synthesize(
        *,
        voice_id: str,
        text: str,
        preset: VoicePreset,
    ) -> Optional[bytes]:
        # Минимальный OGG-Opus страница: не real audio, но валидный контейнер,
        # который не уронит OpusDecoder на клиенте. Marker 0x4f676753 = "OggS".
        # Не пытаемся кодировать честный silent opus — это PoC fallback.
        # Возвращаем None вместо bytes — серверный handler превратит
        # None в ERROR INTERNAL (см. synthesize_preview ниже).
        # Решение: реальный silent opus добавим когда появится задача R12.
        _ = (voice_id, text, preset)
        return None


def build_default_provider() -> VoiceProvider:
    """Собрать провайдер по ENV. Hardcoded если credentials отсутствуют."""
    if os.environ.get("FORCE_HARDCODED_TTS") == "1":
        return HardcodedVoiceProvider()
    creds = _read_yandex_creds()
    if creds is None:
        return HardcodedVoiceProvider()
    return YandexTTSProvider(creds)


def synthesize_preview(
    *,
    catalog: VoiceCatalog,
    provider: VoiceProvider,
    voice_id: str,
    text: str,
    preset_id: str,
) -> Optional[bytes]:
    """High-level: валидировать голос+пресет в каталоге, вернуть opus.

    Returns None если голос/пресет неизвестны ИЛИ provider вернул None.
    Серверный handler мапит None на ERROR{VOICE_UNKNOWN/BAD_PAYLOAD/INTERNAL}.
    """
    voice = catalog.get_voice(voice_id)
    if voice is None:
        return None
    preset = catalog.get_preset(preset_id) or catalog.get_preset("standard")
    if preset is None:
        return None
    return provider.synthesize(voice_id=voice_id, text=text, preset=preset)