"""VoiceCatalog: статический список голосов + пресетов для TTS picker.

Источник истины:
- задача t_7eba64d9 §1 ("Если voice-pipeline недоступен → fallback
  hardcoded список (4 голоса как в задаче §4)")
- Yandex SpeechKit: yandex-voices (https://cloud.yandex.ru/services/speechkit),
  голоса: alena, anton, ermil, filipp, jane, madirus, omazh, zahar, erkanyavas,
  gulnara и т.д.

Чтобы не плодить анти-паттерн (тащить httpx сюда для единственного
запроса), реальные credentials читаются из ENV и парсятся в Voice-объекты
ПРЯМО в `from_env()`. Если ENV нет — возвращается hardcoded fallback.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class Voice:
    """Один голос из каталога (lowercase snake_case id per parent decision)."""

    id: str  # "anton", "alena", "russian_reliable_man"
    name: str  # human-readable: "Anton", "Алёна"
    gender: str  # "male" | "female"
    language: str  # "ru-RU" | "en-US"
    description: str = ""

    def to_dict(self) -> dict[str, str]:
        return {
            "id": self.id,
            "name": self.name,
            "gender": self.gender,
            "language": self.language,
            "description": self.description,
        }


@dataclass(frozen=True)
class VoicePreset:
    """Пресет модификации голоса (speed/pitch)."""

    id: str  # "standard" | "friendly" | "authoritative" | "whisper"
    name: str
    description: str

    def to_dict(self) -> dict[str, str]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
        }


# Пресеты — фиксированный набор (см. задача §1).
PRESETS: tuple[VoicePreset, ...] = (
    VoicePreset(
        id="standard",
        name="Standard",
        description="Default speed/pitch, нейтральная подача",
    ),
    VoicePreset(
        id="friendly",
        name="Friendly",
        description="Чуть быстрее, теплее по pitch — для small-talk",
    ),
    VoicePreset(
        id="authoritative",
        name="Authoritative",
        description="Ниже pitch, медленнее — для команд и инструкций",
    ),
    VoicePreset(
        id="whisper",
        name="Whisper",
        description="Тихо, с паузами — для личных/медицинских ответов",
    ),
)


# Hardcoded fallback (Yandex SpeechKit на 2026-08): 4 голоса ровно как в
# задаче §4. Если ENV-конфиг пуст — это и есть каталог.
_FALLBACK_VOICES: tuple[Voice, ...] = (
    Voice(
        id="anton",
        name="Anton",
        gender="male",
        language="ru-RU",
        description="Yandex SpeechKit, мужской, тёплый баритон",
    ),
    Voice(
        id="alena",
        name="Алёна",
        gender="female",
        language="ru-RU",
        description="Yandex SpeechKit, женский, нейтральный",
    ),
    Voice(
        id="russian_reliable_man",
        name="Russian Reliable Man",
        gender="male",
        language="ru-RU",
        description="Yandex SpeechKit premium, мужской, спокойный",
    ),
    Voice(
        id="zahar",
        name="Zahar",
        gender="male",
        language="ru-RU",
        description="Yandex SpeechKit, мужской, мягкий тенор",
    ),
)


# ENV формат: YANDEX_VOICES_JSON = JSON-список Voice-объектов.
# Не используем более сложный формат чтобы не раздувать config — за
# лимитом 4 голоса в fallback список можно перебить через ENV без кода.
_ENV_VAR_VOICES = "YANDEX_VOICES_JSON"
_ENV_VAR_APIKEY = "YANDEX_TTS_APIKEY"
_ENV_VAR_FOLDER = "YANDEX_TTS_FOLDER_ID"


class VoiceCatalog:
    """Immutable каталог голосов + пресетов.

    Используется двумя способами:
    - `VoiceCatalog.default()` — hardcoded fallback.
    - `VoiceCatalog.from_env()` — читает ENV JSON если задан, иначе default.
    """

    def __init__(
        self,
        voices: tuple[Voice, ...],
        presets: tuple[VoicePreset, ...] = PRESETS,
    ) -> None:
        if len(voices) < 4:
            # Acceptance criterion: "list_voices возвращает ≥4 голосов".
            # Это требование выполняется в `default()`, но если кто-то
            # соберёт каталог из ENV-конфиг с <4 — явно скажем.
            raise ValueError(
                f"VoiceCatalog требует ≥4 голосов (got {len(voices)})"
            )
        self._voices = voices
        self._by_id: dict[str, Voice] = {v.id: v for v in voices}
        self._presets = presets

    @classmethod
    def default(cls) -> "VoiceCatalog":
        return cls(_FALLBACK_VOICES)

    @classmethod
    def from_env(cls) -> "VoiceCatalog":
        """Прочитать ENV JSON, fallback на default."""
        import json

        raw = os.environ.get(_ENV_VAR_VOICES)
        if not raw:
            return cls.default()
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            # Не молчим: capability-honest (ADR-0018) — если ENV битый,
            # возвращаем default и логируем warning.
            import logging

            logging.getLogger(__name__).warning(
                "VoiceCatalog.from_env: bad JSON in %s, falling back to default",
                _ENV_VAR_VOICES,
            )
            return cls.default()
        if not isinstance(data, list):
            return cls.default()
        voices: list[Voice] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            try:
                voices.append(
                    Voice(
                        id=str(item["id"]),
                        name=str(item.get("name", item["id"])),
                        gender=str(item.get("gender", "neutral")),
                        language=str(item.get("language", "ru-RU")),
                        description=str(item.get("description", "")),
                    )
                )
            except KeyError:
                continue
        if len(voices) < 4:
            return cls.default()
        return cls(tuple(voices))

    @property
    def voices(self) -> tuple[Voice, ...]:
        return self._voices

    @property
    def presets(self) -> tuple[VoicePreset, ...]:
        return self._presets

    def get_voice(self, voice_id: str) -> Optional[Voice]:
        return self._by_id.get(voice_id)

    def get_preset(self, preset_id: str) -> Optional[VoicePreset]:
        for p in self._presets:
            if p.id == preset_id:
                return p
        return None

    def to_list_payload(self) -> dict[str, list[dict[str, str]]]:
        """Payload для JSON_EVENT{type:voices_list}."""
        return {
            "voices": [v.to_dict() for v in self._voices],
            "presets": [p.to_dict() for p in self._presets],
        }


# Модуль-уровень singleton (lazy).
_default_catalog: Optional[VoiceCatalog] = None


def default_catalog() -> VoiceCatalog:
    """Lazy singleton default-каталога (ENV или fallback)."""
    global _default_catalog
    if _default_catalog is None:
        _default_catalog = VoiceCatalog.from_env()
    return _default_catalog