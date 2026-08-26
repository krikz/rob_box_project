"""Voice TTS pipeline wrapper для rob_box_quest.

Источник истины: docs/architecture/meta-quest-api.md §5 (list_voices /
set_voice / preview_voice), карточка t_7eba64d9.

Иерархия:
- VoiceCatalog   — статический каталог (голоса + пресеты) с двумя бэкендами.
- VoiceProvider  — Protocol для синтеза (preview_voice).
- HardcodedVoiceProvider — fallback (всегда работает, возвращает silent
  opus payload для preview). Используется когда ENV не настроены.
- YandexTTSProvider — реальная интеграция с Yandex SpeechKit (HTTP POST
  к https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize). Только
  если есть YANDEX_TTS_APIKEY + YANDEX_TTS_FOLDER_ID. Любой сбой →
  fallback на HardcodedVoiceProvider (см. capability-honest, ADR-0018).

Чистая логика — без зависимостей от aiohttp / ROS / Zenoh. Тестируется
в pytest без rclpy / без сети (mock httpx).
"""

from .catalog import (
    PRESETS,
    Voice,
    VoiceCatalog,
    VoicePreset,
    default_catalog,
)
from .provider import (
    HardcodedVoiceProvider,
    VoiceProvider,
    YandexTTSProvider,
    build_default_provider,
    synthesize_preview,
)
from .voice_state import (
    PREVIEW_RATE_LIMIT,
    PREVIEW_WINDOW_S,
    SessionVoiceState,
    VoiceStateRegistry,
)

__all__ = [
    "PRESETS",
    "Voice",
    "VoiceCatalog",
    "VoicePreset",
    "VoiceProvider",
    "HardcodedVoiceProvider",
    "YandexTTSProvider",
    "default_catalog",
    "build_default_provider",
    "synthesize_preview",
    "PREVIEW_RATE_LIMIT",
    "PREVIEW_WINDOW_S",
    "SessionVoiceState",
    "VoiceStateRegistry",
]