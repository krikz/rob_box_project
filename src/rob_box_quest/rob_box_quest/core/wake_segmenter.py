"""DEPRECATED shim — переезд на ``rob_box_core.speech_segmentation`` (issue #2199).

Исторически wake-канал шлема сегментировал кадры 20 мс → фразы отдельный
модуль ``core/wake_segmenter.py`` (issue #2135). В рамках issue #2199 вся
логика «фраза кончилась» переехала в :mod:`rob_box_core.speech_segmentation`
и конфигурируется через :class:`~rob_box_core.speech_segmentation.SpeechSegmentationConfig`.

Этот файл остался как **обратно-совместимый shim**:

* ``WakePhraseSegmenter`` → :class:`rob_box_core.speech_segmentation.PhraseSegmenter`
  сконструированный с ``DEFAULT_WAKE_CONFIG``.
* ``WAKE_PHRASE_GAP_TIMEOUT_S`` / ``WAKE_PHRASE_MAX_BYTES`` / ``WAKE_PHRASE_MIN_BYTES``
  / ``WAKE_PHRASE_MAX_S`` / ``WAKE_PHRASE_MIN_S`` / ``WAKE_BYTES_PER_S`` /
  ``WAKE_SAMPLE_RATE_HZ`` → соответствующие атрибуты/свойства
  :class:`~rob_box_core.speech_segmentation.SpeechSegmentationConfig` или
  :data:`~rob_box_core.speech_segmentation.BYTES_PER_S` / ``SAMPLE_RATE_HZ``.

Каждое обращение к символам из этого модуля пишет :class:`DeprecationWarning`
— при первой возможности переведите потребителей на
``rob_box_core.speech_segmentation`` напрямую и удалите этот файл.

Зачем этот shim вообще
----------------------
Чтобы существующие юнит-тесты ``test_quest_bridge_wake_segmentation.py``,
``test_quest_bridge_wake_audio_observability.py`` и ``test_wake_segmenter.py``
продолжили собирать те же символы, что и раньше — без правки их импортов.
Перевод тестов на новый модуль — следующий шаг (см. ``tools/migrate_wake_*.py``
в TODO-листе #2199, если будет).
"""

from __future__ import annotations

import warnings as _warnings

from rob_box_core.speech_segmentation import (  # noqa: F401  (re-export)
    BYTES_PER_S,
    DEFAULT_WAKE_CONFIG,
    PhraseSegmenter as _PhraseSegmenter,
    SAMPLE_RATE_HZ,
)


# ── Контракт старого wake_segmenter.py (issue #2135) ──────────────────

# Исторические имена констант.
WAKE_SAMPLE_RATE_HZ: int = SAMPLE_RATE_HZ
WAKE_BYTES_PER_S: int = BYTES_PER_S
WAKE_PHRASE_GAP_TIMEOUT_S: float = DEFAULT_WAKE_CONFIG.gap_timeout_s
WAKE_PHRASE_MAX_S: float = DEFAULT_WAKE_CONFIG.max_phrase_s
WAKE_PHRASE_MIN_S: float = DEFAULT_WAKE_CONFIG.min_phrase_s
WAKE_PHRASE_MAX_BYTES: int = DEFAULT_WAKE_CONFIG.max_phrase_bytes
WAKE_PHRASE_MIN_BYTES: int = DEFAULT_WAKE_CONFIG.min_phrase_bytes


def __getattr__(name: str):  # PEP 562 — ленивый алиас
    """Резолвим ``WAKE_*`` / ``WakePhraseSegmenter`` через rob_box_core.

    Срабатывает только когда атрибут не найден обычным образом — например,
    при ``from rob_box_quest.core.wake_segmenter import WAKE_PHRASE_GAP_TIMEOUT_S``
    Python сначала пытается найти ``WAKE_PHRASE_GAP_TIMEOUT_S`` в глобальной
    области модуля, и только если не нашёл — зовёт ``__getattr__``. Поэтому
    явные константы выше (с теми же именами) **перебивают** эту функцию.
    """
    if name == "WakePhraseSegmenter":
        _warnings.warn(
            "rob_box_quest.core.wake_segmenter.WakePhraseSegmenter is deprecated; "
            "use rob_box_core.speech_segmentation.PhraseSegmenter "
            "with DEFAULT_WAKE_CONFIG instead (issue #2199).",
            DeprecationWarning,
            stacklevel=2,
        )
        # Возвращаем фабрику-класс, совместимую по сигнатуре
        # (``WakePhraseSegmenter()`` без аргументов — старый API).
        class _WakePhraseSegmenterCompat(_PhraseSegmenter):
            def __init__(self) -> None:
                super().__init__(DEFAULT_WAKE_CONFIG)

        return _WakePhraseSegmenterCompat
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


_warnings.warn(
    "rob_box_quest.core.wake_segmenter is deprecated and will be removed; "
    "import rob_box_core.speech_segmentation instead (issue #2199).",
    DeprecationWarning,
    stacklevel=2,
)


__all__ = [
    "WAKE_SAMPLE_RATE_HZ",
    "WAKE_BYTES_PER_S",
    "WAKE_PHRASE_GAP_TIMEOUT_S",
    "WAKE_PHRASE_MAX_BYTES",
    "WAKE_PHRASE_MIN_BYTES",
    "WAKE_PHRASE_MAX_S",
    "WAKE_PHRASE_MIN_S",
    "WakePhraseSegmenter",
]
