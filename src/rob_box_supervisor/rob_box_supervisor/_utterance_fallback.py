"""Минимальный fallback для ``rob_box_core.utterance`` (issue #2233, deploy
round-381 — avatar-supervisor restart-loop с
``ModuleNotFoundError: No module named 'rob_box_core.utterance'``).

Контекст
--------
PR #2218 / issue #2197 ([voice-vr 12]) ввёл единый сборщик SSML
``rob_box_core.utterance`` (класс :class:`Utterance` + enum :class:`Sink`).
Импорт :func:`from rob_box_core.utterance import Sink, Utterance` появился в
:mod:`rob_box_supervisor.supervisor_node`. Это нормально для develop-билдов,
которые синхронно пересобирают и ``rob_box_core``, и ``rob_box_supervisor``.

Проблема возникает в round-деплоях (``z-{e2e}/test-round-N`` ветках):
avatar-supervisor Dockerfile наследует
``ghcr.io/krikz/rob_box:voice-assistant-humble-test`` и собирает только
``rob_box_supervisor_msgs`` + ``rob_box_supervisor``. ``voice-assistant``
для тега ``-test`` обновляется из develop-билдов, но SHA-теги для
round-веток не публикуются (см. issue #1826 — anti-loop guard), а
не-SHA тег ``-test`` указывает на ПОСЛЕДНИЙ push develop-билда, который
мог быть сделан до PR c1eedb1c8. В итоге supervisor стартует в
base-image, где :mod:`rob_box_core.utterance` ещё нет, и падает с
``ModuleNotFoundError`` в первой же итерации, уходя в restart-loop.

Решение (минимально-инвазивное)
-------------------------------
Сейчас: try/except на импорт ``rob_box_core.utterance``. Если base image
содержит ``utterance.py`` — используем его (SoT, инвариант 5 ADR-0080:
«Один канал сборки реплики»). Если НЕ содержит — supervisor не должен
падать в __init__, а должен подняться в monitor-режиме и продолжить
публиковать ``/avatar/state``. Для этого мы делаем локальный минимальный
fallback с тем же публичным API (escape_xml_text + Sink + Utterance),
который собирает SSML через :func:`xml.sax.saxutils.escape` — не идеал,
но достаточно для ``<speak>...</speak>`` контракта tts_node.

Что НЕ входит в fallback
------------------------
* ``extra`` (merge с Utterance-полями через priority) — не критично,
  supervisor сейчас его не использует.
* ``ALLOWED_PRIORITIES`` / ``DEFAULT_PRIORITY`` / ``PRIORITY_*`` —
  supervisor не валидирует priority на своей стороне, tts_node сам
  фильтрует.
* Полный набор dataclass-полей ``Utterance`` — добавляем только те, что
  использует :func:`AvatarSupervisor._publish_avatar_tts` и
  :func:`AvatarSupervisor._publish_grip_tts` (``text``, ``sink``,
  ``voice``, ``language``, ``priority``).

Когда фикс должен быть удалён
------------------------------
Когда round-ветки начнут использовать SHA-pinned теги ИЛИ develop-build
будет триггерить ``voice-assistant`` rebuild на каждый push в develop.
Это потребует изменения процесса (issue #1826 + новый) и обсуждения с
товарищем Шифу. До того момента fallback держит supervisor живым при
рассинхронизации тегов в registry.

Тесты
-----
``src/rob_box_supervisor/test/unit/test_utterance_fallback.py`` —
минимальный контракт (escape, Sink enum, Utterance.to_request).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional
from xml.sax.saxutils import escape as _xml_escape


class Sink(str, Enum):
    """Адресат реплики (ADR-0080 §2.3 инвариант 6b).

    Зеркало :class:`rob_box_core.utterance.Sink` для случая, когда
    :mod:`rob_box_core.utterance` недоступен в base image. Значения строк
    совпадают с SoT.
    """

    SPEAKERS = "speakers"
    HEADSET = "headset"
    PREVIEW = "preview"


DEFAULT_PRIORITY = "normal"


def escape_xml_text(text: str) -> str:
    """Экранировать ``&``, ``<``, ``>`` для вставки в SSML.

    Использует :func:`xml.sax.saxutils.escape` с ``entities={}`` — это
    эквивалентно ``str.maketrans``-варианту из SoT (только три символа:
    ``&``, ``<``, ``>``; апострофы/кавычки для текстового узла SSML
    безопасны).
    """
    return _xml_escape(text or "", entities={})


@dataclass
class Utterance:
    """Минимальный fallback-аналог :class:`rob_box_core.utterance.Utterance`.

    Контракт :py:meth:`to_request` совместим с SoT для полей, которые
    использует :mod:`rob_box_supervisor.supervisor_node` (text, sink,
    voice, language, priority, emotion, speech_id, ssml).

    Если ``rob_box_core.utterance.Utterance`` импортируется успешно —
    supervisor использует его. Этот класс живёт только для fallback-пути.
    """

    text: str
    sink: Sink = Sink.SPEAKERS
    priority: Optional[str] = None
    voice: Optional[str] = None
    language: Optional[str] = None
    emotion: Optional[str] = None
    speech_id: Optional[str] = None
    extra: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.sink, Sink):
            # Разрешаем str-значения, чтобы зовущие с JSON-литералами
            # (``sink="speakers"``) не падали — Sink-Enum хранит те же строки.
            self.sink = Sink(self.sink)
        if self.emotion is None:
            self.emotion = "neutral"

    @property
    def ssml(self) -> str:
        """Собранный и экранированный SSML — ``<speak>...</speak>``."""
        return f"<speak>{escape_xml_text(self.text or '')}</speak>"

    def to_request(self) -> dict[str, Any]:
        """Словарь для ``json.dumps`` в payload топика.

        Гарантии (см. SoT docstring):

        * ``ssml`` — единственный источник тегов ``<speak>``, экранирован;
        * ``sink`` — строка из :class:`Sink`;
        * ``priority`` — строка (или ``"normal"`` по умолчанию);
        * ``emotion`` — строка (или ``"neutral"`` по умолчанию);
        * ``voice``/``language``/``speech_id`` присутствуют, только если заданы.
        """
        payload: dict[str, Any] = {
            "ssml": self.ssml,
            "sink": self.sink.value,
            "priority": self.priority or DEFAULT_PRIORITY,
            "emotion": self.emotion,
        }
        if self.voice is not None:
            payload["voice"] = self.voice
        if self.language is not None:
            payload["language"] = self.language
        if self.speech_id is not None:
            payload["speech_id"] = self.speech_id
        if self.extra:
            # Utterance-поля выигрывают — защита от случайного
            # переопределения ``ssml``/``sink`` через ``extra``.
            merged = dict(self.extra)
            merged.update(payload)
            payload = merged
        return payload


__all__ = [
    "DEFAULT_PRIORITY",
    "Sink",
    "Utterance",
    "escape_xml_text",
]
