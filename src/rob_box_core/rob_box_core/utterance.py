"""Utterance — единое место сборки SSML для всех TTS-запросов rob_box.

Контекст
--------
До появления ``Utterance`` каждый TTS-producer собирал SSML самостоятельно
через ``f"<speak>{text}</speak>"``. На сентябрь 2026 таких мест 10 в 5
пакетах (``rob_box_voice``, ``rob_box_supervisor``, ``rob_box_telegram``,
``rob_box_mcp_tools``), и ни одно не экранировало XML: текст с ``&`` /
``<`` / ``>`` ломал синтезатор или проходил со «съеденными» символами.

ADR-0080 §2.3 / §1.3, инвариант 5 (один канал сборки реплики) и 6
(экранирование — на стороне сборщика, не TTS) требуют, чтобы:

* SSML собирался в **одном** модуле, в **одном** месте;
* XML-экранирование выполнялось до вставки в ``<speak>...</speak>``;
* ``Utterance`` не знал ни о ROS, ни о producer'е — он только про текст
  и адресата (``sink``).

Использование
-------------
::

    from rob_box_core.utterance import Utterance, Sink

    request = Utterance(
        text="Ошибка: x<y & z>0",
        sink=Sink.SPEAKERS,
        priority="operator",
    ).to_request()
    # {"ssml": "<speak>Ошибка: x&lt;y &amp; z&gt;0</speak>",
    #  "sink": "speakers", "priority": "operator"}

Producer'ы дописывают свои поля (``speech_id``, ``emotion``, ``batch_id``,
``commands``) на словарь из ``to_request()`` — это и сохраняет существующий
контракт топиков, и держит экранирование централизованным.

Не путать
---------
* ``Utterance`` — структура данных; **не** ROS-сообщение.
* ``Sink`` — адресат (динамики / наушники / preview), а не канал DDS
  (``/voice/tts/request`` vs ``/avatar/tts/request``); выбор топика остаётся
  на producer'е.

Out of scope (см. voice-vr 13): единый вход ``tts_node``. Этот модуль только
про сборку запроса.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional


class Sink(str, Enum):
    """Адресат реплики (инвариант 6b: «Два выхода звука не смешиваются»).

    * ``SPEAKERS`` — реплика личности / пайплайна грипа, играется динамиками
      робота (``/voice/tts/request``).
    * ``HEADSET`` — собственная реплика ТАРС, играется только в наушники
      шлема оператора (``/avatar/tts/request``).
    * ``PREVIEW`` — не идёт ни в TTS-очередь, ни в шлем: уходит в
      ``/avatar/preview_voice`` (operator preview panel).
    """

    SPEAKERS = "speakers"
    HEADSET = "headset"
    PREVIEW = "preview"


# Допустимые значения ``priority`` для TTS-очереди. Это «публичный» словарь,
# потому что линейка ``tts_node`` (avatar-ветка) и dialogue-очередь читают
# его по-разному; единый enum защищает от опечаток на стороне producer'ов.
# (см. ADR-0021 и ``tts_node._PRIORITY_ORDER``).
DEFAULT_PRIORITY = "normal"
PRIORITY_NORMAL = "normal"
PRIORITY_OPERATOR = "operator"
PRIORITY_BARGE_IN = "barge_in"
PRIORITY_REPLACE = "replace"
ALLOWED_PRIORITIES = frozenset(
    {PRIORITY_NORMAL, PRIORITY_OPERATOR, PRIORITY_BARGE_IN, PRIORITY_REPLACE}
)


# Полный набор XML-управляющих символов, которые могут встретиться в
# пользовательском / LLM-тексте и которые ломают парсер SSML. Их
# экранирование обязательно; остальные символы (включая кавычки и
# апострофы) для текстового узла SSML безопасны.
_XML_ESCAPE_TABLE = str.maketrans(
    {
        "&": "&amp;",  # must run first
        "<": "&lt;",
        ">": "&gt;",
    }
)


def _escape_xml_text(text: str) -> str:
    """Экранировать ``&``, ``<``, ``>`` для вставки в текстовый узел SSML.

    Публичный re-export: ``escape_xml_text``. Нужен вызывающим, которые
    собирают НЕ-стандартный SSML (например, ``<prosody>...</prosody>`` внутри
    ``<speak>...</speak>``) и хотят ровно тот же escape, что в
    :attr:`Utterance.ssml`.

    >>> escape_xml_text("a & b < c > d")
    'a &amp; b &lt; c &gt; d'
    >>> escape_xml_text("plain")
    'plain'
    """
    return text.translate(_XML_ESCAPE_TABLE)


#: Публичный алиас для :func:`_escape_xml_text`.
escape_xml_text = _escape_xml_text


@dataclass
class Utterance:
    """Одна реплика, готовая к упаковке в TTS-запрос.

    Поля ровно те, что определяет контракт TTS-очереди (см. ADR-0080 §1.3):
    текст, адресат, приоритет, голос, язык, эмоция и стабильный
    ``speech_id`` (если передан). Producer'ы дописывают свои поля на
    словарь из :meth:`to_request`, не конкурируя за SSML.

    Attributes:
        text: Текст реплики. Любые ``&`` / ``<`` / ``>`` экранируются
            автоматически.
        sink: Куда играть (см. :class:`Sink`).
        priority: Приоритет TTS-очереди. ``None`` ⇒ ``"normal"``.
        voice: Имя голоса (``"ermil"``, ``"alena"`` …). ``None`` — дефолт
            провайдера.
        language: Код языка (``"ru-RU"``, ``"en-US"``). ``None`` —
            дефолт провайдера.
        emotion: Эмоция / анимация (``"neutral"``, ``"happy"`` …). ``None`` —
            ``"neutral"``.
        speech_id: Идентификатор реплики. ``None`` — producer сгенерирует
            свой (``uuid4``).
    """

    text: str
    sink: Sink = Sink.SPEAKERS
    priority: Optional[str] = None
    voice: Optional[str] = None
    language: Optional[str] = None
    emotion: Optional[str] = None
    speech_id: Optional[str] = None
    # Доп. поля, которые producer хочет протащить в dict, но не считает
    # частью ``Utterance`` (``chunk``, ``commands``, ``batch_id``,
    # ``tg_chat_id`` и т. п.). Применяются **после** SSML-сборки.
    extra: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.sink, Sink):
            # Разрешаем str-значения, чтобы зовущие с JSON-литералами
            # (``sink="speakers"``) не падали — Sink-Enum хранит те же
            # строки.
            self.sink = Sink(self.sink)
        if self.priority is not None and self.priority not in ALLOWED_PRIORITIES:
            raise ValueError(
                f"Utterance.priority must be one of {sorted(ALLOWED_PRIORITIES)}, "
                f"got {self.priority!r}"
            )
        if self.emotion is None:
            self.emotion = "neutral"

    @property
    def ssml(self) -> str:
        """Собранный и экранированный SSML — ``<speak>...</speak>``."""
        return f"<speak>{_escape_xml_text(self.text or '')}</speak>"

    def to_request(self) -> dict[str, Any]:
        """Словарь для ``json.dumps`` в payload ``/voice/tts/request`` или
        ``/avatar/tts/request``.

        Гарантируется:

        * ``ssml`` — единственный источник тегов ``<speak>``, экранирован;
        * ``sink`` — строка из :class:`Sink` (``"speakers"`` / ``"headset"``
          / ``"preview"``);
        * ``priority`` — строка (или ``"normal"`` по умолчанию);
        * ``emotion`` — строка (или ``"neutral"`` по умолчанию);
        * ``speech_id``, ``voice``, ``language`` присутствуют, только если
          заданы;
        * ``extra`` (если передан) сливается с приоритетом Utterance
          (поля Utterance выигрывают).
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
    "ALLOWED_PRIORITIES",
    "DEFAULT_PRIORITY",
    "PRIORITY_BARGE_IN",
    "PRIORITY_NORMAL",
    "PRIORITY_OPERATOR",
    "PRIORITY_REPLACE",
    "Sink",
    "Utterance",
]
