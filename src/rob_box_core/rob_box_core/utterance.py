"""Utterance — реплика: голый текст, адресат и голос для всех запросов речи rob_box.

Контекст
--------
ADR-0080 §2.3: реплика собирается в **одном** модуле — продюсеры не лепят
JSON запроса сами. До ADR-0160 этот модуль обрамлял текст в ``<speak>`` с
XML-экранированием, а ``tts_node`` тут же снимал все теги: SSML был
конвертом, который никто не читал.

ADR-0160 §2.3 (PR-2, #4239): на проводе голый текст — слова и обычная
пунктуация, строки через ``
``. Разметку под свой сервис делает адаптер
провайдера голоса; конверт ``<speak>`` с экранированием собирает только
адаптер Silero (``rob_box_voice.tts_ssml.silero_ssml``).

Использование
-------------
::

    from rob_box_core.utterance import Utterance, Sink

    request = Utterance(
        text="Ошибка: x<y & z>0",
        sink=Sink.SPEAKERS,
        priority="operator",
    ).to_request()
    # {"text": "Ошибка: x<y & z>0", "sink": "speakers",
    #  "priority": "operator", "emotion": "neutral"}

Producer'ы дописывают свои поля (``request_id``, ``batch_id``, ``commands``)
на словарь из ``to_request()`` — контракт топиков держится в одном месте.

Не путать
---------
* ``Utterance`` — структура данных; **не** ROS-сообщение.
* ``Sink`` — адресат (динамики / наушники / preview), а не канал DDS
  (``/voice/speech/request`` vs ``/avatar/tts/request``); выбор топика остаётся
  на producer'е.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional


class Sink(str, Enum):
    """Адресат реплики (инвариант 6b: «Два выхода звука не смешиваются»).

    * ``SPEAKERS`` — реплика личности / пайплайна грипа, играется динамиками
      робота (``SpeechRequest`` в ``/voice/speech/request``).
    * ``HEADSET`` — собственная реплика ТАРС, играется только в наушники
      шлема оператора (``SpeechRequest(sink=headset)``, #4252).
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


@dataclass
class Utterance:
    """Одна реплика, готовая к упаковке в запрос речи.

    Поля ровно те, что определяет контракт очереди речи (см. ADR-0080 §1.3):
    текст, адресат, приоритет, голос, язык, эмоция и стабильный
    ``speech_id`` (если передан). Producer'ы дописывают свои поля на
    словарь из :meth:`to_request`.

    Attributes:
        text: Голый текст реплики (ADR-0160 §2.3): без SSML и без
            экранирования; строки через ``
``.
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
        role: Герой реплики (ADR-0160 §2.3) или ``""`` для рассказчика. Голос роль не читает:
            она нужна приёмникам текста (экран, чат).
    """

    text: str
    sink: Sink = Sink.SPEAKERS
    priority: Optional[str] = None
    voice: Optional[str] = None
    language: Optional[str] = None
    emotion: Optional[str] = None
    speech_id: Optional[str] = None
    role: str = ""
    # Доп. поля, которые producer хочет протащить в dict, но не считает
    # частью ``Utterance`` (``chunk``, ``commands``, ``batch_id``,
    # ``tg_chat_id`` и т. п.). Поля ``Utterance`` их перекрывают.
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

    def to_request(self) -> dict[str, Any]:
        """Словарь для ``json.dumps`` в payload ``/avatar/tts/request``.

        Гарантируется:

        * ``text`` — голый текст реплики как есть (ADR-0160 §2.3);
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
            "text": self.text or "",
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
        if self.role:
            payload["role"] = self.role
        if self.extra:
            # Utterance-поля выигрывают — защита от случайного
            # переопределения ``text``/``sink`` через ``extra``.
            merged = dict(self.extra)
            merged.update(payload)
            payload = merged
        return payload


#: Поля, без которых ``tts_node.dialogue_callback`` отбрасывает запрос
#: (лог «⚠ Chunk без текста», звука нет). Единый
#: источник правды для приёмника и продюсеров: 29.09.2026 инструмент ``say``
#: слал ``{"text": ...}`` без ``ssml`` и отчитывался success=True, хотя
#: робот молчал.
TTS_REQUEST_REQUIRED_FIELDS: tuple[str, ...] = ("text",)


def missing_tts_request_fields(payload: Any) -> list[str]:
    """Каких обязательных полей не хватает TTS-запросу (пусто — принят).

    Этой же функцией пользуется ``tts_node.dialogue_callback`` — поэтому
    тест продюсера, прогнавший свой payload через неё, проверяет ровно тот
    контракт, по которому приёмник решает «синтезировать / отбросить».

    >>> missing_tts_request_fields({"text": "a"})
    []
    >>> missing_tts_request_fields({"ssml": "<speak>a</speak>"})
    ['text']
    """
    if not isinstance(payload, dict):
        return list(TTS_REQUEST_REQUIRED_FIELDS)
    return [f for f in TTS_REQUEST_REQUIRED_FIELDS if f not in payload]


__all__ = [
    "ALLOWED_PRIORITIES",
    "DEFAULT_PRIORITY",
    "PRIORITY_BARGE_IN",
    "PRIORITY_NORMAL",
    "PRIORITY_OPERATOR",
    "PRIORITY_REPLACE",
    "Sink",
    "TTS_REQUEST_REQUIRED_FIELDS",
    "Utterance",
    "missing_tts_request_fields",
]
